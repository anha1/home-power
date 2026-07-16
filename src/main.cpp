#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_sntp.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <Adafruit_NeoPixel.h>

#include "/home/user/git/secrets/secrets.h" // Local credentials (ignored by Git)


#define WDT_TIMEOUT_S           120

#ifndef ESP_FIRMWARE_PASS
#error "ESP_FIRMWARE_PASS is not defined. Did you export the environment variable?"
#endif



// --- CONFIGURATION ---
const int NEOPIXEL_PIN = 21;
const int NUMPIXELS = 1;
const int SENSOR_PIN = 4;
const float V_REF = 3.3;               
const int ADC_MAX = 4095;              
const float VOLTS_TO_AMPS = 30.0;      
const float AC_VOLTAGE = 230.0;        

// --- MEASUREMENT STATE ---
double dc_offset = 2048.0; 
const double OFFSET_ALPHA = 0.001;

// --- ACCUMULATORS ---
uint32_t sum_raw = 0;
uint64_t sum_sq_raw = 0; 
uint32_t sample_count = 0;
unsigned long last_submit_time = 0;
const unsigned long SUBMIT_INTERVAL_MS = 5000;
const int SAMPLES_PER_YIELD = 199;

WiFiUDP udp;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASSWORD;
const char* firmware_upload_password = ESP_FIRMWARE_PASS;

const char* statsd_host = "10.1.1.69"; 
const uint16_t statsd_port = 8125;

void send_statsd(const char* metric, float value) {
    if (WiFi.status() != WL_CONNECTED) return;
    
    char payload[64];
    snprintf(payload, sizeof(payload), "%s:%.5f|g", metric, value);
    
    udp.beginPacket(statsd_host, statsd_port);
    udp.print(payload);
    udp.endPacket();
}

void otaTask(void *pvParameters) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.handle();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void initOTA() {
    ArduinoOTA.setHostname("esp32-power");
    ArduinoOTA.setPort(1726); 
    ArduinoOTA.setPassword(firmware_upload_password); 

    ArduinoOTA.onStart([]() {
        Serial.println("\nOTA Update Started...");
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA Update Finished. Rebooting...");
        delay(1000); 
        WiFi.disconnect();
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        esp_task_wdt_reset(); 
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    xTaskCreate(otaTask, "OTATask", 4096, NULL, 1, NULL);
}

void calibrateOffset() {
    unsigned long sum = 0;
    const int num_samples = 10000;
    
    for (int i = 0; i < num_samples; i++) {
        sum += analogRead(SENSOR_PIN);
    }
    
    dc_offset = (double)sum / num_samples;
}

void setup() {
    Serial.begin(115200);

    analogReadResolution(12);

    pixel.begin();

    pixel.setBrightness(20); 

    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true 
    };
    esp_task_wdt_reconfigure(&twdt_config);
    esp_task_wdt_add(NULL);

    calibrateOffset();
    esp_task_wdt_reset(); 

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    initOTA();

    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();
    
    // Initialize the timing state machine
    last_submit_time = millis();
}

void loop() {
    // 1. Read a batch of samples rapidly to maintain AC waveform resolution
    for (int i = 0; i < SAMPLES_PER_YIELD; i++) {
        uint32_t raw = analogRead(SENSOR_PIN);
        sum_raw += raw;
        sum_sq_raw += (raw * raw);
        sample_count++;
    }

    unsigned long now = millis();
    if (now - last_submit_time >= SUBMIT_INTERVAL_MS) {
        if (sample_count > 0) {
            uint8_t r = (sample_count * 197) % 251;
            uint8_t g = (sample_count * 311) % 251;
            uint8_t b = (sample_count * 439) % 251;

            pixel.setPixelColor(0, pixel.Color(r, g, b));
            pixel.show();

            double batch_mean = (double)sum_raw / sample_count;
            dc_offset += (batch_mean - dc_offset) * OFFSET_ALPHA;

            double sum_raw_d = (double)sum_raw;
            double sum_sq = (double)sum_sq_raw - ((sum_raw_d * sum_raw_d) / sample_count);
            
            double rms_adc = sqrt(sum_sq / sample_count);
            double rms_volts = rms_adc * (V_REF / ADC_MAX);
            double irms = rms_volts * VOLTS_TO_AMPS;
            double power_w = irms * AC_VOLTAGE;

            send_statsd("esp32.power.watts", power_w);
            send_statsd("esp32.power.amps", irms);
            send_statsd("esp32.power.samples", sample_count);
            send_statsd("esp32.power.midpoint", dc_offset);
            send_statsd("esp32.power.wifi_rssi", abs(WiFi.RSSI()));

            Serial.println(power_w);
        }

        // Reset accumulators for the next window
        sum_raw = 0;
        sum_sq_raw = 0;
        sample_count = 0;
        last_submit_time = now;

        if (WiFi.status() == WL_CONNECTED) {
            esp_task_wdt_reset(); 
            pixel.setPixelColor(0, pixel.Color(0, 0, 0));
            pixel.show();
        } else {
            pixel.setPixelColor(0, pixel.Color(255, 0, 0));
            pixel.show();
        }
    }

    // Mandatory context switch.
    delay(2);
    
}