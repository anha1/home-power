#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_sntp.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <Adafruit_NeoPixel.h>
#include <SilkyRMS.h>
#include <PZEM004Tv30.h>

#include "/home/user/git/secrets/secrets.h" // Local credentials (ignored by Git)

#define WDT_TIMEOUT_S 120

#ifndef ESP_FIRMWARE_PASS
#error "ESP_FIRMWARE_PASS is not defined. Did you export the environment variable?"
#endif

// --- CONFIGURATION ---
const int NEOPIXEL_PIN = 21;            

// PZEM-004T v4 & Level Shifter pin configuration for ESP32-S3:
const int PZEM_RX_PIN = 12; // receives from PZEM TX via voltage divider
const int PZEM_TX_PIN = 9; // transmits directly to PZEM RX

const unsigned long SUBMIT_INTERVAL_MS = 5000;
int pzem_success_count = 0;
int pzem_fail_count = 0;

// Exponential Moving Average (EMA) for smooth real-time success ratio
float pzem_ratio = 100.0f;
const float EMA_ALPHA = 0.05f; // ~20-sample smoothing factor

void update_pzem_ratio(bool success) {
    float current_val = success ? 100.0f : 0.0f;
    pzem_ratio = (pzem_ratio * (1.0f - EMA_ALPHA)) + (current_val * EMA_ALPHA);
}

WiFiUDP udp;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Initialize the library with your hardware profile:
// SilkyRMS(ADC_BITS, V_REF, VOLTS_TO_AMPS, AC_VOLTAGE)
SilkyRMS powerSensor(12, 3.3, 30.0, 230.0);

// Hardware serial 1 for PZEM-004T v4
PZEM004Tv30 pzem(Serial1, PZEM_RX_PIN, PZEM_TX_PIN);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* firmware_upload_password = ESP_FIRMWARE_PASS;

const char* statsd_host = "10.1.1.69"; 
const uint16_t statsd_port = 8125;

void add_statsd_metric(char* buffer, size_t max_len, const char* metric, float value) {
    char line[64];
    int len = snprintf(line, sizeof(line), "%s:%.5f|g\n", metric, value);
    if (len > 0 && (strlen(buffer) + len < max_len)) {
        strcat(buffer, line);
    }
}

void send_statsd_batch(const char* payload) {
    if (WiFi.status() != WL_CONNECTED || payload == NULL || payload[0] == '\0') return;
    
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
    // Pin OTA task to Core 0 (PRO_CPU), leaving Core 1 (APP_CPU) dedicated to main loop & sensors
    xTaskCreatePinnedToCore(otaTask, "OTATask", 4096, NULL, 1, NULL, 0);
}


void setup() {
    Serial.begin(115200);

    // Explicitly initialize HardwareSerial 1 inside setup for ESP32-S3
    Serial1.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);

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

    esp_task_wdt_reset(); 

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    initOTA();


    esp_task_wdt_reset(); 
    powerSensor.begin_adc_native(4);
    powerSensor.calibrate_offset();
    esp_task_wdt_reset(); 

    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();
}

void loop() {
    double power_w = 0.0;
    double irms = 0.0;

    powerSensor.run(3000, &power_w, &irms);

    uint32_t samples = powerSensor.get_last_sample_count();
    double current_offset = powerSensor.get_dc_offset();


    uint16_t hue = samples * 1979; 
    uint32_t rgbColor = pixel.ColorHSV(hue, 255, 255);
    pixel.setPixelColor(0, rgbColor);
    pixel.show();

    char statsd_buffer[1024] = "";

    // 1. SilkyRMS power sensor metrics (if samples > 0)
    if (samples > 0) {
        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.watts", power_w);
        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.amps", irms);
        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.samples", (float)samples);
        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.midpoint", current_offset);
    }

    // 2. Telemetry & uptime (independent of SilkyRMS samples)
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.wifi_rssi", abs(WiFi.RSSI()));
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.uptime", millis() / 1000.0f);


    float pzem_voltage = pzem.voltage();
    
    float pzem_power = pzem.power();
    float pzem_frequency = pzem.frequency();
    float pzem_pf = pzem.pf() * 100.0;

    if (isnan(pzem_voltage)) {
        pzem_fail_count++;
        update_pzem_ratio(false);
    } else {
        pzem_success_count++;
        update_pzem_ratio(true);
        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.voltage", pzem_voltage);
        if (!isnan(pzem_power))     add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.power", pzem_power);
        if (!isnan(pzem_frequency)) add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.frequency", pzem_frequency);
        if (!isnan(pzem_pf))        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.pf", pzem_pf);
    }

    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.success", (float)pzem_success_count);
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.fail", (float)pzem_fail_count);
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.ratio", pzem_ratio);

    // 4. Single-block UDP submit
    send_statsd_batch(statsd_buffer);

    // 5. Cleanup, Reset WDT, and go dark for the next measurement cycle
    if (WiFi.status() == WL_CONNECTED) {
        esp_task_wdt_reset(); 
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show();
    } else {
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
        pixel.show();
    }
    delay(1000);
} 