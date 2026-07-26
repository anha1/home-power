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

#include "/home/user/git/secrets/secrets.h" // Local credentials (ignored by Git)

#define WDT_TIMEOUT_S 120

#ifndef ESP_FIRMWARE_PASS
#error "ESP_FIRMWARE_PASS is not defined. Did you export the environment variable?"
#endif

// --- CONFIGURATION ---
const int NEOPIXEL_PIN = 21;            
              
const unsigned long SUBMIT_INTERVAL_MS = 5000;

WiFiUDP udp;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Initialize the library with your hardware profile:
// SilkyRMS(ADC_BITS, V_REF, VOLTS_TO_AMPS, AC_VOLTAGE)
SilkyRMS powerSensor(12, 3.3, 30.0, 230.0);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
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


void setup() {
    Serial.begin(115200);

    // Initialize the hardware pin and 12-bit resolution via the library

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
    powerSensor.begin_adc_native(1);
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

    if (samples > 0) {
        // 2. Fire the mathematical prime heartbeat
        uint16_t hue = samples * 1979; 

        // ColorHSV(hue, saturation, value)
        // 255 saturation = full color vibrancy
        // 255 value = maximum brightness
        uint32_t rgbColor = pixel.ColorHSV(hue, 255, 255);

        pixel.setPixelColor(0, rgbColor);
        pixel.show();


        // 4. Dispatch payloads
        send_statsd("esp32.power.watts", power_w);
        send_statsd("esp32.power.amps", irms);
        send_statsd("esp32.power.samples", samples);
        send_statsd("esp32.power.midpoint", current_offset);
        send_statsd("esp32.power.wifi_rssi", abs(WiFi.RSSI()));

        Serial.println(power_w);
    }

    // 5. Cleanup, Reset WDT, and go dark for the next measurement cycle
    if (WiFi.status() == WL_CONNECTED) {
        esp_task_wdt_reset(); 
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show();
    } else {
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
        pixel.show();
    }
    delay (1000);
} 