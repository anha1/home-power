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
#include <cmath>
#include <algorithm>

#include "/home/user/git/secrets/secrets.h" // Local credentials (ignored by Git)

#define WDT_TIMEOUT_S 120

#ifndef ESP_FIRMWARE_PASS
#error "ESP_FIRMWARE_PASS is not defined. Did you export the environment variable?"
#endif

// --- CONFIGURATION ---
const int NEOPIXEL_PIN = 21;            

// PZEM-004T v4 & Level Shifter pin configuration for ESP32-S3:
const int PZEM_RX_PIN = 43; // receives from PZEM TX via voltage divider
const int PZEM_TX_PIN = 44; // transmits directly to PZEM RX

const unsigned long SUBMIT_INTERVAL_MS = 5000;

int pzem_fail_count = 0;
int pzem_fail_state = 0;

#include <cmath>

class DynamicCalibrator {
private:
    float k;
    float offset;
    
    float alpha_k_base;
    float alpha_offset_base;
    float alpha_k_fast;
    float alpha_offset_fast;

    float soft_threshold;

    float certainty;           
    float cert_alpha;          

    // -- AC/DC Split Variables --
    bool is_initialized;
    float b_ema;
    float a_ema;
    float signal_ema_alpha; 

public:
    DynamicCalibrator(float initial_k = 1.02f, float initial_offset = -130.0f,
                      float lr_k_slow = 1e-7f, float lr_offset_slow = 1e-4f, 
                      float lr_k_fast = 1e-5f, 
                      float lr_offset_fast = 0.5f, // CRANKED UP: Snaps DC gap in ~4 ticks when uncertain
                      float tight_error_scale = 30.0f, 
                      float certainty_learning_rate = 0.05f,
                      float baseline_memory = 0.01f)
        : k(initial_k), offset(initial_offset), 
          alpha_k_base(lr_k_slow), alpha_offset_base(lr_offset_slow),
          alpha_k_fast(lr_k_fast), alpha_offset_fast(lr_offset_fast),
          soft_threshold(tight_error_scale), 
          certainty(0.0f), cert_alpha(certainty_learning_rate),
          is_initialized(false), b_ema(0.0f), a_ema(0.0f), signal_ema_alpha(baseline_memory) {}

    float predict(float b_raw) const {
        return (k * b_raw) - offset;
    }

    bool update(float b_raw, float a_val) {
        if (std::isnan(b_raw) || std::isnan(a_val)) {
            return false;
        }

        // Initialize the baseline trackers
        if (!is_initialized) {
            b_ema = b_raw;
            a_ema = a_val;
            is_initialized = true;
        } else {
            // Track the DC baseline
            b_ema = (1.0f - signal_ema_alpha) * b_ema + (signal_ema_alpha * b_raw);
            a_ema = (1.0f - signal_ema_alpha) * a_ema + (signal_ema_alpha * a_val);
        }

        float current_prediction = predict(b_raw);
        float error = a_val - current_prediction;

        // 1. REALITY CHECK
        float objective_error_ratio = error / soft_threshold;
        float objective_weight = 1.0f / (1.0f + (objective_error_ratio * objective_error_ratio));
        certainty = (1.0f - cert_alpha) * certainty + (cert_alpha * objective_weight);

        // 2. DYNAMIC LEARNING RATES
        float current_alpha_k = alpha_k_base + (alpha_k_fast - alpha_k_base) * (1.0f - certainty);
        float current_alpha_offset = alpha_offset_base + (alpha_offset_fast - alpha_offset_base) * (1.0f - certainty);

        // 3. LEARNING GATE
        float learning_error_ratio = (error * certainty) / soft_threshold;
        float learning_weight = 1.0f / (1.0f + (learning_error_ratio * learning_error_ratio));
        
        // 4. KINEMATICS (AC/DC Split)
        
        // DC Update for Offset
        float effective_error = error * learning_weight;
        offset -= current_alpha_offset * effective_error; 

        // AC Update for K
        float b_ac = b_raw - b_ema;
        float a_ac = a_val - a_ema;
        
        float error_ac = a_ac - (k * b_ac); 
        float effective_error_ac = error_ac * learning_weight;

        k += current_alpha_k * effective_error_ac * b_ac;

        // 5. HARD LIMITS
        // Clamp K firmly between 0.8 and 1.2
        k = std::max(0.8f, std::min(1.2f, k));

        return true;
    }

    void force_unlock() {
        certainty = 0.0f;
    }

    float get_k() const { return k; }
    float get_offset() const { return offset; }
    float get_certainty() const { return certainty; }
};

DynamicCalibrator calibrator;
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
    char line[128];
    
    if (value < 0) {
        // Bypass StatsD relative decrement by zeroing it first
        snprintf(line, sizeof(line), "%s:0|g\n%s:%.5f|g\n", metric, metric, value);
    } else {
        // Positive floats from snprintf have no leading '+', so they naturally set absolutely
        snprintf(line, sizeof(line), "%s:%.5f|g\n", metric, value);
    }
    
    int len = strlen(line);
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

void tryRecoverUART(bool isInitial) {
    if (!isInitial) {
        // 1. Flush any stuck bytes in rx/tx buffers
        Serial1.flush(); 

        // 2. Shut down the UART peripheral
        Serial1.end(); 

        // 3. Small delay to allow line idle states to settle
        vTaskDelay(pdMS_TO_TICKS(500));
    } 

    // 4. Re-initialize UART
    Serial1.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN); 
}


void setup() {
    Serial.begin(115200);


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

    tryRecoverUART(true);

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

    delay(4000);
    powerSensor.run(1000, &power_w, &irms);

    uint32_t samples = powerSensor.get_last_sample_count();
    double current_offset = powerSensor.get_dc_offset();


    uint16_t hue = samples * 1979; 
    uint32_t rgbColor = pixel.ColorHSV(hue, 255, 255);
    pixel.setPixelColor(0, pixel.Color(0, 0, 255));
    //pixel.setPixelColor(0, rgbColor);
    pixel.show();

    char statsd_buffer[1024] = "";

    // 1. SilkyRMS power sensor metrics (if samples > 0)
    if (samples > 0) {
        
        //add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.amps", irms);
        //add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.samples", (float)samples);
        //add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.midpoint", current_offset);
    } else {
        power_w = NAN;
    }

    // 2. Telemetry & uptime (independent of SilkyRMS samples)
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.wifi_rssi", abs(WiFi.RSSI()));
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.uptime", 10000 * log(millis()));


    float pzem_voltage = pzem.voltage();
    
    float pzem_power = pzem.power();
    float pzem_frequency = pzem.frequency();
    float pzem_pf = pzem.pf() * 100.0;


    if (isnan(pzem_voltage)) {
        pzem_fail_count++;
        pzem_fail_state++;
        if (pzem_fail_count % 7 == 0) {
            tryRecoverUART(false);
        }
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
        pixel.show();
    } else {
        if (pzem_fail_state > 0) pzem_fail_state--;
        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.voltage", pzem_voltage);
        if (!isnan(pzem_power))     add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.power", pzem_power);
        if (!isnan(pzem_frequency)) add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.frequency", pzem_frequency);
        if (!isnan(pzem_pf))        add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.pf", pzem_pf);

        // 1. Update calibration if Sensor A is healthy
        calibrator.update(power_w, pzem_power);


        pixel.setPixelColor(0, pixel.Color(0, 255, 255));
        pixel.show();
    }

    float power_w_calibrated = calibrator.predict(power_w);
    if (power_w_calibrated < 0) power_w_calibrated = 0;
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.power.watts", power_w_calibrated);
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.calibrator.k", calibrator.get_k());
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.calibrator.offset", calibrator.get_offset());
    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.calibrator.certainty", calibrator.get_certainty());

    add_statsd_metric(statsd_buffer, sizeof(statsd_buffer), "esp32.pzem.fail", (float)pzem_fail_state);


    // 4. Single-block UDP submit
    send_statsd_batch(statsd_buffer);

    // 5. Cleanup, Reset WDT, and go dark for the next measurement cycle
    if (WiFi.status() == WL_CONNECTED && pzem_fail_count < 32) {
        esp_task_wdt_reset(); 
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show();
    } else {
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
        pixel.show();
    }
} 