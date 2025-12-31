#include "baro.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "src/telemetry.h"
#include <iostream>

extern "C" {
    #include "hx711.h"
}

// 0cm -> -878756
// 5cm -> -113575
// 10cm -> -140500
// 15cm -> -170936
// 20cm -> -200300
// 25cm -> -227200
// 30cm -> -247000

static const char *TAG = "BARO_METER";

// --- Global Variables ---
static SemaphoreHandle_t xBarometerMutex = NULL;
static barometer_shared_data_t g_sensor_data; 

static hx711_t dev_hx1;
static hx711_t dev_hx2;

const long SURFACE_OFFSET = -878756;
const float UNITS_PER_CM = -55730.0f;
const long SURFACE_THRESHOLD = -850000;

float getDepth(long raw_reading) {
    if (raw_reading > SURFACE_THRESHOLD) {
        return 0.0f;
    }

    // Linear equation: y = mx + c
float depth_magnitude = (raw_reading - SURFACE_OFFSET) / UNITS_PER_CM;
    // Optional: Clamp to 0 if noise makes it slightly negative in air
  return -depth_magnitude;
}

// --- Task Function (Must match the name in header) ---
void barometer_task(void *pvParameters)
{
    // Initialize HX1
    dev_hx1.dout = HX1_DT_PIN;
    dev_hx1.pd_sck = HX1_SCK_PIN;
    dev_hx1.gain = HX711_GAIN_A_128;
    hx711_init(&dev_hx1);

    // Initialize HX2
    dev_hx2.dout = HX2_DT_PIN;
    dev_hx2.pd_sck = HX2_SCK_PIN;
    dev_hx2.gain = HX711_GAIN_A_128;
    hx711_init(&dev_hx2);

    while (1) {
        int32_t val1 = 0, val2 = 0;
        esp_err_t err1 = ESP_FAIL, err2 = ESP_FAIL;

        if (hx711_wait(&dev_hx1, 500) == ESP_OK) {
            err1 = hx711_read_data(&dev_hx1, &val1);
        }

        if (hx711_wait(&dev_hx2, 500) == ESP_OK) {
            err2 = hx711_read_data(&dev_hx2, &val2);
        }

        if (xBarometerMutex != NULL) {
             if (xSemaphoreTake(xBarometerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (err1 == ESP_OK) g_sensor_data.hx1_raw = val1;
                if (err2 == ESP_OK) g_sensor_data.hx2_raw = val2;    
                g_sensor_data.depth = getDepth(g_sensor_data.hx1_raw);  
                xSemaphoreGive(xBarometerMutex);
            }
        }

        baro_msg_t baro_data;
        baro_data.pressure_out = (float)g_sensor_data.hx1_raw;
        baro_data.pressure_in = (float)g_sensor_data.hx2_raw;
        baro_data.depth = g_sensor_data.depth; 
        telemetry_send_baro(&baro_data);

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// --- Init Function (Only creates Mutex now) ---
bool barometer_init(void)
{
    xBarometerMutex = xSemaphoreCreateMutex();
    if (xBarometerMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }
    return true;
}

// --- Getter Function ---
bool barometer_get_data(barometer_shared_data_t *out_data)
{
    if (xBarometerMutex == NULL) return false;

    if (xSemaphoreTake(xBarometerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *out_data = g_sensor_data;
        xSemaphoreGive(xBarometerMutex);
        return true;
    }
    return false;
}