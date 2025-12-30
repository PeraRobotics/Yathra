#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hx711.h" // Include the library header

static const char *TAG = "HX710B_App";

// Pin Config
#define DOUT_GPIO GPIO_NUM_19
#define SCK_GPIO  GPIO_NUM_18

extern "C" void app_main(void)
{
    // 1. Initialize the Device Object
    hx711_t dev = {
        .dout = DOUT_GPIO,
        .pd_sck = SCK_GPIO,
        .gain = HX711_GAIN_A_128 // Sends 25 pulses (Sets HX711_GAIN_A_64 - 10Hz , HX711_GAIN_A_128 - 40Hz)
    };

    // 2. Initialize Hardware
    // This handles GPIO setup and internal timing automatically
    esp_err_t err = hx711_init(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init HX710B");
        return;
    }

    ESP_LOGI(TAG, "HX710B Initialized (Using HX711 Driver)");

    while (1)
    {
        // 3. Wait for sensor to be ready (TIMEOUT: 1000ms)
        // This replaces the manual 'while(gpio_get_level...)' loop
        esp_err_t r = hx711_wait(&dev, 1000);
        
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "Sensor not found (Timeout)");
        } else {
            int32_t data;
            // 4. Read Data
            r = hx711_read_data(&dev, &data);
            if (r == ESP_OK) {
                ESP_LOGI(TAG, "Raw: %ld", data);
            } else {
                ESP_LOGE(TAG, "Read Error");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

