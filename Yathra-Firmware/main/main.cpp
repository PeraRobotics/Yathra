#include "util_i2c_scanner.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


extern "C" void app_main() { 
    static const char* TAG = "main";

    ESP_LOGI(TAG, "Starting application");

    i2c_scanner::I2cScanner scanner({
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .i2c_port = I2C_NUM_0,
        .clk_speed = 100000
    });

    scanner.scan();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        scanner.scan();
    }
}

