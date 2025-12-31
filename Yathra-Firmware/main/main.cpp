#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

#include "util_i2c.hpp"
#include "thruster.hpp"
#include "src/imu.h" 
#include "src/telemetry.h"
#include "src/baro.h"
#include "control.hpp"

static const char* TAG = "SUB_LOG";

static imu_task_config_t imu_cfg = { .enable_mag = false };
 std::vector<int> thruster_pins = {26, 25, 33, 32, 23, 4};
ThrusterController vehicle;

extern "C" void app_main() { 

    ESP_LOGI(TAG, "Starting application");

    i2c_util::i2c_init();
    i2c_util::i2c_scan();
    imu_init(); 
    xTaskCreatePinnedToCore(imu_task, "IMU", 4096, (void *)&imu_cfg,19, NULL, 1);
    
    barometer_init();
    xTaskCreatePinnedToCore(barometer_task, "BARO", 3072, NULL,5, NULL, 0);

    // telemetry_init(); 
    // xTaskCreatePinnedToCore(telemetry_rx_task, "TEL_RX", 3072, NULL,10, NULL, 0);
    // xTaskCreatePinnedToCore(telemetry_tx_task, "TEL_TX", 2048, NULL,3, NULL, 0);

    xTaskCreatePinnedToCore(control_loop_task, "CONTROL", 4096, NULL,20, NULL, 1);
    // vehicle.init(thruster_pins);
    // vehicle.stopAll();
    // vTaskDelay(pdMS_TO_TICKS(3000)); 



    while (true) {

        // std::vector<float> speeds(8, 20.0f);
        // vehicle.setSpeeds(speeds);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sample rate for printing
    }
}

//   Found device at: 0x0d