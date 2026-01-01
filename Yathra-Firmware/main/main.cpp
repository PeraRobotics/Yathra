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
 std::vector<int> thruster_pins = { 32, 33, 25, 26, 23, 4};
ThrusterController vehicle;

extern "C" void app_main() { 

    ESP_LOGI(TAG, "Starting application");

    i2c_util::i2c_init();
    i2c_util::i2c_scan();
    imu_init(); 
    xTaskCreatePinnedToCore(imu_task, "IMU", 4096, (void *)&imu_cfg,20, NULL, 0);
    
    barometer_init();
    xTaskCreatePinnedToCore(barometer_task, "BARO", 3072, NULL,10, NULL, 0);

    // telemetry_init(); 
    // xTaskCreatePinnedToCore(telemetry_rx_task, "TEL_RX", 3072, NULL,10, NULL, 0);
    // xTaskCreatePinnedToCore(telemetry_tx_task, "TEL_TX", 2048, NULL,3, NULL, 0);

    xTaskCreatePinnedToCore(control_loop_task, "CONTROL", 4096, NULL,20, NULL, 1);
    
    
    // vehicle.init(thruster_pins);
    // vehicle.stopAll();
    // vTaskDelay(pdMS_TO_TICKS(3000)); 



    // const int num_motors = 6;
    // const int test_duration = 2000; // Time to run each test in ms (2 seconds)

    // Loop through each motor index
    // for (int i = 0; i < num_motors; i++) {
        
    //     // 1. Initialize a vector of zeros (stops all other motors)
    //     std::vector<float> test_speeds(num_motors, 0.0f);

    //     // --- TEST FORWARD (0.5) ---
    //     test_speeds[i] = 0.2f; 
    //     ESP_LOGI("MOTOR_TEST", "Testing Motor %d -> Speed: 0.5", i);
    //     vehicle.setSpeeds(test_speeds);
    //     vTaskDelay(pdMS_TO_TICKS(test_duration));
        
    //     test_speeds[i] = 0.0f; 
    //     vehicle.setSpeeds(test_speeds);
    //     vTaskDelay(pdMS_TO_TICKS(test_duration));

    //     // --- TEST REVERSE (-0.5) ---
    //     test_speeds[i] = -0.2f;
    //     ESP_LOGI("MOTOR_TEST", "Testing Motor %d -> Speed: -0.5", i);
    //     vehicle.setSpeeds(test_speeds);
    //     vTaskDelay(pdMS_TO_TICKS(test_duration));

    //     // --- STOP (0.0) ---
    //     // Brief stop before moving to the next motor to protect mechanics
    //     test_speeds[i] = 0.0f;
    //     vehicle.setSpeeds(test_speeds);
    //     vTaskDelay(pdMS_TO_TICKS(500)); 
    // }

    // Ensure everything is stopped at the end of the full sequence
    // std::vector<float> all_stop(num_motors, 0.0f);
    // vehicle.setSpeeds(all_stop);
    // ESP_LOGI("MOTOR_TEST", "Test Sequence Complete.");
    // vTaskDelay(pdMS_TO_TICKS(3000)); 
    while (true) {

        // std::vector<float> speeds = {0.5f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
        // vehicle.setSpeeds(speeds);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sample rate for printing
    }
}

//   Found device at: 0x0d