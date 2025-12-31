#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

#include "util_i2c.hpp"
#include "src/imu.h" 
#include "src/telemetry.h"
#include "src/baro.h"

static const char* TAG = "SUB_LOG";

void control_loop_task(void *arg) {

    // robot_shared_state_t current_state;

    // while(1) {
    //     // 1. Update State (Thread Safe)
    //     telemetry_get_state(&current_state);

    //     // 2. Use Config
    //     if (current_state.config.arm) {
    //         float err = current_state.target.h - current_height;
    //         float output = err * current_state.config.kp;
            
    //         // Check motor reverse config
    //         if (current_state.config.motor_rev[0]) {
    //              // reverse motor 1 logic
    //         }
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }


    imu_shared_data_t imu_data;
    barometer_shared_data_t baro_data;
    while(1) {
        // Get Sensor Data
        if (imu_get_data(&imu_data)) {
        //     ESP_LOGI(TAG, "IMU Data - Heading: %.2f Roll: %.2f Pitch: %.2f | Accel: [%.2f, %.2f, %.2f] | Gyro: [%.2f, %.2f, %.2f] | Mag: [%.2f, %.2f, %.2f]", 
        //              imu_data.heading, imu_data.roll, imu_data.pitch,   
        //              imu_data.accel.x, imu_data.accel.y, imu_data.accel.z,
        //              imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z,
        //              imu_data.mag.x, imu_data.mag.y, imu_data.mag.z);
        // }
        }
        
        if (barometer_get_data(&baro_data)) {
        //  ESP_LOGI(TAG, "Barometer Data - HX1 Raw: %ld HX2 Raw: %ld", 
        //           baro_data.hx1_raw, baro_data.hx2_raw);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// // --- I2C ---
// #define I2C_SDA_PIN  21
// #define I2C_SCL_PIN  22

// // --- HX711 Load Cells ---
// // HX711 1
// #define HX1_DT_PIN   34  // Input Only pin
// #define HX1_SCK_PIN  4
// // HX711 2
// #define HX2_DT_PIN   35  // Input Only pin
// #define HX2_SCK_PIN  5

// // --- ESCs (Motors) ---
// // We can put these in an array for easy iteration
// const int esc_pins[8] = {13, 18, 19, 23, 25, 26, 27, 32};




static imu_task_config_t imu_cfg = { .enable_mag = false };
extern "C" void app_main() { 

    ESP_LOGI(TAG, "Starting application");

    i2c_util::i2c_init();
    i2c_util::i2c_scan();

    // Initialize modules
    imu_init(); 
    telemetry_init(); 
    barometer_init();
    

    xTaskCreatePinnedToCore(control_loop_task, "CONTROL", 4096, NULL,20, NULL, 1);
    xTaskCreatePinnedToCore(imu_task, "IMU", 4096, NULL,19, NULL, 1);

    xTaskCreatePinnedToCore(telemetry_rx_task, "TEL_RX", 3072, NULL,10, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_tx_task, "TEL_TX", 2048, NULL,3, NULL, 0);
    xTaskCreatePinnedToCore(barometer_task, "BARO", 3072, NULL,5, NULL, 0);

    while (true) {

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sample rate for printing
    }
}

//   Found device at: 0x0d