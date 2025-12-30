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
    imu_shared_data_t imu_data;
    command_shared_data_t cmd_data;
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

        // Get Command Data using the new Getter
        if (telemetry_get_command(&cmd_data)) {
            // Check if system is enabled
            // if (cmd_data.system_enable) {
                // Run control logic using cmd_data.target_heading, etc.
            // }
        }
        
        if (barometer_get_data(&baro_data)) {
         ESP_LOGI(TAG, "Barometer Data - HX1 Raw: %ld HX2 Raw: %ld", 
                  baro_data.hx1_raw, baro_data.hx2_raw);
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

#define DOUT_GPIO GPIO_NUM_34
#define SCK_GPIO  GPIO_NUM_4


extern "C" void app_main() { 

    ESP_LOGI(TAG, "Starting application");
    // i2c_util::i2c_init();
    // i2c_util::i2c_scan();

    // Initialize modules
    // imu_init(); 
    // telemetry_init(); 
    barometer_init();
    
    // static imu_task_config_t imu_cfg = { .enable_mag = false };
    // xTaskCreate(imu_task, "imu_task", 4096, (void *)&imu_cfg, 10, NULL);

    xTaskCreate(barometer_task, "HX_Task", 4096, NULL, 5, NULL);
    
    xTaskCreate(control_loop_task, "control_task", 4096, NULL, 5, NULL);
    
    // xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);

    while (true) {

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sample rate for printing
    }
}

//   Found device at: 0x0d