#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include "util_i2c_scanner.hpp"
// Include your custom headers
#include "src/imu.h" 
#include "src/telemetry.h"
#include <string.h>
#include "driver/i2c.h"



static const char* TAG = "SUB_LOG";

void control_loop_task(void *arg) {
    imu_shared_data_t imu_data;
    // command_shared_data_t cmd_data; // Local struct to hold received commands
    
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
        // if (telemetry_get_command(&cmd_data)) {
        //     // Check if system is enabled
        //     if (cmd_data.system_enable) {
        //         // Run control logic using cmd_data.target_heading, etc.
        //     }
        // }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ   100000



extern "C" void app_main() { 
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));


    ESP_LOGI(TAG, "Starting application");
    // Initialize modules
    imu_init(); 
    telemetry_init(); // Initializes UART and Mutex

    static imu_task_config_t imu_cfg = { .enable_mag = false };
    xTaskCreate(imu_task, "imu_task", 4096, (void *)&imu_cfg, 10, NULL);

    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_loop_task, "control_task", 4096, NULL, 5, NULL);

    // qmc5883l_init();
    // int16_t mag_x, mag_y, mag_z;

    while (true) {
        // Read QMC5883L
        // qmc5883l_read_mag(&mag_x, &mag_y, &mag_z);
        
        // Print Data
        // printf("Magnetometer: X=%d Y=%d Z=%d\n", mag_x, mag_y, mag_z);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz sample rate for printing
    }
}

//   Found device at: 0x0d