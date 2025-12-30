#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

// Include your custom headers
#include "src/imu.h" 
#include "src/telemetry.h"


static const char* TAG = "SUB_LOG";

void control_loop_task(void *arg) {
    imu_shared_data_t imu_data;
    command_shared_data_t cmd_data; // Local struct to hold received commands
    
    while(1) {
        // Get Sensor Data
        if (imu_get_data(&imu_data)) {
            // Processing...
        }

        // Get Command Data using the new Getter
        if (telemetry_get_command(&cmd_data)) {
            // Check if system is enabled
            if (cmd_data.system_enable) {
                // Run control logic using cmd_data.target_heading, etc.
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() { 
    // Initialize modules
    imu_init(); 
    telemetry_init(); // Initializes UART and Mutex

    static imu_task_config_t imu_cfg = { .enable_mag = false };

    // Create Tasks
    xTaskCreate(imu_task, "imu_task", 4096, (void *)&imu_cfg, 10, NULL);
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_loop_task, "control_task", 4096, NULL, 5, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}