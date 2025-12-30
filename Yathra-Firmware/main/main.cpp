#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "src/imu.h" // Include your new header

static const char* TAG = "MAIN";

void control_loop_task(void *arg) {
    imu_shared_data_t current_imu;
    
    while(1) {
        // Get the latest data safely
        if (imu_get_data(&current_imu)) {
            ESP_LOGI(TAG,"Current Attitude | Heading: %f, Pitch: %f, Roll: %f, Temp: %f", current_imu.heading, current_imu.pitch, current_imu.roll, current_imu.temp_c);
        }
        
        // Run control loop at 100Hz (10ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() { 
    
    imu_init();

    static imu_task_config_t imu_cfg = { .enable_mag = false };
    xTaskCreate(imu_task, "imu_task", 4096, (void *)&imu_cfg, 10, NULL);

    xTaskCreate(control_loop_task, "control_task", 4096, NULL, 5, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}