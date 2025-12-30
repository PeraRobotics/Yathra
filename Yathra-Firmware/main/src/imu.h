#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern "C" {
    #include "ahrs.h"
    #include "mpu9250.h"
    #include "calibrate.h"
    #include "common.h"
}

// 1. Configuration struct for the task
typedef struct {
    bool enable_mag;
} imu_task_config_t;

// 2. Shared Data Structure (Public to all)
typedef struct {
    // Raw Sensor Data (for microROS / debugging)
    vector_t accel;
    vector_t gyro;
    vector_t mag;
    
    // Filtered Orientation (for Control Loop)
    float roll;
    float pitch;
    float heading;
    
    // System Status
    float temp_c;
} imu_shared_data_t;

/**
 * @brief Initialize the IMU system (Mutexes, etc.)
 */
void imu_init(void);

/**
 * @brief The FreeRTOS task entry point
 */
void imu_task(void *arg);

/**
 * @brief Thread-safe getter. Call this from ANY task (Main, Control, ROS).
 * @param out_data Pointer to the struct to fill with the latest data.
 * @return true if data was read successfully, false if mutex timeout.
 */
bool imu_get_data(imu_shared_data_t *out_data);