#include "control.hpp"
#include <math.h>
#include <algorithm>

// FreeRTOS Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "imu.h"       
#include "baro.h" 
#include "telemetry.h" 

static const char *TAG = "CONTROL";

// --- Constants ---
#define CONTROL_LOOP_FREQ_HZ 10 
#define CONTROL_LOOP_DELAY   (1000 / CONTROL_LOOP_FREQ_HZ)
#define DT                   (1.0f / CONTROL_LOOP_FREQ_HZ)

// --- Helper: PID Class ---
class PIDController {
public:
    float error_sum = 0.0f;
    float last_error = 0.0f;

    float compute(float target, float current, float kp, float ki, float kd) {
        float error = target - current;
        
        float p_term = kp * error;
        
        error_sum += error * DT;
        error_sum = std::max(-1.0f, std::min(1.0f, error_sum));
        float i_term = ki * error_sum;

        float d_term = kd * (error - last_error) / DT;
        last_error = error;

        float output = p_term + i_term + d_term;
        return std::max(-1.0f, std::min(1.0f, output));
    }

float compute_heading(float target, float current, float kp, float ki, float kd) {
        float error = target - current;

        // Normalize error to -180 to +180
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        float p_term = kp * error;
        
        error_sum += error * DT;
        // Clamp integral
        error_sum = std::max(-1.0f, std::min(1.0f, error_sum));
        float i_term = ki * error_sum;

        float d_term = kd * (error - last_error) / DT;
        last_error = error;

        // --- FIX: Calculate total output ---
        float output = p_term + i_term + d_term; 

        return std::max(-1.0f, std::min(1.0f, output)); 
    }
};

// --- Helper: Clamp Thruster Values ---
float clamp(float value) {
    return std::max(-1.0f, std::min(1.0f, value));
}

void control_loop_task(void *arg) {

    imu_shared_data_t imu_data;
    barometer_shared_data_t baro_data;
    robot_shared_state_t robot_state;

    // Initialize State
    robot_state.config.arm = false;
    robot_state.config.kp = 0.01f; robot_state.config.ki = 0.0f; robot_state.config.kd = 0.0f;
    robot_state.target.v = 0.0f; robot_state.target.w = 0.0f; robot_state.target.h = 0.0f; 

    // Initialize PIDs
    PIDController pid_roll;
    PIDController pid_heading;
    PIDController pid_height;

    // State tracking
    float current_target_heading = 0.0f;
    bool heading_initialized = false;

    ESP_LOGI(TAG, "Control Loop Started at %d Hz", CONTROL_LOOP_FREQ_HZ);

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // 1. Get latest Data
        telemetry_get_state(&robot_state); // Updates targets (v, w, h)
        bool imu_ok = imu_get_data(&imu_data);
        bool baro_ok = barometer_get_data(&baro_data);

        if (imu_ok && baro_ok) {
            
            // --- A. Heading Logic ---
            // If this is the first loop, lock the current heading as target
            if (!heading_initialized) {
                current_target_heading = imu_data.heading;
                heading_initialized = true;
            }

            // If user provides 'w' (angular rotation input), rotate the target heading
            // w is assumed to be degrees per second here
            current_target_heading += robot_state.target.w * DT;
            
            // Normalize target heading to 0-360 range for safety
            if(current_target_heading > 360.0f) current_target_heading -= 360.0f;
            if(current_target_heading < 0.0f) current_target_heading += 360.0f;

            // --- B. Height Logic ---
            // Convert raw pressure to approximate height (simplification)
            // You typically need a baseline pressure subtraction here.
            // Assuming hx1_raw scales somewhat linearly with depth for this PID example.
            float current_height = (float)baro_data.depth; 

            // --- C. PID Calculations ---
            
            // 1. Heading PID (Controls Yaw torque)
            float yaw_output = pid_heading.compute_heading(
                current_target_heading, 
                imu_data.heading, 
                robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
            );

            // 2. Height PID (Controls Heave force)
            float heave_output = pid_height.compute(
                robot_state.target.h, 
                current_height, 
                robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
            );

            // 3. Roll PID (Stabilize to 0)
            float roll_output = pid_roll.compute(
                0.0f,             // Target Roll is always level (0)
                imu_data.roll, 
                robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
            );

            // --- D. Thruster Mixing ---
            float t[6]; // Stores thruster values -1 to +1

            float surge = robot_state.target.v;

            // Horizontal Thrusters (1, 2, 3, 4)
            // Assuming X-Configuration or Left/Right differential setup
            // Left Side (e.g., 1 & 3): Surge + Turn
            // Right Side (e.g., 2 & 4): Surge - Turn
            // (Adjust signs based on your physical motor wiring)
            t[0] = clamp(surge + yaw_output); // T1
            t[1] = clamp(surge - yaw_output); // T2
            t[2] = clamp(surge + yaw_output); // T3
            t[3] = clamp(surge - yaw_output); // T4

            // Vertical Thrusters (5, 6)
            // 5 = Left, 6 = Right
            // Heave moves both up. Roll moves one up, one down.
            t[4] = clamp(heave_output + roll_output); // T5
            t[5] = clamp(heave_output - roll_output); // T6

            // --- E. Output ---
            ESP_LOGI(TAG, "T: [%.2f, %.2f, %.2f, %.2f] | V: [%.2f, %.2f] | HeadErr: %.1f | RollErr: %.1f", 
                t[0], t[1], t[2], t[3], 
                t[4], t[5], 
                (current_target_heading - imu_data.heading), 
                (0.0f - imu_data.roll)
            );

        } else {
            ESP_LOGW(TAG, "Sensor Error: Waiting for IMU/Baro...");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_DELAY));
    }
}