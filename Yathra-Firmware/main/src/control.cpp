#include "control.hpp"
#include <math.h>
#include <algorithm>
#include <vector>

// FreeRTOS Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Custom Includes (Assumed these exist per your snippet)
#include "imu.h"       
#include "baro.h" 
#include "telemetry.h" 
#include "thruster.hpp"



static const char *TAG = "CONTROL";

// --- Configuration Constants ---
#define CONTROL_LOOP_FREQ_HZ 10 
#define CONTROL_LOOP_DELAY   (1000 / CONTROL_LOOP_FREQ_HZ)
#define DT                   (1.0f / CONTROL_LOOP_FREQ_HZ)

// --- Heading Mode Selection ---
// 0 = ABSOLUTE (0 degrees is Magnetic North)
// 1 = RELATIVE (0 degrees is the heading the robot had when code started)
#define HEADING_TYPE 1 

// --- Helper: PID Class ---
class PIDController {
public:
    float error_sum = 0.0f;
    float last_error = 0.0f;

    // Standard PID for linear values (Height, Roll)
    float compute(float target, float current, float kp, float ki, float kd) {
        float error = target - current;
        
        float p_term = kp * error;
        
        error_sum += error * DT;
        error_sum = std::max(-1.0f, std::min(1.0f, error_sum)); // Anti-windup
        float i_term = ki * error_sum;

        float d_term = kd * (error - last_error) / DT;
        last_error = error;

        float output = p_term + i_term + d_term;
        return std::max(-1.0f, std::min(1.0f, output));
    }

    // Circular PID for Heading (Handles 0-360 wrap around)
    float compute_heading(float target, float current, float kp, float ki, float kd) {
        float error = target - current;

        // Normalize error to -180 to +180 (Shortest path turn)
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        float p_term = kp * error;
        
        error_sum += error * DT;
        error_sum = std::max(-1.0f, std::min(1.0f, error_sum));
        float i_term = ki * error_sum;

        float d_term = kd * (error - last_error) / DT;
        last_error = error;

        float output = p_term + i_term + d_term; 
        return std::max(-1.0f, std::min(1.0f, output)); 
    }
};

// --- Helper: Clamp Thruster Values ---
float clamp(float value) {
    return std::max(-1.0f, std::min(1.0f, value));
}

// --- Helper: Normalize Angle 0-360 ---
float normalize_angle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

void control_loop_task(void *arg) {

    std::vector<int> thruster_pins = {26, 25, 33, 32, 23, 4};
    ThrusterController vehicle;

    vehicle.init(thruster_pins);
    vehicle.stopAll();
    vTaskDelay(pdMS_TO_TICKS(3000)); 

    imu_shared_data_t imu_data;
    barometer_shared_data_t baro_data;
    robot_shared_state_t robot_state;

    // --- Tuning Parameters ---
    robot_state.config.kp = 0.01f; 
    robot_state.config.ki = 0.0f; 
    robot_state.config.kd = 0.0f;
    robot_state.target.v = 0.0f; robot_state.target.w = 0.0f; robot_state.target.h = 0.0f; 

    // Initialize PIDs
    PIDController pid_roll;
    PIDController pid_heading;
    PIDController pid_height;

    // --- Initialization Logic ---
    float current_target_heading = 0.0f;
    float startup_heading_offset = 0.0f;
    bool heading_initialized = false;
    
    // DELAY LOGIC: Wait for 30 loops (approx 3 seconds at 10Hz) before locking heading
    const int SETTLE_COUNT_LIMIT = 30; 
    int settle_counter = 0;

    ESP_LOGI(TAG, "Control Loop Started. Waiting for sensor settle...");

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        telemetry_get_state(&robot_state);
        bool imu_ok = imu_get_data(&imu_data);
        bool baro_ok = barometer_get_data(&baro_data);

        // Initialize thrusters to 0 safety default
        float t[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        if (imu_ok && baro_ok) {
            
            // --- PHASE 1: SENSOR SETTLING ---
            if (!heading_initialized) {
                settle_counter++;
                
                // Blink LED or Log every second
                if (settle_counter % 10 == 0) {
                    ESP_LOGI(TAG, "Calibrating IMU... [%d/%d] Curr: %.1f", 
                             settle_counter, SETTLE_COUNT_LIMIT, imu_data.heading);
                }

                // If we have waited enough loops, LOCK the heading now
                if (settle_counter >= SETTLE_COUNT_LIMIT) {
                    startup_heading_offset = imu_data.heading;
                    
                    #if HEADING_TYPE == 1
                        current_target_heading = 0.0f; // Relative Target is 0
                    #else
                        current_target_heading = imu_data.heading; // Absolute Target is current mag heading
                    #endif

                    heading_initialized = true;
                    ESP_LOGW(TAG, ">>> SYSTEM ARMED <<< | Init Heading Locked: %.2f", startup_heading_offset);
                }
            } 
            // --- PHASE 2: ACTIVE CONTROL ---
            else {
                // 1. Process Heading
                float processed_heading = imu_data.heading;
                
                #if HEADING_TYPE == 1
                    // RELATIVE: Shift raw heading by the startup offset
                    processed_heading = imu_data.heading - startup_heading_offset;
                    processed_heading = normalize_angle(processed_heading);
                #endif

                // 2. Input Integration
                current_target_heading += robot_state.target.w * DT;
                current_target_heading = normalize_angle(current_target_heading);
                
                // 3. Sensor Conversion
                float current_height = (float)baro_data.depth; 

                // 4. PID Compute
                float yaw_output = pid_heading.compute_heading(
                    current_target_heading, processed_heading, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                float heave_output = pid_height.compute(
                    robot_state.target.h, current_height, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                float roll_output = pid_roll.compute(
                    0.0f, imu_data.roll, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                // 5. Thruster Mixing
                float surge = robot_state.target.v;

                // Horizontal (Surge + Yaw)
                t[0] = clamp(surge - yaw_output); // T1
                t[1] = clamp(surge + yaw_output); // T2
                t[2] = clamp(surge + yaw_output); // T3
                t[3] = clamp(surge - yaw_output); // T4

                // Vertical (Heave + Roll)
                t[4] = clamp(heave_output - roll_output); // T5 (Left)
                t[5] = clamp(heave_output + roll_output); // T6 (Right)

                // Log Status
                static int log_counter = 0;
                if (log_counter++ > 10) {
                    //  LOG Target V, W, H, Target Heading \n  Hedaing Error, Roll Error, Depth Error,\n FR, FL, RL, RR, BL, BR Thruster Outputs  
                    ESP_LOGI(TAG, 
                        "Tgt V: %.2f W: %.2f H: %.2f | Tgt Head: %.2f | "
                        "Head Err: %.2f Roll Err: %.2f Depth Err: %.2f | "
                        "T1: %.2f T2: %.2f T3: %.2f T4: %.2f T5: %.2f T6: %.2f",
                        robot_state.target.v, robot_state.target.w, robot_state.target.h,
                        current_target_heading,
                        current_target_heading - processed_heading,
                        0.0f - imu_data.roll,
                        robot_state.target.h - current_height,
                        t[0], t[1], t[2], t[3], t[4], t[5]
                    );
                    log_counter = 0;
                }
                // 6. Apply Thruster Outputs
                t[0] = 0.0;
                t[1] = 0.0;
                t[2] = 0.0;
                t[3] = 0.0;
                vehicle.setSpeeds(std::vector<float>(t, t + 1));
            }
        } 
        else {
            vehicle.stopAll();
        }

        // Apply Motor Output (Keep this outside the if/else to ensure 0 is written during calibration)
        // motors_set_thrust(t); 

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_DELAY));
    }
}
// Thruster 1: Front-Right
// Thruster 2: Front-Left
// Thruster 3: Rear-Right
// Thruster 4: Rear-Left
// Thruster 5: Middle-Right
// Thruster 6: Middle-Left