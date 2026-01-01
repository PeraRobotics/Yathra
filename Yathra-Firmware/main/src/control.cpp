#include "control.hpp"
#include <math.h>
#include <algorithm>
#include <vector>
#include <iterator>

// FreeRTOS Includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Custom Includes (Assumed these exist per your snippet)
#include "imu.h"       
#include "baro.h" 
#include "telemetry.h" 
#include "thruster.hpp"
#include "mission.hpp"


static const char *TAG = "CONTROL";

// --- Configuration Constants ---
#define CONTROL_LOOP_FREQ_HZ 10 
#define CONTROL_LOOP_DELAY   (1000 / CONTROL_LOOP_FREQ_HZ)
#define DT                   (1.0f / CONTROL_LOOP_FREQ_HZ)


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



void control_loop_task(void *arg) {

    // Init Hardware
    std::vector<int> thruster_pins = {26, 25, 33, 32, 23, 4};
    ThrusterController vehicle;
    vehicle.init(thruster_pins);
    vehicle.stopAll();
    vTaskDelay(pdMS_TO_TICKS(3000)); 

    imu_shared_data_t imu_data;
    barometer_shared_data_t baro_data;
    robot_shared_state_t robot_state;

    MissionManager mission_supervisor;
    std::vector<MissionStep> test_mission = {
        // Name        Type         Dura.   Depth   Heading     Surge  HeadingFlag      
        {"Dive",      MISSION_HOLD, 5.0f,   0.5f,   0.0f,       0.0f,   true}, // Depth 0.5m, Heading 0
        {"Forward",   MISSION_MOVE, 3.0f,   0.5f,   0.0f,       0.3f,   true}, // Surge 30%
        {"Turn 90",   MISSION_HOLD, 4.0f,   0.5f,   90.0f,      0.0f,   true}, // Heading 90
        {"Surface",   MISSION_HOLD, 5.0f,   0.0f,   90.0f,      0.0f,   true}  // Depth 0
    };
    mission_supervisor.loadMission(test_mission);



    // Initialize PIDs
    PIDController pid_roll, pid_heading, pid_height;
    robot_state.config.kp = 0.01f; 
    robot_state.config.ki = 0.0f; 
    robot_state.config.kd = 0.0f;

    robot_state.config.mode = 0; // 0: STABILIZE, 1: MISSION , 2: GUIDED
    robot_state.config.heading_type = 1; // 0: ABSOLUTE, 1: RELATIVE 

    robot_state.target.v = 0.0f; 
    robot_state.target.w = 0.0f; 
    robot_state.target.h = 0.0f; 
    robot_state.target.is_w_head = true;

    // --- Initialization Logic ---
    float target_heading = 0.0f;
    float heading_offset = 0.0f;
    bool heading_initialized = false;

    const int SETTLE_COUNT_LIMIT = 30;
    int settle_counter = 0;

    ESP_LOGI(TAG, "Control Loop Started.");
    TickType_t xLastWakeTime = xTaskGetTickCount();


    while(1) {

        if (robot_state.config.mode == 1) {
                bool running = mission_supervisor.update(DT, &robot_state);
                if (!running) {
                    robot_state.target.v = 0.0f;
                    robot_state.target.w = 0.0f;
                    robot_state.target.h = 0.0f;
                }
        } else if  (robot_state.config.mode == 2) {
            telemetry_get_state(&robot_state);
        }
        
        bool imu_ok = imu_get_data(&imu_data);
        bool baro_ok = barometer_get_data(&baro_data);
        float t[6] = {0};

        if (imu_ok && baro_ok) {
            
            // --- PHASE 1: SENSOR SETTLING ---
            if (!heading_initialized) {
                if (settle_counter % 10 == 0) {
                    ESP_LOGI(TAG, "Calibrating IMU... [%d/%d] Curr: %.1f", 
                             settle_counter, SETTLE_COUNT_LIMIT, imu_data.heading);
                }
                if (++settle_counter >= SETTLE_COUNT_LIMIT) {
                    target_heading = imu_data.heading;
                    heading_initialized = true;
                    ESP_LOGW(TAG, "Heading Locked: %.2f", target_heading);
                }
            }
            // Phase 2: Active Control
            else{
                if(robot_state.target.is_w_head){
                    //  If w is a heading angle
                    if(robot_state.config.heading_type == 1){
                        // Heading Control is Relative to Current heading
                        if(heading_offset != robot_state.target.w){
                            heading_offset = robot_state.target.w;
                            target_heading += robot_state.target.w;
                        }
                    }else{
                        target_heading = robot_state.target.w;
                    }
                }else{
                    // If w is a angular velocity
                    target_heading += robot_state.target.w * DT;
                    target_heading = normalize_angle(target_heading);
                }  

                
                // PID Compute
                float yaw_output = pid_heading.compute_heading(
                    target_heading, imu_data.heading, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                float heave_output = pid_height.compute(
                    robot_state.target.h, (float)baro_data.depth, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                float roll_output = pid_roll.compute(
                    0.0f, imu_data.roll, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                float surge = robot_state.target.v;

                // Horizontal (Surge + Yaw)
                t[0] = clamp(surge - yaw_output); // T1: Front-Right
                t[1] = clamp(surge + yaw_output); // T2: Front-Left
                t[2] = clamp(surge - yaw_output); // T3: Rear-Right
                t[3] = clamp(surge + yaw_output); // T4: Rear-Left
                // Vertical (Heave + Roll)
                t[4] = clamp(heave_output + roll_output); // T5: Middle-UP-Right
                t[5] = clamp(heave_output - roll_output); // T6: Middle-UP-Left

                static int log_counter = 0;
                if (log_counter++ > 5) {
                    printf("\033[2J\033[H");
                    printf(
                            "========== ROBOT STATUS ==========\n"
                            "Target V: %5.2f  W: %5.2f  H: %5.2f  |  Head: %6.2f\n"
                            "Head Err: %6.2f  Roll Err: %6.2f  Depth Err: %6.2f\n"
                            "Current IMU: H: %6.2f  R: %6.2f  P: %6.2f\n"
                            "----------------------------------\n"
                            "   FL: %6.2f      FR: %6.2f\n"
                            "   ML: %6.2f      MR: %6.2f\n"
                            "   RL: %6.2f      RR: %6.2f\n"
                            "==================================\n",
                            robot_state.target.v, robot_state.target.w, robot_state.target.h,
                            target_heading,
                            target_heading - imu_data.heading,
                            0.0f - imu_data.roll,
                            robot_state.target.h - (float)baro_data.depth,
                            imu_data.heading, imu_data.roll, imu_data.pitch,
                            t[1], t[0], // FL, FR
                            t[5], t[4], // ML, MR 
                            t[3], t[2]  // RL, RR
                        );
                    log_counter = 0;
                }
                
                t[0] = 0.0;
                t[1] = 0.0;
                t[2] = 0.0;
                t[3] = 0.0;
                vehicle.setSpeeds(std::vector<float>(t, t + 1));
            }
        
        }else {
            vehicle.stopAll();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_DELAY));
    }
}
