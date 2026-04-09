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
#define CONTROL_LOOP_FREQ_HZ 20 
#define CONTROL_LOG_FREQ_HZ 5
#define LOG_THRESHOLD  ((CONTROL_LOOP_FREQ_HZ / CONTROL_LOG_FREQ_HZ) - 2)
#define CONTROL_LOOP_DELAY   (1000 / CONTROL_LOOP_FREQ_HZ)
#define DT                   (1.0f / CONTROL_LOOP_FREQ_HZ)
#define BUTTON_PIN GPIO_NUM_16

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
    std::vector<int> thruster_pins = { 32, 33, 25, 26, 23, 4};
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
        {"Dive",      MISSION_HOLD, 5.0f,   -0.1f,   0.0f,       0.0f,   false}, // Depth 0.5m, Heading 0        
        {"Forward",   MISSION_MOVE, 2.0f,   -0.1f,   0.0f,       0.0f,   false}, // Surge 30%
        {"Turn 90",   MISSION_HOLD, 5.0f,   -0.1f,   0.0f,      0.5f,   false}, // Heading 90
        {"Forward",   MISSION_MOVE, 2.0f,  -0.1f,   0.2f,       0.0f,   false}, // Surge 30%
        {"Turn 90",   MISSION_HOLD, 2.0f,   -0.1f,   0.0f,      0.0f,   false}, // Heading 90
        {"Forward",   MISSION_MOVE, 4.0f,  -0.1f,   0.0f,       -0.75f,   false}, // Surge 30%
        {"Turn 90",   MISSION_HOLD, 2.0f,   -0.1f,   0.2f,      0.0f,   false}, // Heading 90
        {"Turn 90",   MISSION_HOLD, 2.0f,   -0.1f,   0.0f,      0.0f,   false}, // Heading 90
        {"Turn 90",   MISSION_HOLD, 2.0f,   -0.1f,   -0.3f,      0.8f,   false}, // Heading 90
    };
    // mission_supervisor.loadMission(test_mission);

    // Initialize PIDs
    PIDController pid_roll, pid_heading, pid_height;
    robot_state.config.kp = 0.01f; 
    robot_state.config.ki = 0.0f; 
    robot_state.config.kd = 0.005f;

    robot_state.config.mode = 1; // 0: STABILIZE, 1: MISSION , 2: GUIDED
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
    const TickType_t LONG_PRESS_TICKS = pdMS_TO_TICKS(2000);
    TickType_t press_start_tick = 0;
    bool is_pressed = false;
    bool long_press_action_taken = false;
    int last_btn_state = 1;
    while(1) {

        int curr_btn_state = gpio_get_level(BUTTON_PIN);
        if (last_btn_state == 1 && curr_btn_state == 0) {
            press_start_tick = xTaskGetTickCount();
            is_pressed = true;
            long_press_action_taken = false;
        }


        if (curr_btn_state == 0) {
            TickType_t current_tick = xTaskGetTickCount();
            
            // Check if held longer than threshold AND action not yet taken
            if ((current_tick - press_start_tick) > LONG_PRESS_TICKS) {
                if (!long_press_action_taken) {
                    // --- LONG PRESS DETECTED: STOP ---
                    ESP_LOGW(TAG, "Long Press Detected: STOPPING MISSION!");
                    vehicle.stopAll();
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    mission_supervisor.loadMission(test_mission);
  
                    
                    // Mark as handled so we don't trigger it again or trigger "Start" on release
                    long_press_action_taken = true; 
                }
            }
        }

        // 3. DETECT RELEASE (Rising Edge 0 -> 1)
        if (last_btn_state == 0 && curr_btn_state == 1) {
            is_pressed = false;

            // Only Start if it wasn't a long press (Short Click)
            if (!long_press_action_taken) {
                // --- SHORT PRESS DETECTED: START/RESTART ---
                ESP_LOGI(TAG, "Short Press: Starting Mission...");
                    mission_supervisor.stop();
                    
                    // SAFETY: Kill motors immediately
                    robot_state.target.v = 0.0f;
                    robot_state.target.w = 0.0f;
                    robot_state.target.h = 0.0f;
                
            }
        }

        last_btn_state = curr_btn_state;

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
        const int num_trusters = 6;
        std::vector<float> trusters(num_trusters, 0.0f);


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
                // if(robot_state.target.is_w_head){
                //     //  If w is a heading angle
                //     if(robot_state.config.heading_type == 1){
                //         // Heading Control is Relative to Current heading
                //         if(heading_offset != robot_state.target.w){
                //             heading_offset = robot_state.target.w;
                //             target_heading += robot_state.target.w;
                //         }
                //     }else{
                //         target_heading = robot_state.target.w;
                //     }
                // }else{
                //     // If w is a angular velocity
                //     target_heading += robot_state.target.w * DT;
                //     target_heading = normalize_angle(target_heading);
                // }  

                
                // PID Compute
                // float yaw_output = pid_heading.compute_heading(
                //     target_heading, imu_data.heading, 
                //     robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                // );

                // float heave_output = pid_height.compute(
                //     robot_state.target.h, (float)baro_data.depth, 
                //     robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                // );

                float roll_output = pid_roll.compute(
                    0.0f, imu_data.roll, 
                    robot_state.config.kp, robot_state.config.ki, robot_state.config.kd
                );

                float surge_output = robot_state.target.v;
                float yaw_output = robot_state.target.w;
                float heave_output = robot_state.target.h;
                // Horizontal (Surge + Yaw)
                trusters[0] = clamp(surge_output - yaw_output); // T1: Front-Right
                trusters[1] = clamp(surge_output + yaw_output); // T2: Front-Left
                trusters[2] = clamp(surge_output - yaw_output); // T3: Rear-Right
                trusters[3] = clamp(surge_output + yaw_output); // T4: Rear-Left
                // Vertical (Heave + Roll)
                trusters[4] = clamp(heave_output + roll_output); // T5: Middle-UP-Right
                trusters[5] = clamp(heave_output - roll_output); // T6: Middle-UP-Left

                static int log_counter = 0;
                if (log_counter++ > LOG_THRESHOLD) {
                    printf("\033[2J\033[H");
                    printf(
                            "Tar V: %5.2f W: %5.2f H: %5.2f | H: %6.2f\n"
                            "Err H: %6.2f  R: %6.2f  D: %6.2f\n"
                            "Cur H: %6.2f  R: %6.2f  P: %6.2f D:%6.2f\n"
                            "FL: %6.2f FR: %6.2f\n"
                            "ML: %6.2f MR: %6.2f\n"
                            "RL: %6.2f RR: %6.2f\n"
                            "Step: %s\n\n",
                            robot_state.target.v, robot_state.target.w, robot_state.target.h,
                            target_heading,
                            target_heading - imu_data.heading,
                            0.0f - imu_data.roll,
                            robot_state.target.h - (float)baro_data.depth,
                            imu_data.heading, imu_data.roll, imu_data.pitch,baro_data.depth,
                            trusters[1], trusters[0], // FL, FR
                            trusters[5], trusters[4], // ML, MR 
                            trusters[3], trusters[2],  // RL, RR
                            mission_supervisor.getCurrentStepName().c_str()
                        );
                    log_counter = 0;
                }
                
                // trusters[0] = 0.0;
                // trusters[1] = 0.0;
                // trusters[2] = 0.0;
                // trusters[3] = 0.0;
                // trusters[4] = 0.0;
                // trusters[5] = 0.0;
                // vehicle.setSpeeds(std::vector<float>(t, t + 1));
                // std::vector<float> speeds = {0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f};
                vehicle.setSpeeds(trusters);
            }
        
        }else {

            vehicle.stopAll();
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_DELAY));
    }
}
