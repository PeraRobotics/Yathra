#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

// --- Packet IDs (Header Bytes) ---
// We use these to know what data is flying over the wire
typedef enum {
    // Outgoing (ESP32 -> PC)
    TEL_ID_IMU   = 0x10,
    TEL_ID_AHRS  = 0x11,
    TEL_ID_BARO  = 0x12,
    
    // Incoming (PC -> ESP32)
    CMD_ID_CONFIG = 0x20,
    CMD_ID_TARGET = 0x21
} packet_id_t;

// --- Outgoing Data Structures ---
typedef struct __attribute__((packed)){
    float accel[3];
    float gyro[3];
    float mag[3];
} imu_msg_t;

typedef struct __attribute__((packed)){
    float heading;
    float pitch;
    float roll;
} ahrs_msg_t;

typedef struct __attribute__((packed)){
    float pressure_out;
    float pressure_in;
    float temp_in;
} baro_msg_t;

// --- Incoming Command Structures ---
typedef struct {
    bool arm;
    float kp;
    float ki;
    float kd;
    bool motor_rev[6]; // t1_reverse to t6_reverse
} config_cmd_t;

typedef struct {
    float v; // Velocity
    float w; // Angular Velocity
    float h; // Height
} target_cmd_t;

// --- Global State Container ---
// Your control loop will read from this
typedef struct {
    config_cmd_t config;
    target_cmd_t target;
} robot_shared_state_t;

// --- Function Prototypes ---
void telemetry_init(void);
void telemetry_tx_task(void *pvParameters);
void telemetry_rx_task(void *pvParameters);
// For Control Loop: Get latest state safely
bool telemetry_get_state(robot_shared_state_t *out_state);

// For Tasks: Send data (Non-blocking)
bool telemetry_send_imu(const imu_msg_t *data);
bool telemetry_send_ahrs(const ahrs_msg_t *data);
bool telemetry_send_baro(const baro_msg_t *data);

#endif