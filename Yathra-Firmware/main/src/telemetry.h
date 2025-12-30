#pragma once

#include <stdint.h>
#include <stdbool.h>

// --- Struct Definitions ---

// 1. Command Struct (PC -> ESP32)
// Defined here so other tasks know what data they are getting
typedef struct __attribute__((packed)) {
    uint8_t system_enable;      
    uint8_t arm;                
    float target_heading;
    float target_pitch;
    float target_roll;
    float target_altitude;      
} command_shared_data_t;

// --- Public Function Prototypes ---

// Initializes UART and the Mutex
void telemetry_init(void);

// The FreeRTOS task function
void telemetry_task(void *pvParameters);

// Thread-safe getter for command data
bool telemetry_get_command(command_shared_data_t *out_data);