#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// --- Pin Definitions ---
#define HX1_DT_PIN   GPIO_NUM_19
#define HX1_SCK_PIN  GPIO_NUM_18

#define HX2_DT_PIN   GPIO_NUM_27
#define HX2_SCK_PIN  GPIO_NUM_14

// --- Data Structure ---
typedef struct {
    int32_t hx1_raw;       
    int32_t hx2_raw; 
    float depth;      
} barometer_shared_data_t;

// --- Function Prototypes ---
bool barometer_init(void);
bool barometer_get_data(barometer_shared_data_t *out_data);

// EXPOSE THE TASK HERE:
void barometer_task(void *arg);