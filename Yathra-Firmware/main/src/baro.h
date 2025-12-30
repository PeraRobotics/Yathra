#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// --- Pin Definitions ---
#define HX1_DT_PIN   GPIO_NUM_34
#define HX1_SCK_PIN  GPIO_NUM_4

#define HX2_DT_PIN   GPIO_NUM_35
#define HX2_SCK_PIN  GPIO_NUM_5

// --- Data Structure ---
typedef struct {
    int32_t hx1_raw;       
    int32_t hx2_raw;       
} barometer_shared_data_t;

// --- Function Prototypes ---
bool barometer_init(void);
bool barometer_get_data(barometer_shared_data_t *out_data);

// EXPOSE THE TASK HERE:
void barometer_task(void *arg);