#include "telemetry.h"
#include "imu.h"             // Needed to get IMU data to send
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "SUB_TEL_MOD";

// --- Constants ---
#define TELEMETRY_UART_NUM      UART_NUM_0
#define TELEMETRY_TX_PIN        UART_PIN_NO_CHANGE
#define TELEMETRY_RX_PIN        UART_PIN_NO_CHANGE
#define BUF_SIZE                1024
#define TAG_TEL                 "SUB_TEL"
#define TAG_CMD                 "SUB_CMD"

// --- Private Structs ---
typedef struct __attribute__((packed)) {
    char tag[8]; 
} packet_header_t;

// --- Private Globals ---
// Static ensures these are only visible in this file
static command_shared_data_t g_command_data = {}; 
static SemaphoreHandle_t xCommandMutex = NULL;

// --- Implementation ---

void telemetry_init(void) {
    xCommandMutex = xSemaphoreCreateMutex();
    if (xCommandMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create Command Mutex");
    }

    uart_config_t uart_config = {}; 
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;
    
    ESP_ERROR_CHECK(uart_driver_install(TELEMETRY_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TELEMETRY_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TELEMETRY_UART_NUM, TELEMETRY_TX_PIN, TELEMETRY_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// THE NEW GETTER FUNCTION
bool telemetry_get_command(command_shared_data_t *out_data) {
    if (xCommandMutex == NULL) return false;

    // Try to take the mutex (wait up to 10ms)
    if (xSemaphoreTake(xCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Copy the data safely
        *out_data = g_command_data;
        xSemaphoreGive(xCommandMutex);
        return true;
    }
    return false;
}

void telemetry_task(void *pvParameters) {
    imu_shared_data_t tx_data;
    command_shared_data_t rx_data; 
    
    packet_header_t tx_header;
    packet_header_t rx_header;

    // Prepare TX Header
    memset(&tx_header, 0, sizeof(tx_header));
    strncpy(tx_header.tag, TAG_TEL, sizeof(tx_header.tag));
    
    while (1) { 
        // --- PART A: SEND TELEMETRY (TX) ---
        if (imu_get_data(&tx_data)) {
            uart_write_bytes(TELEMETRY_UART_NUM, (const char*)&tx_header, sizeof(packet_header_t));
            uart_write_bytes(TELEMETRY_UART_NUM, (const char*)&tx_data, sizeof(imu_shared_data_t));
        }

        // --- PART B: READ COMMANDS (RX) ---
        size_t available_bytes = 0;
        uart_get_buffered_data_len(TELEMETRY_UART_NUM, &available_bytes);

        if (available_bytes >= sizeof(packet_header_t)) {
            
            // 1. Peek Header
            uart_read_bytes(TELEMETRY_UART_NUM, &rx_header, sizeof(packet_header_t), pdMS_TO_TICKS(10));

            // 2. Verify Tag
            if (strncmp(rx_header.tag, TAG_CMD, 8) == 0) {
                
                int len = uart_read_bytes(TELEMETRY_UART_NUM, &rx_data, sizeof(command_shared_data_t), pdMS_TO_TICKS(20));

                if (len == sizeof(command_shared_data_t)) {
                    if (xSemaphoreTake(xCommandMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        // Update global protected struct
                        g_command_data = rx_data;
                        xSemaphoreGive(xCommandMutex);
                    }
                }
            } else {
                uart_flush_input(TELEMETRY_UART_NUM);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}