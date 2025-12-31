#include "telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>


#define TELEMETRY_UART      UART_NUM_0
#define BUF_SIZE            1024
#define TX_QUEUE_LEN        20
#define SYNC_BYTE           0xA5 

#define IMU_TELEM_RATE_MS    100   // 100ms = 10Hz
#define AHRS_TELEM_RATE_MS   200   // 200ms = 5Hz
#define BARO_TELEM_RATE_MS   500   // 500ms = 2Hz

// --- Internal Queue Item ---
typedef struct {
    packet_id_t id;
    union {
        imu_msg_t imu;
        ahrs_msg_t ahrs;
        baro_msg_t baro;
    } payload;
} tx_queue_item_t;

// --- Globals ---
static QueueHandle_t xTxQueue = NULL;
static QueueHandle_t xUartEventQueue = NULL;
static SemaphoreHandle_t xStateMutex = NULL;

// Default State (Safety First: Disarmed, Zero PID)
static robot_shared_state_t g_robot_state = {
    .config = { .arm = false, .kp=0, .ki=0, .kd=0, .motor_rev={0} },
    .target = { .v = 0, .w = 0, .h = 0 }
};

// ==========================================================
// TX TASK (Sends Data)
// ==========================================================
void telemetry_tx_task(void *pvParameters) {
    tx_queue_item_t item;
    uint8_t header[2];

    while (1) {
        if (xQueueReceive(xTxQueue, &item, portMAX_DELAY) == pdTRUE) {
            
            // 1. Send Header (Sync + ID)
            header[0] = SYNC_BYTE;
            header[1] = (uint8_t)item.id;
            uart_write_bytes(TELEMETRY_UART, (const char*)header, 2);

            // 2. Send Payload
            switch (item.id) {
                case TEL_ID_IMU:
                    uart_write_bytes(TELEMETRY_UART, (const char*)&item.payload.imu, sizeof(imu_msg_t));
                    break;
                case TEL_ID_AHRS:
                    uart_write_bytes(TELEMETRY_UART, (const char*)&item.payload.ahrs, sizeof(ahrs_msg_t));
                    break;
                case TEL_ID_BARO:
                    uart_write_bytes(TELEMETRY_UART, (const char*)&item.payload.baro, sizeof(baro_msg_t));
                    break;
                default: break;
            }
        }
    }
}

// ==========================================================
// RX TASK (Reads Commands)
// ==========================================================
void telemetry_rx_task(void *pvParameters) {
    uart_event_t event;
    uint8_t header[2]; // [SYNC, ID]
    
    // Temp buffers for incoming data
    config_cmd_t temp_config;
    target_cmd_t temp_target;

    while (1) {
        if (xQueueReceive(xUartEventQueue, (void *)&event, portMAX_DELAY)) {
            
            if (event.type == UART_DATA) {
                // We peek to see if we have enough data for at least a header
                size_t buffered_len;
                uart_get_buffered_data_len(TELEMETRY_UART, &buffered_len);

                while (buffered_len >= 2) {
                    // 1. Peek Header to verify SYNC
                    // Note: In a real robust system, you'd read byte-by-byte to find SYNC. 
                    // Here we assume aligned packets for simplicity.
                    uart_read_bytes(TELEMETRY_UART, header, 2, pdMS_TO_TICKS(10));

                    if (header[0] != SYNC_BYTE) {
                        // Bad sync, flush or skip 1 byte (simple flush here)
                        uart_flush_input(TELEMETRY_UART); 
                        break; 
                    }

                    // 2. Process based on ID
                    if (header[1] == CMD_ID_CONFIG) {
                        int len = uart_read_bytes(TELEMETRY_UART, &temp_config, sizeof(config_cmd_t), pdMS_TO_TICKS(20));
                        if (len == sizeof(config_cmd_t)) {
                            xSemaphoreTake(xStateMutex, portMAX_DELAY);
                            g_robot_state.config = temp_config; // Update Config Only
                            xSemaphoreGive(xStateMutex);
                        }
                    } 
                    else if (header[1] == CMD_ID_TARGET) {
                        int len = uart_read_bytes(TELEMETRY_UART, &temp_target, sizeof(target_cmd_t), pdMS_TO_TICKS(20));
                        if (len == sizeof(target_cmd_t)) {
                            xSemaphoreTake(xStateMutex, portMAX_DELAY);
                            g_robot_state.target = temp_target; // Update Target Only
                            xSemaphoreGive(xStateMutex);
                        }
                    }
                    
                    // Update len for while loop
                    uart_get_buffered_data_len(TELEMETRY_UART, &buffered_len);
                }
            } 
            else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                uart_flush_input(TELEMETRY_UART);
                xQueueReset(xUartEventQueue);
            }
        }
    }
}

// ==========================================================
// PUBLIC FUNCTIONS
// ==========================================================
void telemetry_init(void) {
    xStateMutex = xSemaphoreCreateMutex();
    xTxQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(tx_queue_item_t));

    uart_config_t uart_config = {};
        uart_config.baud_rate = 115200;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_DEFAULT;
    

    ESP_ERROR_CHECK(uart_driver_install(TELEMETRY_UART, BUF_SIZE * 2, BUF_SIZE * 2, 20, &xUartEventQueue, 0));
    ESP_ERROR_CHECK(uart_param_config(TELEMETRY_UART, &uart_config));

    // xTaskCreate(telemetry_tx_task, "tel_tx", 4096, NULL, 5, NULL);
    // xTaskCreate(telemetry_rx_task, "tel_rx", 4096, NULL, 10, NULL);
}

bool telemetry_get_state(robot_shared_state_t *out_state) {
    if (xStateMutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *out_state = g_robot_state;
        xSemaphoreGive(xStateMutex);
        return true;
    }
    return false;
}

// --- SEND WRAPPERS ---

// --- TELEMETRY FREQUENCY CONFIG ---
#define IMU_TELEM_RATE_MS    100   // 100ms = 10Hz
#define AHRS_TELEM_RATE_MS   200   // 200ms = 5Hz
#define BARO_TELEM_RATE_MS   500   // 500ms = 2Hz

// --- SEND WRAPPERS ---

bool telemetry_send_imu(const imu_msg_t *data) {
    if (xTxQueue == NULL) return false;

    // 1. Static variable holds state between function calls
    static TickType_t last_sent_time = 0;
    
    // 2. Check current time
    TickType_t now = xTaskGetTickCount();

    // 3. Only send if enough time has passed (Limit to 10Hz)
    // The subtraction handles timer overflow automatically
    if ((now - last_sent_time) >= pdMS_TO_TICKS(IMU_TELEM_RATE_MS)) {
        
        last_sent_time = now; // Update timestamp

        tx_queue_item_t item = {}; 
        item.id = TEL_ID_IMU;
        item.payload.imu = *data;
        
        // Use 0 wait time to avoid blocking if queue is full
        return xQueueSend(xTxQueue, &item, 0) == pdTRUE;
    }

    // Return true to pretend we succeeded (so the calling task doesn't panic)
    return true; 
}

bool telemetry_send_ahrs(const ahrs_msg_t *data) {
    if (xTxQueue == NULL) return false;

    static TickType_t last_sent_time = 0;
    TickType_t now = xTaskGetTickCount();

    if ((now - last_sent_time) >= pdMS_TO_TICKS(AHRS_TELEM_RATE_MS)) { // 5Hz
        last_sent_time = now;

        tx_queue_item_t item = {};
        item.id = TEL_ID_AHRS;
        item.payload.ahrs = *data;
        return xQueueSend(xTxQueue, &item, 0) == pdTRUE;
    }
    return true;
}

bool telemetry_send_baro(const baro_msg_t *data) {
    if (xTxQueue == NULL) return false;

    static TickType_t last_sent_time = 0;
    TickType_t now = xTaskGetTickCount();

    if ((now - last_sent_time) >= pdMS_TO_TICKS(BARO_TELEM_RATE_MS)) { // 2Hz
        last_sent_time = now;

        tx_queue_item_t item = {};
        item.id = TEL_ID_BARO;
        item.payload.baro = *data;
        return xQueueSend(xTxQueue, &item, 0) == pdTRUE;
    }
    return true;
}