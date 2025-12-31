#include "imu.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // Required for Mutex
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "src/telemetry.h"

extern "C" {
    #include "ahrs.h"
    #include "mpu9250.h"
    #include "calibrate.h"
    #include "common.h"
    #include "sens_qmc5883l.h"
}

static const char* TAG = "SUB_LOG";

#define I2C_MASTER_NUM I2C_NUM_0 

// --- Shared Data & Mutex ---
static imu_shared_data_t g_imu_data;      // The actual data storage
static SemaphoreHandle_t xImuMutex = NULL; // The lock

// --- Calibration (Same as before) ---
// static calibration_t cal = {
//     .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
//     .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
//     .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782},
//     .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
//     .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
//     .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645}
// };
static calibration_t cal = {
    .mag_offset = {.x = 0, .y = 0, .z = 0},
    .mag_scale = {.x = 0, .y = 0, .z = 0},
    .gyro_bias_offset = {.x = 0, .y = 0, .z = 0},
    .accel_offset = {.x = 0, .y = 0, .z = 0},
    .accel_scale_lo = {.x = 0, .y = 0, .z = 0},
    .accel_scale_hi = {.x = 0, .y = 0, .z = 0},
};

// --- Transformation Functions (Same as before) ---
static void transform_accel_gyro(vector_t *v) {
  float x = v->x; float y = v->y; float z = v->z;
  v->x = -x; v->y = -z; v->z = -y;
}

static void transform_mag(vector_t *v) {
  float x = v->x; float y = v->y; float z = v->z;
  v->x = -y; v->y = z; v->z = -x;
}

void pack_imu_message(const imu_shared_data_t *src, imu_msg_t *dest) {
    // Map Accelerometer
    dest->accel[0] = src->accel.x;
    dest->accel[1] = src->accel.y;
    dest->accel[2] = src->accel.z;

    // Map Gyroscope
    dest->gyro[0] = src->gyro.x;
    dest->gyro[1] = src->gyro.y;
    dest->gyro[2] = src->gyro.z;

    // Map Magnetometer
    dest->mag[0] = src->mag.x;
    dest->mag[1] = src->mag.y;
    dest->mag[2] = src->mag.z;
}

// --- Public Getter Function ---
bool imu_get_data(imu_shared_data_t *out_data)
{
    if (xImuMutex == NULL) return false;

    // Try to take the mutex (wait up to 10ms)
    if (xSemaphoreTake(xImuMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Copy the data safely
        *out_data = g_imu_data;
        xSemaphoreGive(xImuMutex);
        return true;
    }
    return false;
}

// --- Internal Logic ---
static void run_imu(bool use_mag)
{
  i2c_mpu9250_init(&cal, use_mag); // Ensure your libs support this signature!
  if(!use_mag){
    qmc5883l_init(I2C_NUM_0);
  }
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);

  while (true)
  {
    vector_t va, vg, vm;

    // 1. Read Sensors
    if(use_mag){
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);
    } else {
        ESP_ERROR_CHECK(get_accel_gyro(&va, &vg));
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        qmc5883l_read_mag_float((qmc_vector_t*)&vm); 
        // vm = {0,0,0};
    }

    // 2. Run Filter (Madgwick/Mahony)
    // if (use_mag) {
    //     ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
    //                 va.x, va.y, va.z, vm.x, vm.y, vm.z);
    // } else {
    //     ahrs_update_imu(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
    //                     va.x, va.y, va.z);
    // }
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                    va.x, va.y, va.z, vm.x, vm.y, vm.z);
    // 3. Update Shared Data (Protected by Mutex)

    if (xImuMutex != NULL) {
        if (xSemaphoreTake(xImuMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            
            // Update Raw Data
            g_imu_data.accel = va;
            g_imu_data.gyro = vg;
            g_imu_data.mag = vm;

            // Update Filtered Data (Euler Angles)
            ahrs_get_euler_in_degrees(&g_imu_data.heading, 
                                      &g_imu_data.pitch, 
                                      &g_imu_data.roll);
            
            // Optional: Temperature
            get_temperature_celsius(&g_imu_data.temp_c);

            xSemaphoreGive(xImuMutex);
        }
    }
    // imu_msg_t imu_data;
    // pack_imu_message(&g_imu_data, &imu_data);
    // telemetry_send_imu(&imu_data);


    // ahrs_msg_t ahrs_msg;
    //     ahrs_msg.roll = g_imu_data.roll;
    //     ahrs_msg.pitch = g_imu_data.pitch;
    //     ahrs_msg.heading = g_imu_data.heading;
    // ESP_LOGI(TAG, "AHRS Data - Heading: %.2f Roll: %.2f Pitch: %.2f", 
    //          ahrs_msg.heading, ahrs_msg.roll, ahrs_msg.pitch);
    // telemetry_send_ahrs(&ahrs_msg);
    
    // Pause to maintain Sample Rate
    pause(); 
  }
}

// --- Init & Task ---

void imu_init(void)
{
    // Create the Mutex
    xImuMutex = xSemaphoreCreateMutex();
    if (xImuMutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU Mutex");
    }
}

void imu_task(void *arg)
{
    imu_task_config_t *config = (imu_task_config_t *)arg;
    bool use_mag = false;
    if (config != NULL) use_mag = config->enable_mag;

    // Optional: Call init here if not called in main
    if (xImuMutex == NULL) imu_init();

#ifdef CONFIG_CALIBRATION_MODE
    calibrate_gyro();
    calibrate_accel();
    if (use_mag) calibrate_mag();
#else
    run_imu(use_mag);
#endif

    vTaskDelete(NULL);
}