#include "imu_task.h"
#include "bmi088_driver.h"
#include "imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>
#include <float.h>
#include <string.h>

extern QueueHandle_t xIMUQueue;
/* Legacy symbol retained. Other tasks should use IMU_ReadSnapshot(). */
IMUData_TypeDef IMUdata;
static uint8_t calibration_requested;
static const uint8_t body_axes[3] = IMU_BODY_AXES;
static const int8_t body_signs[3] = IMU_BODY_SIGNS;

typedef struct {
    uint32_t count;
    float gyro_mean[3], gyro_m2[3];
    float accel_mean[3], accel_m2[3];
} Calibration_t;

static uint8_t Finite(float x)
{
    return x == x && fabsf(x) <= FLT_MAX;
}

static void Publish(const IMUData_TypeDef *sample)
{
    taskENTER_CRITICAL();
    IMUdata = *sample;
    taskEXIT_CRITICAL();
    if (xIMUQueue != NULL) xQueueOverwrite(xIMUQueue, sample);
}

uint8_t IMU_ReadSnapshot(IMUData_TypeDef *out)
{
    uint32_t now;
    if (out == NULL) return 0;
    taskENTER_CRITICAL();
    *out = IMUdata;
    now = (uint32_t)xTaskGetTickCount();
    taskEXIT_CRITICAL();
    if ((uint32_t)(now - out->sample_tick) >
        (uint32_t)pdMS_TO_TICKS(IMU_DATA_MAX_AGE_MS)) out->valid = 0;
    return out->valid && out->calibrated && out->status == IMU_STATUS_RUNNING;
}

void IMU_RequestCalibration(void)
{
    taskENTER_CRITICAL();
    calibration_requested = 1;
    IMUdata.valid = 0;
    IMUdata.calibrated = 0;
    IMUdata.status = IMU_STATUS_CALIBRATING;
    taskEXIT_CRITICAL();
}

static uint8_t MappingValid(void)
{
    unsigned int i, j, inversions = 0;
    int determinant = 1;
    for (i = 0; i < 3; i++) {
        if (body_axes[i] > 2 ||
            (body_signs[i] != 1 && body_signs[i] != -1)) return 0;
        determinant *= body_signs[i];
        for (j = 0; j < i; j++) {
            if (body_axes[i] == body_axes[j]) return 0;
            if (body_axes[j] > body_axes[i]) inversions++;
        }
    }
    if (inversions & 1U) determinant = -determinant;
    return determinant == 1;
}

/* Returns 1 for a stationary window; rejects motion and excessive noise.
 * These are commissioning thresholds, not a proof that the robot is still. */
static uint8_t Calibrate(Calibration_t *cal, const float accel[3],
                         const float gyro[3], float bias[3])
{
    unsigned int i;
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (fabsf(norm - 9.81f) > 0.981f) {
        memset(cal, 0, sizeof(*cal));
        return 0;
    }
    for (i = 0; i < 3; i++) {
        if (fabsf(gyro[i]) > 0.15f) {
            memset(cal, 0, sizeof(*cal));
            return 0;
        }
    }
    cal->count++;
    for (i = 0; i < 3; i++) {
        float d = gyro[i] - cal->gyro_mean[i];
        cal->gyro_mean[i] += d / (float)cal->count;
        cal->gyro_m2[i] += d * (gyro[i] - cal->gyro_mean[i]);
        d = accel[i] - cal->accel_mean[i];
        cal->accel_mean[i] += d / (float)cal->count;
        cal->accel_m2[i] += d * (accel[i] - cal->accel_mean[i]);
    }
    if (cal->count < IMU_CALIBRATION_SAMPLES) return 0;
    for (i = 0; i < 3; i++) {
        if (cal->gyro_m2[i] / (float)(cal->count - 1U) > 0.0004f ||
            cal->accel_m2[i] / (float)(cal->count - 1U) > 0.04f) {
            memset(cal, 0, sizeof(*cal));
            return 0;
        }
    }
    memcpy(bias, cal->gyro_mean, sizeof(cal->gyro_mean));
    return 1;
}

void vIMUTask(void *pvParameters)
{
    /* Large persistent state is in static RAM, not the task stack.
     * Only one instance of this task may be created. */
    static IMU_t imu;
    static Calibration_t calibration;
    static IMUData_TypeDef sample;
    float sensor[BMI088_DATA_LENGTH];
    uint32_t previous_cycles = 0, previous_tick = 0, warmup_tick = 0;
    uint32_t consecutive_errors = 0;
    uint8_t initialized = 0, have_previous = 0, request;
    TickType_t last_wake;
    TickType_t period = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
    unsigned int i;
    (void)pvParameters;
    configASSERT(period > 0);
    configASSERT(xIMUQueue != NULL);
    if (period == 0) period = 1;
    memset(&sample, 0, sizeof(sample));
    sample.quaternion[0] = 1.0f;
    if (!MappingValid()) {
        sample.status = IMU_STATUS_CONFIG_ERROR;
        Publish(&sample);
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Enable the shared free-running cycle counter, without resetting it.
     * Delta uses unsigned subtraction, which handles one counter wrap. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    sample.cycle_clock = (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
    last_wake = xTaskGetTickCount();
    for (;;) {
        if (!initialized) {
            sample.valid = sample.calibrated = 0;
            sample.status = IMU_STATUS_INITIALIZING;
            Publish(&sample);
            if (Drive_BMI088_Init() != BMI088_OK) {
                sample.read_errors++;
                sample.status = IMU_STATUS_SENSOR_ERROR;
                Publish(&sample);
                vTaskDelay(pdMS_TO_TICKS(1000));
                last_wake = xTaskGetTickCount();
                continue;
            }
            IMU_Init(&imu);
            memset(&calibration, 0, sizeof(calibration));
            sample.calibration_count = 0;
            sample.reset_count++;
            warmup_tick = (uint32_t)xTaskGetTickCount();
            have_previous = 0;
            initialized = 1;
            consecutive_errors = 0;
            last_wake = xTaskGetTickCount();
        }
        taskENTER_CRITICAL();
        request = calibration_requested;
        calibration_requested = 0;
        taskEXIT_CRITICAL();
        if (request) {
            IMU_Init(&imu);
            memset(&calibration, 0, sizeof(calibration));
            sample.calibrated = sample.valid = 0;
            sample.calibration_count = 0;
            sample.reset_count++;
            have_previous = 0;
        }
        sample.valid = 0;
        if (Drive_BMI088_Read(sensor) != BMI088_OK) {
            sample.read_errors++;
            consecutive_errors++;
            sample.status = IMU_STATUS_SENSOR_ERROR;
            if (!sample.calibrated) memset(&calibration, 0, sizeof(calibration));
            if (consecutive_errors >= 10U) initialized = 0;
            Publish(&sample);
        } else {
            uint8_t finite = 1;
            uint32_t now_tick = (uint32_t)xTaskGetTickCount();
            uint32_t now_cycles = DWT->CYCCNT;
            uint32_t tick_delta = now_tick - previous_tick;
            uint32_t cycle_delta = now_cycles - previous_cycles;
            float dt = (float)tick_delta / (float)configTICK_RATE_HZ;
            for (i = 0; i < BMI088_DATA_LENGTH; i++) {
                if (!Finite(sensor[i])) finite = 0;
            }
            if (have_previous && sample.cycle_clock && cycle_delta != 0U &&
                tick_delta <= (uint32_t)pdMS_TO_TICKS(50) && SystemCoreClock != 0U) {
                dt = (float)cycle_delta / (float)SystemCoreClock;
            }
            if (!have_previous) dt = (float)period / (float)configTICK_RATE_HZ;
            sample.dt = dt;
            if (!finite) {
                sample.read_errors++;
                consecutive_errors++;
                if (consecutive_errors >= 10U) initialized = 0;
                sample.status = IMU_STATUS_SENSOR_ERROR;
                memset(&calibration, 0, sizeof(calibration));
            } else {
                previous_tick = now_tick;
                previous_cycles = now_cycles;
                have_previous = 1;
                consecutive_errors = 0;
                sample.sample_tick = now_tick;
                sample.sample_count++;
                for (i = 0; i < 3; i++) {
                    sample.accel[i] = body_signs[i] * sensor[body_axes[i]];
                    sample.gyro[i] = body_signs[i] * sensor[3U + body_axes[i]];
                }
                sample.temperature = sensor[6];
                if (dt <= 0.0f || dt > 0.05f) {
                    /* A long data gap requires stationary recalibration.
                     * Status/reset_count expose the resulting yaw discontinuity. */
                    IMU_Init(&imu);
                    memset(&calibration, 0, sizeof(calibration));
                    sample.calibrated = 0;
                    sample.reset_count++;
                    sample.status = IMU_STATUS_TIMING_ERROR;
                } else if ((uint32_t)(now_tick - warmup_tick) <
                           (uint32_t)pdMS_TO_TICKS(500)) {
                    sample.status = IMU_STATUS_WARMUP;
                } else {
                    if (!sample.calibrated) {
                        sample.status = IMU_STATUS_CALIBRATING;
                        if (Calibrate(&calibration, sample.accel, sample.gyro,
                                      sample.gyro_bias)) {
                            IMU_SetGyroBias(&imu, sample.gyro_bias);
                            sample.calibrated = 1;
                        }
                    }
                    if (sample.calibrated) {
                        if (IMU_Update(&imu, sample.accel, sample.gyro, dt)) {
                            const IMUResult_t *result = IMU_GetResult(&imu);
                            memcpy(sample.quaternion, result->quaternion,
                                   sizeof(sample.quaternion));
                            memcpy(sample.angular_velocity, result->gyro,
                                   sizeof(sample.angular_velocity));
                            sample.roll = result->roll;
                            sample.pitch = result->pitch;
                            sample.yaw = result->yaw;
                            sample.valid = 1;
                            sample.status = IMU_STATUS_RUNNING;
                        } else {
                            sample.status = IMU_STATUS_ALIGNMENT_ERROR;
                        }
                    }
                }
                sample.calibration_count = calibration.count;
            }
            Publish(&sample);
        }
        /* Avoid a burst of immediate catch-up reads after a blocking transfer. */
        if ((TickType_t)(xTaskGetTickCount() - last_wake) >= period)
            last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
    }
}
