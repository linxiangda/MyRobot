#ifndef _BMI088_TASK_H_
#define _BMI088_TASK_H_
#include "main.h"

#define IMU_TASK_STACK_WORDS 768U
#define IMU_TASK_PERIOD_MS 1U
#define IMU_CALIBRATION_SAMPLES 1000U
#define IMU_DATA_MAX_AGE_MS 20U

/* Sensor -> body signed axis permutation. Default is identity, NOT a
 * verified board mounting. Use a right-handed body frame: X forward,
 * Y left, Z up. Both accel and gyro use the same mapping.
 * Example: X=+sensorY, Y=-sensorX, Z=+sensorZ: {1,0,2}, {1,-1,1}. */
#define IMU_BODY_AXES  {0, 1, 2}
#define IMU_BODY_SIGNS {1, 1, 1}

typedef enum {
    IMU_STATUS_INITIALIZING = 0,
    IMU_STATUS_WARMUP,
    IMU_STATUS_CALIBRATING,
    IMU_STATUS_RUNNING,
    IMU_STATUS_SENSOR_ERROR,
    IMU_STATUS_TIMING_ERROR,
    IMU_STATUS_ALIGNMENT_ERROR,
    IMU_STATUS_CONFIG_ERROR
} IMUStatus_t;

typedef struct {
    /* Preserve original fields: raw scaled values in BODY axes.
     * accel=m/s2 (specific force), gyro=rad/s, temperature=degrees C. */
    float accel[3];
    float gyro[3];
    float temperature;
    float quaternion[4];       /* [w,x,y,z], body -> world */
    float roll, pitch, yaw;     /* degrees; yaw is relative and drifts */
    float angular_velocity[3]; /* bias-corrected body gyro, rad/s */
    float gyro_bias[3];        /* body axes, rad/s */
    float dt;                 /* actual successful-read interval, seconds */
    uint32_t sample_tick;      /* last successful read, FreeRTOS tick units */
    uint32_t sample_count;
    uint32_t read_errors;
    uint32_t reset_count;      /* calibration/reset invalidates previous yaw */
    uint32_t calibration_count;
    IMUStatus_t status;
    uint8_t calibrated;
    uint8_t valid;
    uint8_t cycle_clock;       /* 1=DWT cycle clock; 0=tick fallback */
} IMUData_TypeDef;

void vIMUTask(void *pvParameters);
/* Task context only. Always copies a consistent snapshot; returns 1 only
 * if calibrated, running, valid and no older than IMU_DATA_MAX_AGE_MS.
 * Prefer this over consuming the log queue in the control task. */
uint8_t IMU_ReadSnapshot(IMUData_TypeDef *out);
/* Task context only; keep robot stationary until status=RUNNING.
 * Calibration resets attitude/yaw. Do not request while motors are enabled. */
void IMU_RequestCalibration(void);
#endif
