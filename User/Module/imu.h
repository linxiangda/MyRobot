#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>

#include "lowpass.h"
#include "mahony.h"

typedef struct {
    float accel[3];       /* 滤波后加速度，m/s2，包含重力 */
    float gyro[3];        /* 去零偏、滤波后角速度，rad/s */

    /* [w,x,y,z], body -> world; roll/pitch/yaw below are degrees. */
    float quaternion[4];

    float roll;           /* degree */
    float pitch;          /* degree */
    float yaw;            /* degree */

    uint8_t valid;        /* 当前姿态是否有效 */
} IMUResult_t;

typedef struct {
    LowPassFilter_t accel_filter[3];
    LowPassFilter_t gyro_filter[3];

    Mahony_t mahony;

    float gyro_bias[3];

    uint8_t aligned;

    IMUResult_t result;
} IMU_t;

void IMU_Init(IMU_t *imu);

/* bias 必须使用 rad/s，并与传入的 gyro 坐标轴一致 */
void IMU_SetGyroBias(IMU_t *imu, const float bias[3]);

/*
 * accel：m/s2
 * gyro ：rad/s
 * dt   ：秒
 *
 * 返回 1：本次姿态更新成功
 * 返回 0：数据/时间异常，或尚未完成初始对齐
 */
uint8_t IMU_Update(IMU_t *imu,
                   const float accel[3],
                   const float gyro[3],
                   float dt);

const IMUResult_t *IMU_GetResult(const IMU_t *imu);

/* Complete sensor/attitude module. Only the IMU task calls Init/Update.
 * No FreeRTOS dependency. Init may block inside the existing sensor driver.
 * Update performs one read and one calibration/estimation step; no task waits.
 * Axes below preserve the existing mounting configuration.
 */
#define IMU_CALIBRATION_SAMPLES 1000U
#define IMU_DATA_MAX_AGE_MS 20U

/* Sensor -> body signed axis permutation. X=+sensorY, Y=-sensorX, Z=+sensorZ.
 * This is a provisional mounting assumption, NOT a
 * verified board mounting. Use a right-handed body frame: X forward,
 * Y left, Z up. Both accel and gyro use the same mapping.
 * Example: X=+sensorY, Y=-sensorX, Z=+sensorZ: {1,0,2}, {1,-1,1}. */
#define IMU_BODY_AXES  {1, 0, 2}
#define IMU_BODY_SIGNS {1, -1, 1}

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
    uint32_t sample_tick;      /* last successful read, HAL millisecond units */
    uint32_t sample_count;
    uint32_t read_errors;
    uint32_t reset_count;      /* calibration/reset invalidates previous yaw */
    uint32_t calibration_count;
    IMUStatus_t status;
    uint8_t calibrated;
    uint8_t valid;
    uint8_t cycle_clock;       /* 1=DWT cycle clock; 0=tick fallback */
} IMUData_TypeDef;


/* 0 = sensor initialized; attitude becomes valid after stationary calibration. */
int IMU_ModuleInit(void);
/* 1 = this sample has a valid attitude; 0 = calibrating or fault. */
uint8_t IMU_ModuleUpdate(void);
/* Copies a coherent snapshot; age > IMU_DATA_MAX_AGE_MS marks it invalid. */
void IMU_ModuleGetData(IMUData_TypeDef *out);
/* Compatibility API used by current log task: 1 only for fresh valid attitude. */
uint8_t IMU_ReadSnapshot(IMUData_TypeDef *out);
/* Request from task context; next Update starts calibration and resets yaw. */
void IMU_RequestCalibration(void);

#endif
