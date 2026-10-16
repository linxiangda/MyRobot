#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>

#include "lowpass.h"
#include "mahony.h"

typedef struct {
    float accel[3];       /* 滤波后加速度，m/s2，包含重力 */
    float gyro[3];        /* 去零偏、滤波后角速度，rad/s */

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

#endif
			   