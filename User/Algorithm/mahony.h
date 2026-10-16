#ifndef _MAHONY_H_
#define _MAHONY_H_

#include <stdint.h>

typedef struct {
    float q[4];

    float kp;
    float ki;

    float integral[3];
    float integral_limit;
} Mahony_t;

void Mahony_Init(Mahony_t *filter, float kp, float ki);

/* 根据静止加速度初始化倾角，Yaw 初始化为 0 */
uint8_t Mahony_AlignGravity(Mahony_t *filter, const float accel[3]);

/*
 * accel_weight：0~1
 * 0：仅使用陀螺仪积分
 * 1：完整使用加速度方向修正
 *
 * 返回 1 表示更新成功
 */
uint8_t Mahony_Update(Mahony_t *filter, const float gyro[3], const float accel[3], float accel_weight, float dt);

void Mahony_GetEuler(const Mahony_t *filter, float euler_rad[3]);

#endif
