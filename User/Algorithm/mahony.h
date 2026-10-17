#ifndef _MAHONY_H_
#define _MAHONY_H_
#include <stdint.h>

typedef struct {
    float q[4];
    float kp;
    float ki;
    float error_int[3];
    float integral_limit;
} Mahony_t;

void Mahony_Init(Mahony_t *filter, float kp, float ki, float integral_limit);
uint8_t Mahony_AlignGravity(Mahony_t *filter, const float accel[3]);
uint8_t Mahony_Update(Mahony_t *filter, const float gyro[3], const float accel[3], float accel_weight, float dt);
void Mahony_GetEuler(const Mahony_t *filter, float euler_rad[3]);
#endif

