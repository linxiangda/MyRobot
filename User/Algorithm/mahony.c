#include "mahony.h"

#include <math.h>
#include <float.h>
#include <string.h>

static float Clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;

    return value;
}

void Normalize(float *v, unsigned int count)
{
    float norm = 0.0f;
    unsigned int i;

    for (i = 0; i < count; i++) {
        norm += v[i] * v[i];
    }

    norm = 1.0f / sqrtf(norm);

    for (i = 0; i < count; i++) {
        v[i] *= norm;
    }
}

void Mahony_Init(Mahony_t *filter, float kp, float ki)
{
    memset(filter, 0, sizeof(*filter));

    filter->q[0] = 1.0f;

    filter->kp = kp;
    filter->ki = ki;

    /* 积分修正的限幅，rad/s */
    filter->integral_limit = 0.05f;
}

uint8_t Mahony_AlignGravity(Mahony_t *filter,
                           const float accel[3])
{
    float a[3];
    float roll;
    float pitch;
    float cr, sr;
    float cp, sp;

    memcpy(a, accel, sizeof(a));

    roll = atan2f(a[1], a[2]);
    pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);

    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    /* 初始 yaw = 0 */
    filter->q[0] = cr * cp;
    filter->q[1] = sr * cp;
    filter->q[2] = cr * sp;
    filter->q[3] = -sr * sp;

    memset(filter->integral, 0, sizeof(filter->integral));

    return 1;
}

uint8_t Mahony_Update(Mahony_t *filter,
                     const float gyro[3],
                     const float accel[3],
                     float accel_weight,
                     float dt)
{
    float a[3];
    float gravity[3];
    float error[3] = {0.0f, 0.0f, 0.0f};
    float integral[3];
    float omega[3];
    float next_q[4];

    float w, x, y, z;
    float half_dt;

    unsigned int i;

    accel_weight = Clamp(accel_weight, 0.0f, 1.0f);

    memcpy(a, accel, sizeof(a));
    memcpy(integral, filter->integral, sizeof(integral));

	/* 取出当前四元数 */
    w = filter->q[0];
    x = filter->q[1];
    y = filter->q[2];
    z = filter->q[3];
    Normalize(a, 3);
    if (accel_weight > 0.0f) {
        /* 当前姿态预测的重力参考方向，表达在机体坐标系中 */
        gravity[0] = 2.0f * (x * z - w * y);
        gravity[1] = 2.0f * (y * z + w * x);
        gravity[2] = w * w - x * x - y * y + z * z;

        /* 测量方向 × 预测方向 */
        error[0] = a[1] * gravity[2] - a[2] * gravity[1];
        error[1] = a[2] * gravity[0] - a[0] * gravity[2];
        error[2] = a[0] * gravity[1] - a[1] * gravity[0];

        for (i = 0; i < 3; i++) {
            error[i] *= accel_weight;

            integral[i] += filter->ki * error[i] * dt;

            integral[i] = Clamp(integral[i],
                                -filter->integral_limit,
                                 filter->integral_limit);
        }
    }

    for (i = 0; i < 3; i++) {
        if (filter->ki == 0.0f) {
            integral[i] = 0.0f;
        }

        omega[i] = gyro[i]
                 + filter->kp * error[i]
                 + integral[i];
    }

    /* q_dot = 0.5 * q ? [0, omega] */
    half_dt = 0.5f * dt;

    next_q[0] = w + (-x * omega[0]
                     -y * omega[1]
                     -z * omega[2]) * half_dt;

    next_q[1] = x + ( w * omega[0]
                     +y * omega[2]
                     -z * omega[1]) * half_dt;

    next_q[2] = y + ( w * omega[1]
                     -x * omega[2]
                     +z * omega[0]) * half_dt;

    next_q[3] = z + ( w * omega[2]
                     +x * omega[1]
                     -y * omega[0]) * half_dt;

    memcpy(filter->q, next_q, sizeof(next_q));
    memcpy(filter->integral, integral, sizeof(integral));

    return 1;
}

void Mahony_GetEuler(const Mahony_t *filter,
                     float euler_rad[3])
{
    float w = filter->q[0];
    float x = filter->q[1];
    float y = filter->q[2];
    float z = filter->q[3];

    /* Roll */
    euler_rad[0] = atan2f(2.0f * (w * x + y * z),
                         1.0f - 2.0f * (x * x + y * y));

    /* Pitch */
    euler_rad[1] = asinf(
        Clamp(2.0f * (w * y - z * x), -1.0f, 1.0f)
    );

    /* Yaw */
    euler_rad[2] = atan2f(2.0f * (w * z + x * y),
                         1.0f - 2.0f * (y * y + z * z));
}
