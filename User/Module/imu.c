#include "imu.h"

#include <math.h>
#include <float.h>
#include <string.h>

#define IMU_GRAVITY       9.81f
#define IMU_RAD_TO_DEG    57.2957795131f

static uint8_t IsFinite(float value)
{
    return value == value && fabsf(value) <= FLT_MAX;
}

static void ResetFilters(IMU_t *imu)
{
    unsigned int i;

    for (i = 0; i < 3; i++) {
        /* 加速度软件低通：40 Hz */
        LowPass_Init(&imu->accel_filter[i], 40.0f);

        /* 陀螺仪软件低通暂时旁路 */
        LowPass_Init(&imu->gyro_filter[i], 0.0f);
    }
}

void IMU_Init(IMU_t *imu)
{
    memset(imu, 0, sizeof(*imu));

    ResetFilters(imu);

    /* 先使用比例修正，积分修正暂不开启 */
    Mahony_Init(&imu->mahony, 2.0f, 0.0f);

    imu->result.quaternion[0] = 1.0f;
}

void IMU_SetGyroBias(IMU_t *imu, const float bias[3])
{
    unsigned int i;

    for (i = 0; i < 3; i++) {
        imu->gyro_bias[i] = bias[i];

        /* 清除滤波器中旧零偏对应的历史数据 */
        LowPass_Init(&imu->gyro_filter[i],
                     imu->gyro_filter[i].cutoff_hz);
    }

    memset(imu->mahony.integral, 0,
           sizeof(imu->mahony.integral));

    imu->result.valid = 0;
}

uint8_t IMU_Update(IMU_t *imu,
                   const float accel[3],
                   const float gyro[3],
                   float dt)
{
    float accel_norm;
    float accel_weight;
    float euler_rad[3];

    unsigned int i;

    imu->result.valid = 0;

    /*
     * 间隔过长时，本版选择重新对齐。
     * 重新对齐后的 yaw 会从 0 开始。
     */
    if (!IsFinite(dt) || dt <= 0.0f || dt > 0.05f) {
        imu->aligned = 0;
        ResetFilters(imu);
        return 0;
    }

    for (i = 0; i < 3; i++) {
        if (!IsFinite(accel[i]) || !IsFinite(gyro[i])) {
            return 0;
        }
    }

    accel_norm = sqrtf(accel[0] * accel[0]
                     + accel[1] * accel[1]
                     + accel[2] * accel[2]);

    if (!IsFinite(accel_norm)) {
        return 0;
    }

    if (!imu->aligned) {
        /*
         * 初次对齐时应保持静止。
         * 模长接近 1g 只是必要检查，不能保证确实静止。
         */
        if (fabsf(accel_norm - IMU_GRAVITY)
            > 0.1f * IMU_GRAVITY) {
            return 0;
        }

        if (!Mahony_AlignGravity(&imu->mahony, accel)) {
            return 0;
        }

        imu->aligned = 1;

        /* 首次只对齐，不积分此前未知的运动 */
        for (i = 0; i < 3; i++) {
            imu->result.accel[i] =
                LowPass_Update(&imu->accel_filter[i],
                               accel[i], dt);

            imu->result.gyro[i] =
                LowPass_Update(&imu->gyro_filter[i],
                               gyro[i] - imu->gyro_bias[i], dt);
        }
    } else {
        for (i = 0; i < 3; i++) {
            imu->result.accel[i] =
                LowPass_Update(&imu->accel_filter[i],
                               accel[i], dt);

            imu->result.gyro[i] =
                LowPass_Update(&imu->gyro_filter[i],
                               gyro[i] - imu->gyro_bias[i], dt);
        }

        /*
         * 加速度模长越偏离 1g，越少相信它代表重力方向。
         * 偏差达到 0.2g 时，本次只进行陀螺仪积分。
         */
        accel_weight =
            1.0f - fabsf(accel_norm - IMU_GRAVITY)
                 / (0.2f * IMU_GRAVITY);

        if (accel_weight < 0.0f) {
            accel_weight = 0.0f;
        }

        if (!Mahony_Update(&imu->mahony,
                           imu->result.gyro,
                           imu->result.accel,
                           accel_weight,
                           dt)) {
            return 0;
        }
    }

    memcpy(imu->result.quaternion,
           imu->mahony.q,
           sizeof(imu->result.quaternion));

    Mahony_GetEuler(&imu->mahony, euler_rad);

    imu->result.roll  = euler_rad[0] * IMU_RAD_TO_DEG;
    imu->result.pitch = euler_rad[1] * IMU_RAD_TO_DEG;
    imu->result.yaw   = euler_rad[2] * IMU_RAD_TO_DEG;

    imu->result.valid = 1;

    return 1;
}

const IMUResult_t *IMU_GetResult(const IMU_t *imu)
{
    return &imu->result;
}
