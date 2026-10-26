#include "mahony.h"

#include <math.h>
#include <float.h>
#include <string.h>

/* 限幅函数 */
static float Clamp(float value, float min, float max){
    if (value < min) return min;
    if (value > max) return max;

    return value;
}

/* 数据正常检查函数,正常为0 */
static uint8_t NFinite(float x){
    return !(x == x && fabsf(x) <= FLT_MAX);
}

/* 归一化函数，向量方向不变，长度变为1 */
static uint8_t Normalize(float *v, uint8_t count){
    float norm = 0.0f;
    unsigned int i;
    for (i = 0; i < count; i++) {
        if (NFinite(v[i])) return 1;
        norm += v[i] * v[i];
    }
    if (NFinite(norm) || norm < 1.0e-12f) return 1;

    norm = 1.0f / sqrtf(norm);
    for (i = 0; i < count; i++){
        v[i] *= norm;
    } 
    return 0;
}

/* 初始化Mahony参数 */
void Mahony_Init(Mahony_t *filter, float kp, float ki, float integral_limit){
    memset(filter, 0, sizeof(*filter));
    filter->q[0] = 1.0f; //q = [1, 0, 0, 0]
    filter->kp = kp;
    filter->ki = ki;
    filter->integral_limit = integral_limit;
    memset(filter->error_int, 0, sizeof(filter->error_int));
}

/* 确定初始四元数姿态 */
uint8_t Mahony_AlignGravity(Mahony_t *filter, const float accel[3]){
    float a[3];
    float roll;
    float pitch;
    float cr, sr;
    float cp, sp;

    if (filter == NULL || accel == NULL) return 1;
    memcpy(a, accel, sizeof(a));
    if (Normalize(a, 3)) return 1;

    /* 启动时根据静止加速度方向估计横滚、俯仰,初始Yaw设为0 */
    roll = atan2f(a[0], a[2]);
    pitch = atan2f(a[1], sqrtf(a[0] * a[0] + a[2] * a[2]));
    
    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);

    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    filter->q[0] = cr * cp;
    filter->q[1] = cr * sp;
    filter->q[2] = -sr * cp;
    filter->q[3] = -sr * sp;

    return 0;
}

/* 更新姿态 */
uint8_t Mahony_Update(Mahony_t *filter, const float gyro[3], const float accel[3], float accel_weight, float dt){
    float a[3];
    float gravity[3];
    float error[3] = {0.0f, 0.0f, 0.0f};
    float error_int[3];
    float omega[3];
    float next_q[4];

    float w, x, y, z;
    float half_dt;

    /* 入口处合法性校验 */
    if (gyro == NULL || accel == NULL)
        return 1;

    /* 权重限幅 */
    accel_weight = Clamp(accel_weight, 0.0f, 1.0f);

    /* 读取加速度(因为a会被修改备份一下) */
    memcpy(a, accel, sizeof(a));

    /* 读取上次计算四元数与上次积分 */
    memcpy(error_int, filter->error_int, sizeof(error_int));
    w = filter->q[0];
    x = filter->q[1];
    y = filter->q[2];
    z = filter->q[3];
    
    /*** 核心滤波代码(本质就是让预测的重力方向逐渐对齐加速度测得的方向，使用kp，ki进行校准) ***/
    if (accel_weight > 0.0f) {
        if (Normalize(a, 3)) return 1;
        gravity[0] = 2.0f * (x * z - w * y);
        gravity[1] = 2.0f * (y * z + w * x);
        gravity[2] = w * w - x * x - y * y + z * z;
        /* error = 测得方向 a × 预测方向 gravity */
        error[0] = a[1] * gravity[2] - a[2] * gravity[1];
        error[1] = a[2] * gravity[0] - a[0] * gravity[2];
        error[2] = a[0] * gravity[1] - a[1] * gravity[0];

        for (uint8_t i = 0; i < 3; i++) {
            error[i] *= accel_weight;
            error_int[i] += error[i]*dt;
            error_int[i] = Clamp(error_int[i], -filter->integral_limit, filter->integral_limit);
        }
    }

    for (uint8_t i = 0; i < 3; i++) {
        omega[i] = gyro[i] + filter->kp*error[i] + filter->ki*error_int[i];
    }
    half_dt = 0.5f * dt;
    next_q[0] = w + (- x*omega[0] - y*omega[1] - z*omega[2]) * half_dt;
    next_q[1] = x + ( w*omega[0] + y*omega[2] - z*omega[1]) * half_dt;
    next_q[2] = y + ( w*omega[1] - x*omega[2] + z*omega[0]) * half_dt;
    next_q[3] = z + ( w*omega[2] + x*omega[1] - y*omega[0]) * half_dt;        
    if (Normalize(next_q, 4)) return 1;
    /*** 核心滤波代码(本质就是让预测的重力方向逐渐对齐加速度测得的方向，使用kp，ki进行校准) ***/

    /* 保存本次融合后的四元数和误差积分 */
    memcpy(filter->q, next_q, sizeof(next_q));
    memcpy(filter->error_int, error_int, sizeof(error_int));

    return 0;
}

/* 把四元数转换成角度 */
void Mahony_GetEuler(const Mahony_t *filter, float euler_rad[3]){
    float w = filter->q[0];
    float x = filter->q[1];
    float y = filter->q[2];
    float z = filter->q[3];

    /* Roll */
    euler_rad[0] = atan2f(2.0f * (w * y - x * z), 1.0f - 2.0f * (x * x + y * y));
    /* Pitch */
    euler_rad[1] = asinf(Clamp(2.0f * (w * x + y * z), -1.0f, 1.0f));
    /* Yaw */
    euler_rad[2] = atan2f(2.0f * (w * z - x * y), 1.0f - 2.0f * (x * x + z * z));
}
