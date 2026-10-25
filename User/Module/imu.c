#include "imu.h"
#include "bmi088_driver.h"
#include "mahony.h"
#include "lowpass.h"
#include "PID.h"
#include "tim.h"

#define PI 3.14159265f
#define WHEEL_RADIUS 0.04566f
#define WHEEL_TRACK 0.2062f

Mahony_t mahonyfliter;
PID_t temperature_PID;

uint8_t IMU_Init(void){
    float data[7]={0};
    float accel[3]={0};
    PID_Init(&temperature_PID,20,30,0,1000);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

    if(Drive_BMI088_Init() != 0) return 1;
    if(Drive_BMI088_Read(data)) return 1;
    for(uint8_t i=0;i<3;i++){
        accel[i] = data[i];
    }
    
    Mahony_Init(&mahonyfliter, 0.02, 0.02, 0.05);
    if(Mahony_AlignGravity(&mahonyfliter, accel))return 1;
    return 0;
}

void IMU_RawDataGet(IMUData_TypeDef *imudata){
    float data[7]={0};
    if (Drive_BMI088_Read(data)){
        imudata->error = 1;
        return;
    }
    imudata->error = 0;
    for(uint8_t i=0;i<3;i++){
        imudata->accel[i] = data[i];
    }
    for(uint8_t i=0;i<3;i++){
        imudata->gyro[i] = data[i+3];
    }
    imudata->temperature = data[6];
}
LowPassFilter_t motorspeed_lowpass_L;
LowPassFilter_t motorspeed_lowpass_R;
float IMU_UpdateYaw(float left_speed, float right_speed, float gyro_z,float dt){
    static float gyro_yaw = 0;
    static float wheel_yaw = 0;
    float error;
    
    left_speed = LowPass_Update(&motorspeed_lowpass_L, left_speed, 50,dt);
    right_speed = LowPass_Update(&motorspeed_lowpass_R, right_speed, 50,dt);
    float wheel_yaw_rate = WHEEL_RADIUS * (left_speed - right_speed) / WHEEL_TRACK;

    wheel_yaw += wheel_yaw_rate * dt;
    gyro_yaw -= gyro_z * dt;

    error = wheel_yaw - gyro_yaw;
    gyro_yaw += 0.001f * error;
    return gyro_yaw;
}

void IMU_PosCalculate(IMUData_TypeDef *imudata,float left_speed, float right_speed,float dt){
    float accel[3] = {0};
    float gyro[3] = {0};
    float euler_rad[3] = {0};
    for(uint8_t i=0;i<3;i++){
        accel[i] = imudata->accel[i];
        gyro[i] = imudata->gyro[i];
    }
    Mahony_Update(&mahonyfliter, gyro, accel, 1, dt);
    Mahony_GetEuler(&mahonyfliter, euler_rad);
    imudata->roll = euler_rad[0] / PI * 180.0f;
    imudata->pitch = euler_rad[1] / PI * 180.0f;
    imudata->yaw = IMU_UpdateYaw(left_speed, right_speed, gyro[2],dt) / PI * 180.0f;
    // imudata->yaw = euler_rad[2] / PI * 180.0f;
}

void IMU_TemperatureControl(IMUData_TypeDef *imudata, float set_temperature, float dt){
    static uint8_t time = 0;

    if (imudata->error != 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        PID_Reset(&temperature_PID);
        time = 0;
        return;
    }

    time++;
    if(time >= 50){
        float temperature_out = PID_Compute(&temperature_PID,set_temperature,imudata->temperature,dt*(float)time);
        if (temperature_out < 0.0f) temperature_out = 0.0f;
        if (temperature_out > 1000.0f) temperature_out = 1000.0f;
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (int)temperature_out);

        time=0;
    }
}
