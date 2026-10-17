#include "imu.h"
#include "bmi088_driver.h"
#include "mahony.h"
#include "lowpass.h"

#define PI 3.14159265f
Mahony_t mahonyfliter;
uint8_t IMU_Init(void){
    float data[7]={0};
    float accel[3]={0};
    Drive_BMI088_Init();
    Drive_BMI088_Read(data);
    for(uint8_t i=0;i<3;i++){
        accel[i] = data[i];
    }
    
    Mahony_Init(&mahonyfliter, 0.02, 0.02, 0.05);
    if(Mahony_AlignGravity(&mahonyfliter, accel)) return 1;
    return 0;

}

void IMU_RawDataGet(IMUData_TypeDef *imudata){
    float data[7]={0};
    Drive_BMI088_Read(data);
    for(uint8_t i=0;i<3;i++){
        imudata->accel[i] = data[i];
    }
    for(uint8_t i=0;i<3;i++){
        imudata->gyro[i] = data[i+3];
    }
    imudata->temperature = data[6];
}

float KP = 0.02;
float KI = 0.01;
void only_pitch(IMUData_TypeDef *imudata,float dt){
    static uint8_t first = 0;
    float error;
    static float error_int;

    imudata->pitch_acc = -atan2f(imudata->accel[1],imudata->accel[2]) / PI * 180.0f;
    if(!first){
        imudata->pitch_gyro = imudata->pitch_acc;
        first = 1;
        return;
    }
    error = imudata->pitch_acc - imudata->pitch_gyro;
    error_int += error*dt;

    imudata->pitch_gyro -= imudata->gyro[0] * dt / PI * 180;
    imudata->pitch_gyro = imudata->pitch_gyro + KP*error + KI*error_int;
}

void IMU_PosCalculate(IMUData_TypeDef *imudata,float dt){
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
    imudata->yaw = euler_rad[2] / PI * 180.0f;
}
