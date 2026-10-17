#ifndef _IMU_H_
#define _IMU_H_

#include "main.h"

typedef struct{
    float accel[3];
    float gyro[3];
    float temperature;
    float quaternion[4];
    float roll,pitch,yaw;
    uint8_t error;
}IMUData_TypeDef;
uint8_t IMU_Init(void);
void IMU_RawDataGet(IMUData_TypeDef *imudata);
void IMU_PosCalculate(IMUData_TypeDef *imudata, float dt);
void IMU_TemperatureControl(IMUData_TypeDef *imudata, float set_temperature, float dt);

#endif
