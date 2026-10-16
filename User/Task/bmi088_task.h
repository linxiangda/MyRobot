#ifndef _BMI088_TASK_H_
#define _BMI088_TASK_H_
#include "main.h"

/*
 * Drive_BMI088_Read() 输出数据单位
 *
 * 加速度单位：m/s2
 * 角速度单位：rad/s
 * 温度单位：℃
 */
typedef struct{
    float accel[3];
    float gyro[3];
    float temperature;

}IMUData_TypeDef;

void vIMUTask(void *pvParameters);
#endif
