#ifndef _BMI088_DRIVER_H_
#define _BMI088_DRIVER_H_

#include <stdint.h>

/* Drive_BMI088_Read()输出数组的长度 */
#define BMI088_DATA_LENGTH           7

/* 函数返回值 */
#define BMI088_OK                    0
#define BMI088_ERROR_COMM           -1
#define BMI088_ERROR_ACC_ID         -2
#define BMI088_ERROR_GYR_ID         -3
#define BMI088_ERROR_ACC_CONFIG     -4
#define BMI088_ERROR_GYR_CONFIG     -5

int Drive_BMI088_Init(void);
int Drive_BMI088_Read(float data[BMI088_DATA_LENGTH]);

#endif
