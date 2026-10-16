#include "bmi088_driver.h"
#include "bmi088_bsp.h"

#define BMI088_ACC  0
#define BMI088_GYR  1

/* 加速度 ±6g，角速度 ±2000°/s */
#define ACC_SCALE   (6.0f * 9.81f / 32768.0f)
#define GYR_SCALE   (2000.0f * 0.0174532925199433f / 32768.0f)

static int16_t BMI088_ToInt16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int BMI088_CheckId(uint8_t dev, uint8_t expected)
{
    uint8_t id;

    if (BSP_BMI088_Read(dev, 0x00, &id, 1) != 0)
        return -1;

    return id == expected ? 0 : -1;
}

static int BMI088_WriteCheck(uint8_t dev, uint8_t reg, uint8_t value)
{
    uint8_t readback;

    if (BSP_BMI088_Write(dev, reg, value) != 0)
        return -1;

    BSP_BMI088_DelayMs(2);

    if (BSP_BMI088_Read(dev, reg, &readback, 1) != 0)
        return -1;

    return readback == value ? 0 : -1;
}

/*
 * 返回值：
 *  0：成功
 * -1：SPI 通信失败或复位失败
 * -2：加速度计 ID 错误
 * -3：陀螺仪 ID 错误
 * -4：加速度计配置失败
 * -5：陀螺仪配置失败
 */
int Drive_BMI088_Init(void)
{
    uint8_t dummy;

    BSP_BMI088_Init();// 释放片选
    BSP_BMI088_DelayMs(80);

    /* 首次访问用于将加速度计切换到 SPI 接口 */
    if (BSP_BMI088_Read(BMI088_ACC, 0x00, &dummy, 1) != 0)
        return -1;

    BSP_BMI088_DelayMs(2);

    /* 加速度计和陀螺仪分别软复位 */
    if (BSP_BMI088_Write(BMI088_ACC, 0x7E, 0xB6) != 0)
        return -1;

    if (BSP_BMI088_Write(BMI088_GYR, 0x14, 0xB6) != 0)
        return -1;

    BSP_BMI088_DelayMs(80);

    /* 加速度计复位后，重新进行 SPI 接口激活访问 */
    if (BSP_BMI088_Read(BMI088_ACC, 0x00, &dummy, 1) != 0)
        return -1;

    BSP_BMI088_DelayMs(2);

    if (BMI088_CheckId(BMI088_ACC, 0x1E) != 0)
        return -2;

    if (BMI088_CheckId(BMI088_GYR, 0x0F) != 0)
        return -3;

    /* 加速度计：开启、正常模式、800Hz、±6g */
    if (BMI088_WriteCheck(BMI088_ACC, 0x7D, 0x04) != 0)
        return -4;

    if (BMI088_WriteCheck(BMI088_ACC, 0x7C, 0x00) != 0)
        return -4;

    if (BMI088_WriteCheck(BMI088_ACC, 0x40, 0xAB) != 0)
        return -4;

    if (BMI088_WriteCheck(BMI088_ACC, 0x41, 0x01) != 0)
        return -4;

    /* 陀螺仪：±2000°/s、2000Hz / 230Hz 带宽、正常模式 */
    if (BMI088_WriteCheck(BMI088_GYR, 0x0F, 0x00) != 0)
        return -5;

    if (BMI088_WriteCheck(BMI088_GYR, 0x10, 0x81) != 0)
        return -5;

    if (BMI088_WriteCheck(BMI088_GYR, 0x11, 0x00) != 0)
        return -5;

    BSP_BMI088_DelayMs(80);
    return 0;
}

/* 成功才更新输出，避免出现半帧数据 */
int Drive_BMI088_Read(float data[7])
{
    uint8_t acc[6];
    uint8_t gyr[6];
    uint8_t temp[2];
    float sample[7];
	uint16_t raw_u11;
    int16_t raw_temp;

    if (data == 0)
        return -1;
	
	/* 读取加速度数据*/
    if (BSP_BMI088_Read(BMI088_ACC, 0x12, acc, 6) != 0)
        return -1;

    /* 读取陀螺仪数据 */
    if (BSP_BMI088_Read(BMI088_GYR, 0x02, gyr, 6) != 0)
        return -1;

    /* 读取温度数据 */
    if (BSP_BMI088_Read(BMI088_ACC, 0x22, temp, 2) != 0)
        return -1;

    for (uint8_t i = 0; i <= 2; i++)
    {
        sample[i] = BMI088_ToInt16(&acc[2 * i]) * ACC_SCALE;
        sample[3 + i] = BMI088_ToInt16(&gyr[2 * i]) * GYR_SCALE;
    }

    /* 温度为 11 位有符号数据 */
	raw_u11  = ((uint16_t)temp[0] << 3);
    raw_u11 |= ((uint16_t)temp[1] >> 5);
    if (raw_u11 >= 1024U)
        raw_temp = (int16_t)(raw_u11 - 2048);
    else
        raw_temp = (int16_t)raw_u11;
    sample[6] = raw_temp * 0.125f + 23.0f;

    for (uint8_t i = 0; i < 7; i++)
        data[i] = sample[i];

    return 0;
}

