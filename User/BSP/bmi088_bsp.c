#include "bmi088_bsp.h"

#include "main.h"
#include "spi.h"

/* dev: 0 = 加速度计，1 = 陀螺仪 */
static void BSP_BMI088_Select(uint8_t dev, GPIO_PinState state)
{
    if (dev == 0)
    {
        HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port,BMI088_ACC_CS_Pin, state);
    }
    else
    {
        HAL_GPIO_WritePin(BMI088_GYR_CS_GPIO_Port,BMI088_GYR_CS_Pin, state);
    }
}

void BSP_BMI088_Init(void)
{
    /* 调用前必须完成GPIO和SPI的初始化;此处置高释放片选 */
    BSP_BMI088_Select(0, GPIO_PIN_SET);
    BSP_BMI088_Select(1, GPIO_PIN_SET);
}

void BSP_BMI088_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

int BSP_BMI088_Read(uint8_t dev, uint8_t reg,uint8_t *data, uint16_t len)
{
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    HAL_StatusTypeDef status;

    if (dev > 1 || data == 0 || len == 0)
        return -1;

	/* 片选 */
    BSP_BMI088_Select(dev, GPIO_PIN_RESET);

    status = HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY); //指定寄存器交换出数据

    /* 加速度计读地址之后，需要额外一个 dummy byte */
    if (status == HAL_OK && dev == 0)
    {
        tx = 0x00;
        status = HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    }

    tx = 0x00;

    for (uint16_t i = 0; status == HAL_OK && i < len; i++)
    {
        status = HAL_SPI_TransmitReceive(&hspi1, &tx, &data[i],1, HAL_MAX_DELAY);
    }

    /* 释放片选 */
    BSP_BMI088_Select(dev, GPIO_PIN_SET);

    return status == HAL_OK ? 0 : -1;
}

int BSP_BMI088_Write(uint8_t dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value};
    HAL_StatusTypeDef status;

    if (dev > 1)
        return -1;

    BSP_BMI088_Select(dev, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);

    BSP_BMI088_Select(dev, GPIO_PIN_SET);

    return status == HAL_OK ? 0 : -1;
}
