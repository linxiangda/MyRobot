#ifndef _CAN_TASK_H_
#define _CAN_TASK_H_

#include "main.h"
typedef struct{
    float motor_iq[4];
    float motor_speed[4];
    float motor_position[4];
    uint8_t motor_mode[4];
    uint8_t motor_error[4];
    uint8_t motor_calibrated[4];
    uint8_t motor_calibrating[4];
}MotorData_TypeDef;


void  vMotorCanTask(void *pvParameters);
void vUpperCanTask(void *pvParameters);
#endif
