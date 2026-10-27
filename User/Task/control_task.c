#include "control_task.h"
#include "imu_task.h"
#include "can_task.h"
#include "log_task.h"

#include "wheel_control.h"
#include "can_bsp.h"
#include "fdcan.h"

#include <stdio.h>
#include "string.h"
#include <stddef.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#define LEG_Torque_constant 0.61f /* N*M/A */
#define Wheel_Torque_constant 0.3f /* N*M/A */

extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xMotorQueue;
extern EventGroupHandle_t xEventGroup;

void vControlLegTask(void *pvParameters){	
	for(;;){
        vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void vControlWheelTask(void *pvParameters){	
    static IMUData_TypeDef IMUdata;
    static MotorData_TypeDef Motordata;
    TickType_t period = pdMS_TO_TICKS(5);
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t was_enabled = 0;// 使能一次执行一次
    float position_ref = 0.0f;
    float yaw_ref = 0.0f;
	for(;;){
        /* 读取失败，取消按键使能 */
        if (xQueuePeek(xIMUQueue, &IMUdata, 0) != pdTRUE || xQueuePeek(xMotorQueue, &Motordata, 0) != pdTRUE){
            xEventGroupClearBits(xEventGroup, 1U);
        }
        /* 数据超时，取消按键使能 */
        else{
            uint32_t now = HAL_GetTick();
            if ((uint32_t)(now - IMUdata.update_ms) > 20){
                xEventGroupClearBits(xEventGroup, 1U);
            }

            for (uint8_t i = 0; i < 4; i++) {
                if ((uint32_t)(now - Motordata.update_ms[i]) > 50) {
                    xEventGroupClearBits(xEventGroup, 1U);
                    break;
                }
            }
        }
        /* 倾角过大，取消按键使能 */
        const float pitch_limit = 20.0f * 3.1415926f / 180.0f;
        if (fabsf(IMUdata.pitch - 0.002f) > pitch_limit) {
            xEventGroupClearBits(xEventGroup, 1U);
        }

        /* 事件组准备就绪，才允许开始控制 */
        const EventBits_t required = (1U << 0) | (1U << 1) | (1U << 2);
        EventBits_t bits = xEventGroupGetBits(xEventGroup);
        if ((bits & required) != required) {
            /* 未使能或设备未就绪,发送零电流 */
            uint8_t stop_data[8] = {0};
            uint32_t mode = 0; 
            float data = 0.0f;

            memcpy(&stop_data[0], &mode, sizeof(mode));
            memcpy(&stop_data[4], &data, sizeof(data));

            CAN_Send_Data(&hfdcan1, 0x102, stop_data, 8);
            CAN_Send_Data(&hfdcan1, 0x104, stop_data, 8);

            was_enabled = 0;
            vTaskDelay(period);
            last_wake = xTaskGetTickCount();
            continue;
        }

        /* 使能首次赋值当前位置和偏航 */
        if (was_enabled == 0){
            position_ref = IMUdata.position;
            yaw_ref = IMUdata.yaw;
            was_enabled = 1;
        }
        float state[6] = {
            IMUdata.position - position_ref,   
            IMUdata.speed - 0.0f,          
            IMUdata.pitch - 0.002f,       
            IMUdata.gyro[0] - 0.0f,                
            IMUdata.yaw - yaw_ref,             
            IMUdata.gyro[2] - 0.0f    
        };
        float left_torque,right_torque;
        Wheel_Control(state, &left_torque, &right_torque);
        float left_current  = -left_torque  / Wheel_Torque_constant;
        float right_current = -right_torque / Wheel_Torque_constant;

        uint8_t left_wheel_data[8]  = {0};
        uint8_t right_wheel_data[8] = {0};
        uint32_t mode = 1;
        memcpy(&left_wheel_data[0],  &mode, sizeof(mode));
        memcpy(&right_wheel_data[0], &mode, sizeof(mode));
        memcpy(&left_wheel_data[4],&left_current,sizeof(left_current));
        memcpy(&right_wheel_data[4],&right_current,sizeof(right_current));

        CAN_Send_Data(&hfdcan1,0x102, left_wheel_data,8);
        CAN_Send_Data(&hfdcan1,0x104, right_wheel_data,8);
        // char packet[32];
        // int len;
        // len = snprintf(packet, sizeof(packet),"%.4f,%.4f\n",left_torque,right_torque);
        // static uint16_t i = 0;
        // i++;
        // if (len > 0 && (size_t)len < sizeof(packet) && i>=5){
        //     i=0;
        //     print(packet);
        // }   
        if((xTaskGetTickCount() - last_wake >= period)) last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
	}
}
