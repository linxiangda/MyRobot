#include "can_task.h"
#include "fdcan.h"
#include "can_bsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

extern TaskHandle_t MotorCanTaskHandle;
extern TaskHandle_t UpperCanTaskHandle;
extern MotorData_TypeDef motordata;
extern QueueHandle_t xMotorQueue;

void  vMotorCanTask(void *pvParameters){
	FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
	if (CAN_Start(&hfdcan1,0x200,0x7e8) != HAL_OK){
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK){
        Error_Handler();
    }
    for(;;){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		/* 读出全部报文 */
		while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0){
			HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&header,data);
			if (header.IdType == FDCAN_STANDARD_ID &&
				header.RxFrameType == FDCAN_DATA_FRAME &&
				// header.Identifier == 0x200 &&
				header.DataLength == FDCAN_DLC_BYTES_8)
			{
				if((header.Identifier & 0x0f0) == 0){
					memcpy(&motordata.motor_iq[(header.Identifier & 0x00f)-1],&data[0],4);
					memcpy(&motordata.motor_speed[(header.Identifier & 0x00f)-1],&data[4],4);
				}
				else{
					memcpy(&motordata.motor_position[(header.Identifier & 0x00f)-1],&data[0],4);
					motordata.motor_mode[(header.Identifier & 0x00f)-1] = data[4];		
					motordata.motor_error[(header.Identifier & 0x00f)-1] = data[5];	
					motordata.motor_calibrated[(header.Identifier & 0x00f)-1] = data[6];	
					motordata.motor_calibrating[(header.Identifier & 0x00f)-1] = data[7];	
				}

				// HAL_StatusTypeDef result = CAN_Send_Data(&hfdcan2,0x101, data,sizeof(data));
				// if (result == HAL_ERROR){
				// 	Error_Handler();
				// }
				// else if(result == HAL_BUSY){
				// 	break;
				// }
			}
		}
		xQueueOverwrite(xMotorQueue, &motordata);
    }

}

void vUpperCanTask(void *pvParameters){
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
	if (CAN_Start(&hfdcan2,0x400,0x7fe) != HAL_OK){
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK){
        Error_Handler();
    }
    for (;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		/* 读出全部报文 */
		while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0){
			HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO0,&header,data);
			if (header.IdType == FDCAN_STANDARD_ID &&
				header.RxFrameType == FDCAN_DATA_FRAME &&
				// header.Identifier == 0x400 &&
				header.DataLength == FDCAN_DLC_BYTES_8)
			{
				HAL_StatusTypeDef result = CAN_Send_Data(&hfdcan2,0x300, data,sizeof(data));
				if (result == HAL_ERROR){
					Error_Handler();
				}
				else if(result == HAL_BUSY){
					break;
				}
			}
		}
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if (hfdcan->Instance == FDCAN1 && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0){
        vTaskNotifyGiveFromISR(MotorCanTaskHandle,&higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }

    if (hfdcan->Instance == FDCAN2 && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0){
        vTaskNotifyGiveFromISR(UpperCanTaskHandle,&higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}
