#include "can_task.h"
#include "fdcan.h"
#include "can_bsp.h"

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t MotorCanTaskHandle;
extern TaskHandle_t UpperCanTaskHandle;
void  vMotorCanTask(void *pvParameters){
    for(;;){
        vTaskDelay(100);
        
    }

}




void vUpperCanTask(void *pvParameters){
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
	if (CAN_Start(&hfdcan2,0x100,0x7ff) != HAL_OK){
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK){
        Error_Handler();
    }
    for (;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0){
			HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO0,&rx,data);
			if (rx.IdType == FDCAN_STANDARD_ID &&
				rx.RxFrameType == FDCAN_DATA_FRAME &&
				rx.Identifier == 0x100 &&
				rx.DataLength == FDCAN_DLC_BYTES_8)
			{
				HAL_StatusTypeDef result = CAN_Send_Data(&hfdcan2,0x101, data,sizeof(data));
				/* 调试时检查result，正式程序需要处理忙和错误 */
				if (result != HAL_OK){
					Error_Handler();
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
