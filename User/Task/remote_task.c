#include "remote_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "usart.h"
#include "main.h"
uint8_t rxbuffer[5];
void vRemoteTask(void *pvParameters){
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rxbuffer,sizeof(rxbuffer));
//	HAL_UART_Receive_DMA(&huart1,rxbuffer,sizeof(rxbuffer));
	for(;;){
		vTaskDelay(pdMS_TO_TICKS(10));

	}
	
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size){ // 每次串口空闲或者接收满缓存区触发中断，写入
	if(huart == &huart1){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rxbuffer,sizeof(rxbuffer));
		if(rxbuffer[0] == '1'){
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ // 接收满缓存区触发中断，写入
	if(huart == &huart1){
		HAL_UART_Receive_DMA(&huart1,rxbuffer,sizeof(rxbuffer));
		if(rxbuffer[0] == '1'){
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		}
	}
}
