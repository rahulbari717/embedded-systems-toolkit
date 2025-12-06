/*
 * main.c
 *
 *  Created on: Dec 6, 2025
 *      Author: Rahul B.
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>

UART_HandleTypeDef huart2;


void SystemClkConfig(void);
void UART2_Init(void);
void Error_Handler(void);

char *user_data = "This application is running ...\r\n";


int main(void){

	HAL_Init();
	SystemClkConfig();
	UART2_Init();

	HAL_Delay(100);   // Important!

	uint16_t len = strlen(user_data);
	if(HAL_UART_Transmit(&huart2, (uint8_t *) user_data, len, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}

	uint8_t rcvd_data;
	uint8_t data_buffer[100];
	uint32_t count =0;

	while(1){
		HAL_UART_Receive(&huart2, &rcvd_data, 1, HAL_MAX_DELAY);
		if(rcvd_data == '\r') break;
		else{
			data_buffer[count++] = rcvd_data;
		}
	}
	data_buffer[count++] = '\r';
	if(HAL_UART_Transmit(&huart2, data_buffer, count, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}

	while(1);
	return 0;
}

void SystemClkConfig(void){

}

void UART2_Init(void){
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200; 
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK){
		Error_Handler();
	}
}

void Error_Handler(){
	while(1);
}




