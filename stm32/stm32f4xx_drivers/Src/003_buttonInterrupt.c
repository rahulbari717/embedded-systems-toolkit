/*
 * 003_buttonInterrupt.c
 *
 *  Created on: Nov 25, 2025
 *      Author: Rahul B.
 */


#include <stm32f446xx.h>
#include <string.h>

#define LOW 			DISABLE
#define BTN_PRESSED 	LOW

void delay(void){
	for(int i=0; i<500000/2; i++);
}

int main(){
	GPIO_Handle_t GpioLed, GpioButton;
	memset(&GpioButton, 0, sizeof(GpioButton));
	memset(&GpioLed, 0, sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	// Configure IRQ priority and enable interrupt
	GPIO_IRQPriorityConfig(EXTI15_10_IRQn, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(EXTI15_10_IRQn, ENABLE);

//	while(1){
//		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED){
//			delay();
//			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
//		}
//	}

	return 0;
}

// IRQ Handler for EXTI15_10 (handles PC13)
void EXTI15_10_IRQHandler(void){
	delay(); // Simple debounce
	GPIO_IRQHandling(GPIO_PIN_NO_13); // Clear the pending bit
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5); // Toggle LED
}

