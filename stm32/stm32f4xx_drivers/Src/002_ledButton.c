/*
 * 002_ledButton.c
 *
 *  Created on: Nov 25, 2025
 	Author: Rahul B.
 *  when button pressed led ON otherwise stop.
 *
 */

#include <stm32f446xx.h>

void delay(void){
	for(int i=0; i<500000/2; i++);
}

void init(void){
	GPIO_Handle_t GpioLed, GpioButton;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

}


int main(){

	init();

	while(1){
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED){
		    // Button is pressed: Turn LED ON
		    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_SET);
		    delay();
		}else{
			// Button is NOT pressed: Turn LED OFF
		    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, GPIO_PIN_RESET);
		}
	}

	return 0;
}
