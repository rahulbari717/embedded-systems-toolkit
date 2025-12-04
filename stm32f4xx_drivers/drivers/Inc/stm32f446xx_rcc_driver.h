/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Dec 4, 2025
 *      Author: Rahul B.
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

uint32_t RCC_GetPCLK1Value(void); // returns apb1 bus clk value 

uint32_t RCC_GetPCLK2Value(void); // returns abp2 bus clk value

uint32_t RCC_GetPLLOutputClock(void); 

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
