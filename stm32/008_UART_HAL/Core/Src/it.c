/*
 * it.c
 *
 *  Created on: Dec 6, 2025
 *      Author: Rahul B.
 */

#include "stm32f4xx_hal.h"
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
