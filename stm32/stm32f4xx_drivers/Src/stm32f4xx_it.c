/*
 * stm32f4xx_it.c
 *
 *  Created on: Dec 13, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx.h"

/**
 * @brief Clears the EXTI Pending Register (PR) bit for the given pin number.
 *
 * @param[in] PinNumber: The pin number (0 to 15) corresponding to the EXTI line.
 *
 * @return - none
 *
 * @Note - Writing 1 to the PR bit clears the pending interrupt.
 */
void GPIO_ClearEXTIPendingBit(uint8_t PinNumber)
{
    // The PR register is cleared by writing '1' to the pending bit.
    // First, check if the interrupt is pending to avoid unnecessary writes,
    // then clear it.
    if(EXTI->PR & (1 << PinNumber))
    {
        // Clear the pending bit by writing '1'
        EXTI->PR |= (1 << PinNumber);
    }
}


void EXTI15_10_IRQHandler(void) {
    
    
    
    // ... rest of your DMA-triggering logic ...
}

// IRQ Handler for EXTI15_10 (handles PC13)
void EXTI15_10_IRQHandler(void){
	delay(); // Simple debounce
	GPIO_IRQHandling(GPIO_PIN_NO_13); // Clear the pending bit
    // Clear the EXTI pending bit for PC13 (Pin 13)
    GPIO_ClearEXTIPendingBit(GPIO_PIN_NO_13); 
	// GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5); // Toggle LED
}

