/**
 ******************************************************************************
 * @file           : led.c
 * @author         : Rahul B.
 * @brief          : LED driver implementation
 ******************************************************************************
 */

#include "led.h"

/* GPIO Register offsets */
#define GPIO_MODER_OFFSET    0x00
#define GPIO_ODR_OFFSET      0x14
#define RCC_AHB1ENR          0x40023830

/**
 * @brief Initialize LED pins as output
 */
void led_init(void) {
    uint32_t *pRCC_AHB1ENR = (uint32_t *)RCC_AHB1ENR;
    uint32_t *pGPIOA_MODER = (uint32_t *)(LED1_PORT_BASE + GPIO_MODER_OFFSET);
    uint32_t *pGPIOB_MODER = (uint32_t *)(LED2_PORT_BASE + GPIO_MODER_OFFSET);

    // Enable clock for GPIOA and GPIOB
    *pRCC_AHB1ENR |= (1 << 0);  // GPIOA clock
    *pRCC_AHB1ENR |= (1 << 1);  // GPIOB clock

    // Configure LED1 pin as output (Mode = 01)
    *pGPIOA_MODER &= ~(0x3 << (LED1_PIN * 2));  // Clear bits
    *pGPIOA_MODER |= (0x1 << (LED1_PIN * 2));   // Set as output

    // Configure LED2 pin as output (Mode = 01)
    *pGPIOB_MODER &= ~(0x3 << (LED2_PIN * 2));  // Clear bits
    *pGPIOB_MODER |= (0x1 << (LED2_PIN * 2));   // Set as output
}

/**
 * @brief Turn LED1 ON
 */
void led1_on(void) {
    uint32_t *pGPIOA_ODR = (uint32_t *)(LED1_PORT_BASE + GPIO_ODR_OFFSET);
    *pGPIOA_ODR |= (1 << LED1_PIN);
}

/**
 * @brief Turn LED1 OFF
 */
void led1_off(void) {
    uint32_t *pGPIOA_ODR = (uint32_t *)(LED1_PORT_BASE + GPIO_ODR_OFFSET);
    *pGPIOA_ODR &= ~(1 << LED1_PIN);
}

/**
 * @brief Toggle LED1
 */
void led1_toggle(void) {
    uint32_t *pGPIOA_ODR = (uint32_t *)(LED1_PORT_BASE + GPIO_ODR_OFFSET);
    *pGPIOA_ODR ^= (1 << LED1_PIN);
}

/**
 * @brief Turn LED2 ON
 */
void led2_on(void) {
    uint32_t *pGPIOB_ODR = (uint32_t *)(LED2_PORT_BASE + GPIO_ODR_OFFSET);
    *pGPIOB_ODR |= (1 << LED2_PIN);
}

/**
 * @brief Turn LED2 OFF
 */
void led2_off(void) {
    uint32_t *pGPIOB_ODR = (uint32_t *)(LED2_PORT_BASE + GPIO_ODR_OFFSET);
    *pGPIOB_ODR &= ~(1 << LED2_PIN);
}

/**
 * @brief Toggle LED2
 */
void led2_toggle(void) {
    uint32_t *pGPIOB_ODR = (uint32_t *)(LED2_PORT_BASE + GPIO_ODR_OFFSET);
    *pGPIOB_ODR ^= (1 << LED2_PIN);
}

/**
 * @brief Simple delay function
 * @param count: delay count
 */
void delay(uint32_t count) {
    for(uint32_t i = 0; i < count; i++) {
        __asm volatile("NOP");
    }
}
