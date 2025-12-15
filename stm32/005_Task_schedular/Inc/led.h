/**
 ******************************************************************************
 * @file           : led.h
 * @author         : Rahul B.
 * @brief          : LED driver header file
 ******************************************************************************
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

/* LED GPIO Configuration */
#define LED1_PORT_BASE    0x40020000  // GPIOA base address
#define LED2_PORT_BASE    0x40020400  // GPIOB base address

#define LED1_PIN          5   // PA5
#define LED2_PIN          7   // PB7

/* Function prototypes */
void led_init(void);
void led1_on(void);
void led1_off(void);
void led1_toggle(void);
void led2_on(void);
void led2_off(void);
void led2_toggle(void);
void delay(uint32_t count);

#endif /* LED_H_ */
