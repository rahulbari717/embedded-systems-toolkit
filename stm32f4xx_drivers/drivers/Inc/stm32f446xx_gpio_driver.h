/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Rahul B.
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN            0       /* Input mode (reset state) */
#define GPIO_MODE_OUT           1       /* General purpose output mode */
#define GPIO_MODE_ALTFN         2       /* Alternate function mode */
#define GPIO_MODE_ANALOG        3       /* Analog mode */
#define GPIO_MODE_IT_FT         4       /* Input with falling edge trigger interrupt */
#define GPIO_MODE_IT_RT         5       /* Input with rising edge trigger interrupt */
#define GPIO_MODE_IT_RFT        6       /* Input with rising/falling edge trigger interrupt */

/*
 * @GPIO_PIN_OPTYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP         0       /* Output push-pull (reset state) */
#define GPIO_OP_TYPE_OD         1       /* Output open-drain */

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW          0       /* Low speed */
#define GPIO_SPEED_MEDIUM       1       /* Medium speed */
#define GPIO_SPEED_FAST         2       /* Fast speed */
#define GPIO_SPEED_HIGH         3       /* High speed */

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up/pull-down configuration
 */
#define GPIO_NO_PUPD            0       /* No pull-up, no pull-down */
#define GPIO_PIN_PU             1       /* Pull-up */
#define GPIO_PIN_PD             2       /* Pull-down */

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

/*
 * @GPIO_PIN_ALT_FUN
 * GPIO pin alternate function modes
 */
#define GPIO_AF0                0       /* Alternate function 0 */
#define GPIO_AF1                1       /* Alternate function 1 */
#define GPIO_AF2                2       /* Alternate function 2 */
#define GPIO_AF3                3       /* Alternate function 3 */
#define GPIO_AF4                4       /* Alternate function 4 */
#define GPIO_AF5                5       /* Alternate function 5 */
#define GPIO_AF6                6       /* Alternate function 6 */
#define GPIO_AF7                7       /* Alternate function 7 */
#define GPIO_AF8                8       /* Alternate function 8 */
#define GPIO_AF9                9       /* Alternate function 9 */
#define GPIO_AF10               10      /* Alternate function 10 */
#define GPIO_AF11               11      /* Alternate function 11 */
#define GPIO_AF12               12      /* Alternate function 12 */
#define GPIO_AF13               13      /* Alternate function 13 */
#define GPIO_AF14               14      /* Alternate function 14 */
#define GPIO_AF15               15      /* Alternate function 15 */

/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;         /* Possible values from @GPIO_PIN_NUMBERS */
    uint8_t GPIO_PinMode;           /* Possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;          /* Possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl;    /* Possible values from @GPIO_PIN_PUPD */
    uint8_t GPIO_PinOPType;         /* Possible values from @GPIO_PIN_OPTYPES */
    uint8_t GPIO_PinAltFunMode;     /* Possible values from @GPIO_PIN_ALT_FUN */
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;          /* Holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig; /* Holds GPIO pin configuration settings */
} GPIO_Handle_t;

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/******************************************************************************************
 *                              APIs supported by this driver
 *         For more information about the APIs, check the function definitions
 ******************************************************************************************/

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
