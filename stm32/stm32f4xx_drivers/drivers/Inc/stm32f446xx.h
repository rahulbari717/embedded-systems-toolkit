/*
 * stm32f446xx.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Rahul B.
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define __weak __attribute__((weak))

/************* Arm cortex Processor specific details ****************************************/
/*
 * NVIC ISERx (Interrupt Set-Enable Registers) base addresses
 */
#define NVIC_ISER0              ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1              ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2              ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3              ((volatile uint32_t*)0xE000E10C)

/*
 * NVIC ICERx (Interrupt Clear-Enable Registers) base addresses
 */
#define NVIC_ICER0              ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1              ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2              ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3              ((volatile uint32_t*)0xE000E18C)

/*
 * NVIC Priority Register base address
 */
#define NVIC_PR_BASE_ADDR       ((volatile uint32_t*)0xE000E400)

/*
 * Number of priority bits implemented in STM32F446RE
 */
#define NO_PR_BITS_IMPLEMENTED  4

#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI15			15

/*
 * base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR          0x08000000U     	/* Flash memory base address */
#define SRAM1_BASEADDR          0x20000000U     	/* SRAM1 base address (112KB) */
#define SRAM2_BASEADDR          0x2001C000U     	/* SRAM2 base address (16KB) */
#define SRAM                    SRAM1_BASEADDR  	/* Main SRAM base */
#define ROM_BASEADDR            0x1FFF0000U     	/* System memory (ROM) base address */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR         0x40000000U     	/* Base address of all peripherals */
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR 	/* APB1 bus base (Low speed peripherals, max 45MHz) */
#define APB2PERIPH_BASEADDR     0x40010000U     	/* APB2 bus base (High speed peripherals, max 90MHz) */

#define AHB1PERIPH_BASEADDR     0x40020000U     	/* AHB1 bus base (GPIO, DMA, RCC, Flash interface) */
#define AHB2PERIPH_BASEADDR     0x50000000U     	/* AHB2 bus base (USB OTG FS, DCMI, RNG) */
#define AHB3PERIPH_BASEADDR     0xA0000000U     	/* AHB3 bus base (FMC - Flexible Memory Controller) */

/*
 * Base addresses of peripherals hanging on AHB1 bus
 */
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)     /* GPIOA base address */
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)     /* GPIOB base address */
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)     /* GPIOC base address */
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)     /* GPIOD base address */
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)     /* GPIOE base address */
#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1400)     /* GPIOF base address */
#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1800)     /* GPIOG base address */
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)     /* GPIOH base address */
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)     /* RCC (Reset and Clock Control) base address */

/*
 * Base addresses of peripherals hanging on APB1 bus
 */
#define CAN1_BASEADDR           (APB1PERIPH_BASEADDR + 0x6400)     /* CAN1 base address */
#define CAN2_BASEADDR           (APB1PERIPH_BASEADDR + 0x6800)     /* CAN2 base address */
#define CEC_BASEADDR            (APB1PERIPH_BASEADDR + 0x6C00)     /* HDMI-CEC base address */
#define DAC_BASEADDR            (APB1PERIPH_BASEADDR + 0x7400)     /* DAC base address */
#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400)     /* I2C1 base address */
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800)     /* I2C2 base address */
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)     /* I2C3 base address */
#define IWDG_BASEADDR           (APB1PERIPH_BASEADDR + 0x3000)     /* IWDG base address */
#define PWR_BASEADDR            (APB1PERIPH_BASEADDR + 0x7000)     /* PWR base address */
#define RTC_BASEADDR            (APB1PERIPH_BASEADDR + 0x2800)     /* RTC & BKP Registers base address */
#define SPDIF_RX_BASEADDR       (APB1PERIPH_BASEADDR + 0x4000)     /* SPDIF-RX base address */
#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800)     /* SPI2/I2S2 base address */
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00)     /* SPI3/I2S3 base address */
#define TIM2_BASEADDR           (APB1PERIPH_BASEADDR + 0x0000)     /* TIM2 base address */
#define TIM3_BASEADDR           (APB1PERIPH_BASEADDR + 0x0400)     /* TIM3 base address */
#define TIM4_BASEADDR           (APB1PERIPH_BASEADDR + 0x0800)     /* TIM4 base address */
#define TIM5_BASEADDR           (APB1PERIPH_BASEADDR + 0x0C00)     /* TIM5 base address */
#define TIM6_BASEADDR           (APB1PERIPH_BASEADDR + 0x1000)     /* TIM6 base address */
#define TIM7_BASEADDR           (APB1PERIPH_BASEADDR + 0x1400)     /* TIM7 base address */
#define TIM12_BASEADDR          (APB1PERIPH_BASEADDR + 0x1800)     /* TIM12 base address */
#define TIM13_BASEADDR          (APB1PERIPH_BASEADDR + 0x1C00)     /* TIM13 base address */
#define TIM14_BASEADDR          (APB1PERIPH_BASEADDR + 0x2000)     /* TIM14 base address */
#define UART4_BASEADDR          (APB1PERIPH_BASEADDR + 0x4C00)     /* UART4 base address */
#define UART5_BASEADDR          (APB1PERIPH_BASEADDR + 0x5000)     /* UART5 base address */
#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400)     /* USART2 base address */
#define USART3_BASEADDR         (APB1PERIPH_BASEADDR + 0x4800)     /* USART3 base address */
#define WWDG_BASEADDR           (APB1PERIPH_BASEADDR + 0x2C00)     /* WWDG base address */

/*
 * Base addresses of peripherals hanging on APB2 bus
 */
#define ADC1_BASEADDR           (APB2PERIPH_BASEADDR + 0x2000)     /* ADC1 base address */
#define ADC2_BASEADDR           (APB2PERIPH_BASEADDR + 0x2100)     /* ADC2 base address */
#define ADC3_BASEADDR           (APB2PERIPH_BASEADDR + 0x2200)     /* ADC3 base address */
#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)     /* EXTI base address */
#define SAI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x5800)     /* SAI1 base address */
#define SAI2_BASEADDR           (APB2PERIPH_BASEADDR + 0x5C00)     /* SAI2 base address */
#define SDMMC_BASEADDR          (APB2PERIPH_BASEADDR + 0x2C00)     /* SDMMC base address */
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)     /* SPI1 base address */
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR + 0x3400)     /* SPI4 base address */
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)     /* SYSCFG base address */
#define TIM1_BASEADDR           (APB2PERIPH_BASEADDR + 0x0000)     /* TIM1 base address */
#define TIM8_BASEADDR           (APB2PERIPH_BASEADDR + 0x0400)     /* TIM8 base address */
#define TIM9_BASEADDR           (APB2PERIPH_BASEADDR + 0x4000)     /* TIM9 base address */
#define TIM10_BASEADDR          (APB2PERIPH_BASEADDR + 0x4400)     /* TIM10 base address */
#define TIM11_BASEADDR          (APB2PERIPH_BASEADDR + 0x4800)     /* TIM11 base address */
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000)     /* USART1 base address */
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400)     /* USART6 base address */

/**************************** peripheral register definition structure *************************************/

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct
{
    volatile uint32_t MODER;        /* GPIO port mode register,                    Address offset: 0x00 */
    volatile uint32_t OTYPER;       /* GPIO port output type register,             Address offset: 0x04 */
    volatile uint32_t OSPEEDR;      /* GPIO port output speed register,            Address offset: 0x08 */
    volatile uint32_t PUPDR;        /* GPIO port pull-up/pull-down register,       Address offset: 0x0C */
    volatile uint32_t IDR;          /* GPIO port input data register,              Address offset: 0x10 */
    volatile uint32_t ODR;          /* GPIO port output data register,             Address offset: 0x14 */
    volatile uint32_t BSRR;         /* GPIO port bit set/reset register,           Address offset: 0x18 */
    volatile uint32_t LCKR;         /* GPIO port configuration lock register,      Address offset: 0x1C */
    volatile uint32_t AFR[2];       /* GPIO alternate function registers,          Address offset: 0x20-0x24 */
                                    /* AFR[0] = AFRL (low register), AFR[1] = AFRH (high register) */
} GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
    volatile uint32_t CR;           /* RCC clock control register,                              Address offset: 0x00 */
    volatile uint32_t PLLCFGR;      /* RCC PLL configuration register,                          Address offset: 0x04 */
    volatile uint32_t CFGR;         /* RCC clock configuration register,                        Address offset: 0x08 */
    volatile uint32_t CIR;          /* RCC clock interrupt register,                            Address offset: 0x0C */
    volatile uint32_t AHB1RSTR;     /* RCC AHB1 peripheral reset register,                      Address offset: 0x10 */
    volatile uint32_t AHB2RSTR;     /* RCC AHB2 peripheral reset register,                      Address offset: 0x14 */
    volatile uint32_t AHB3RSTR;     /* RCC AHB3 peripheral reset register,                      Address offset: 0x18 */
    uint32_t RESERVED0;             /* Reserved,                                                Address offset: 0x1C */
    volatile uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register,                      Address offset: 0x20 */
    volatile uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register,                      Address offset: 0x24 */
    uint32_t RESERVED1[2];          /* Reserved,                                                Address offset: 0x28-0x2C */
    volatile uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock enable register,               Address offset: 0x30 */
    volatile uint32_t AHB2ENR;      /* RCC AHB2 peripheral clock enable register,               Address offset: 0x34 */
    volatile uint32_t AHB3ENR;      /* RCC AHB3 peripheral clock enable register,               Address offset: 0x38 */
    uint32_t RESERVED2;             /* Reserved,                                                Address offset: 0x3C */
    volatile uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register,               Address offset: 0x40 */
    volatile uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register,               Address offset: 0x44 */
    uint32_t RESERVED3[2];          /* Reserved,                                                Address offset: 0x48-0x4C */
    volatile uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode reg,  Address offset: 0x50 */
    volatile uint32_t AHB2LPENR;    /* RCC AHB2 peripheral clock enable in low power mode reg,  Address offset: 0x54 */
    volatile uint32_t AHB3LPENR;    /* RCC AHB3 peripheral clock enable in low power mode reg,  Address offset: 0x58 */
    uint32_t RESERVED4;             /* Reserved,                                                Address offset: 0x5C */
    volatile uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode reg,  Address offset: 0x60 */
    volatile uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enable in low power mode reg,  Address offset: 0x64 */
    uint32_t RESERVED5[2];          /* Reserved,                                                Address offset: 0x68-0x6C */
    volatile uint32_t BDCR;         /* RCC Backup domain control register,                      Address offset: 0x70 */
    volatile uint32_t CSR;          /* RCC clock control & status register,                     Address offset: 0x74 */
    uint32_t RESERVED6[2];          /* Reserved,                                                Address offset: 0x78-0x7C */
    volatile uint32_t SSCGR;        /* RCC spread spectrum clock generation register,           Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register,                       Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR;   /* RCC PLLSAI configuration register,                       Address offset: 0x88 */
    volatile uint32_t DCKCFGR;      /* RCC Dedicated Clocks configuration register,             Address offset: 0x8C */
    volatile uint32_t CKGATENR;     /* RCC Clocks Gated Enable Register,                        Address offset: 0x90 */
    volatile uint32_t DCKCFGR2;     /* RCC Dedicated Clocks configuration register 2,           Address offset: 0x94 */
} RCC_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
    volatile uint32_t CR1;          /* SPI control register 1,                     Address offset: 0x00 */
    volatile uint32_t CR2;          /* SPI control register 2,                     Address offset: 0x04 */
    volatile uint32_t SR;           /* SPI status register,                        Address offset: 0x08 */
    volatile uint32_t DR;           /* SPI data register,                          Address offset: 0x0C */
    volatile uint32_t CRCPR;        /* SPI CRC polynomial register,                Address offset: 0x10 */
    volatile uint32_t RXCRCR;       /* SPI RX CRC register,                        Address offset: 0x14 */
    volatile uint32_t TXCRCR;       /* SPI TX CRC register,                        Address offset: 0x18 */
    volatile uint32_t I2SCFGR;      /* SPI_I2S configuration register,             Address offset: 0x1C */
    volatile uint32_t I2SPR;        /* SPI_I2S prescaler register,                 Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
    volatile uint32_t CR1;          /* I2C Control register 1,                     Address offset: 0x00 */
    volatile uint32_t CR2;          /* I2C Control register 2,                     Address offset: 0x04 */
    volatile uint32_t OAR1;         /* I2C Own address register 1,                 Address offset: 0x08 */
    volatile uint32_t OAR2;         /* I2C Own address register 2,                 Address offset: 0x0C */
    volatile uint32_t DR;           /* I2C Data register,                          Address offset: 0x10 */
    volatile uint32_t SR1;          /* I2C Status register 1,                      Address offset: 0x14 */
    volatile uint32_t SR2;          /* I2C Status register 2,                      Address offset: 0x18 */
    volatile uint32_t CCR;          /* I2C Clock control register,                 Address offset: 0x1C */
    volatile uint32_t TRISE;        /* I2C TRISE register,                         Address offset: 0x20 */
    volatile uint32_t FLTR;         /* I2C FLTR register,                          Address offset: 0x24 */
} I2C_RegDef_t;

/*
 * Peripheral register definition structure for USART/UART
 */
typedef struct
{
    volatile uint32_t SR;           /* USART Status register,                      Address offset: 0x00 */
    volatile uint32_t DR;           /* USART Data register,                        Address offset: 0x04 */
    volatile uint32_t BRR;          /* USART Baud rate register,                   Address offset: 0x08 */
    volatile uint32_t CR1;          /* USART Control register 1,                   Address offset: 0x0C */
    volatile uint32_t CR2;          /* USART Control register 2,                   Address offset: 0x10 */
    volatile uint32_t CR3;          /* USART Control register 3,                   Address offset: 0x14 */
    volatile uint32_t GTPR;         /* USART Guard time and prescaler register,    Address offset: 0x18 */
} USART_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
    volatile uint32_t IMR;          /* Interrupt mask register,            Address offset: 0x00 */
    volatile uint32_t EMR;          /* Event mask register,                Address offset: 0x04 */
    volatile uint32_t RTSR;         /* Rising trigger selection register,  Address offset: 0x08 */
    volatile uint32_t FTSR;         /* Falling trigger selection register, Address offset: 0x0C */
    volatile uint32_t SWIER;        /* Software interrupt event register,  Address offset: 0x10 */
    volatile uint32_t PR;           /* Pending register,                   Address offset: 0x14 */
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    volatile uint32_t MEMRMP;       /* Memory remap register,              Address offset: 0x00 */
    volatile uint32_t PMC;          /* Peripheral mode configuration,      Address offset: 0x04 */
    volatile uint32_t EXTICR[4];    /* External interrupt configuration,   Address offset: 0x08-0x14 */
    uint32_t RESERVED1[2];          /* Reserved,                           Address offset: 0x18-0x1C */
    volatile uint32_t CMPCR;        /* Compensation cell control register, Address offset: 0x20 */
    uint32_t RESERVED2[2];          /* Reserved,                           Address offset: 0x24-0x28 */
    volatile uint32_t CFGR;         /* Configuration register,             Address offset: 0x2C */
} SYSCFG_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA                   ((GPIO_RegDef_t*)GPIOA_BASEADDR)    /* GPIOA peripheral definition */
#define GPIOB                   ((GPIO_RegDef_t*)GPIOB_BASEADDR)    /* GPIOB peripheral definition */
#define GPIOC                   ((GPIO_RegDef_t*)GPIOC_BASEADDR)    /* GPIOC peripheral definition */
#define GPIOD                   ((GPIO_RegDef_t*)GPIOD_BASEADDR)    /* GPIOD peripheral definition */
#define GPIOE                   ((GPIO_RegDef_t*)GPIOE_BASEADDR)    /* GPIOE peripheral definition */
#define GPIOF                   ((GPIO_RegDef_t*)GPIOF_BASEADDR)    /* GPIOF peripheral definition */
#define GPIOG                   ((GPIO_RegDef_t*)GPIOG_BASEADDR)    /* GPIOG peripheral definition */
#define GPIOH                   ((GPIO_RegDef_t*)GPIOH_BASEADDR)    /* GPIOH peripheral definition */

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define RCC                     ((RCC_RegDef_t*)RCC_BASEADDR)       /* RCC peripheral definition */
#define EXTI                    ((EXTI_RegDef_t*)EXTI_BASEADDR)        /* EXTI base address */
#define SYSCFG                  ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR) /* SYSCFG peripheral definition */

#define SPI1                    ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                    ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                    ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                    ((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * Peripheral definitions (Peripheral base addresses typecasted to I2C_RegDef_t)
 */
#define I2C1                    ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                    ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                    ((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * Peripheral definitions (Peripheral base addresses typecasted to USART_RegDef_t)
 */
#define USART1                  ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2                  ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3                  ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4                   ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5                   ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6                  ((USART_RegDef_t*)USART6_BASEADDR)

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

/*
 * I2C Peripheral Reset Macros
 */
#define I2C1_REG_RESET()        do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()        do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()        do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)


/*
 * Macros to reset SPI peripherals
 */

// SPI1 Reset (APB2 Bus)
#define SPI1_REG_RESET()        do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()        do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()        do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()        do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * Macros to reset USART peripherals
 */
#define USART1_REG_RESET()      do{ (RCC->APB2RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4));  }while(0)
#define USART2_REG_RESET()      do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()      do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()       do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()       do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()      do{ (RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5));  }while(0)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << 0))          /* Enable GPIOA clock */
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << 1))          /* Enable GPIOB clock */
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1 << 2))          /* Enable GPIOC clock */
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1 << 3))          /* Enable GPIOD clock */
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1 << 4))          /* Enable GPIOE clock */
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= (1 << 5))          /* Enable GPIOF clock */
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= (1 << 6))          /* Enable GPIOG clock */
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1 << 7))          /* Enable GPIOH clock */

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))         /* Enable I2C1 clock */
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))         /* Enable I2C2 clock */
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << 23))         /* Enable I2C3 clock */

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))         /* Enable SPI1 clock */
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))         /* Enable SPI2 clock */
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))         /* Enable SPI3 clock */
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1 << 13))         /* Enable SPI4 clock */

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 4))          /* Enable USART1 clock */
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))         /* Enable USART2 clock */
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1 << 18))         /* Enable USART3 clock */
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1 << 5))          /* Enable USART6 clock */

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART4_PCLK_EN()         (RCC->APB1ENR |= (1 << 19))         /* Enable UART4 clock */
#define UART5_PCLK_EN()         (RCC->APB1ENR |= (1 << 20))         /* Enable UART5 clock */

/*
 * Clock Enable Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))         /* Enable SYSCFG clock */

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 0))         /* Disable GPIOA clock */
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 1))         /* Disable GPIOB clock */
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 2))         /* Disable GPIOC clock */
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 3))         /* Disable GPIOD clock */
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 4))         /* Disable GPIOE clock */
#define GPIOF_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 5))         /* Disable GPIOF clock */
#define GPIOG_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 6))         /* Disable GPIOG clock */
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 7))         /* Disable GPIOH clock */

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))        /* Disable I2C1 clock */
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))        /* Disable I2C2 clock */
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 23))        /* Disable I2C3 clock */

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))        /* Disable SPI1 clock */
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))        /* Disable SPI2 clock */
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))        /* Disable SPI3 clock */
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 13))        /* Disable SPI4 clock */

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))         /* Disable USART1 clock */
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))        /* Disable USART2 clock */
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 18))        /* Disable USART3 clock */
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 5))         /* Disable USART6 clock */

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 19))        /* Disable UART4 clock */
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 20))        /* Disable UART5 clock */

/*
 * Clock Disable Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))        /* Disable SYSCFG clock */

/*
 * Macro to convert GPIO base address to port code (0-7)
 */
#define GPIO_BASEADDR_TO_CODE(x)  ((x == GPIOA)?0:\
                                   (x == GPIOB)?1:\
                                   (x == GPIOC)?2:\
                                   (x == GPIOD)?3:\
                                   (x == GPIOE)?4:\
                                   (x == GPIOF)?5:\
                                   (x == GPIOG)?6:\
                                   (x == GPIOH)?7:0)

/* Core Interrupts */
#define WWDG_IRQn                   0
#define PVD_IRQn                    1
#define TAMP_STAMP_IRQn             2
#define RTC_WKUP_IRQn               3
#define FLASH_IRQn                  4
#define RCC_IRQn                    5

#define EXTI0_IRQn                  6
#define EXTI1_IRQn                  7
#define EXTI2_IRQn                  8
#define EXTI3_IRQn                  9
#define EXTI4_IRQn                  10

#define DMA1_Stream0_IRQn           11
#define DMA1_Stream1_IRQn           12
#define DMA1_Stream2_IRQn           13
#define DMA1_Stream3_IRQn           14
#define DMA1_Stream4_IRQn           15
#define DMA1_Stream5_IRQn           16
#define DMA1_Stream6_IRQn           17

#define ADC_IRQn                    18
#define CAN1_TX_IRQn                19
#define CAN1_RX0_IRQn               20
#define CAN1_RX1_IRQn               21
#define CAN1_SCE_IRQn               22
#define EXTI9_5_IRQn                23
#define TIM1_BRK_TIM9_IRQn          24
#define TIM1_UP_TIM10_IRQn          25
#define TIM1_TRG_COM_TIM11_IRQn     26
#define TIM1_CC_IRQn                27
#define TIM2_IRQn                   28
#define TIM3_IRQn                   29
#define TIM4_IRQn                   30

#define I2C1_EV_IRQn                31
#define I2C1_ER_IRQn                32
#define I2C2_EV_IRQn                33
#define I2C2_ER_IRQn                34

#define SPI1_IRQn                   35
#define SPI2_IRQn                   36

#define USART1_IRQn                 37
#define USART2_IRQn                 38
#define USART3_IRQn                 39

#define EXTI15_10_IRQn              40
#define RTC_Alarm_IRQn              41
#define OTG_FS_WKUP_IRQn            42

#define TIM8_BRK_TIM12_IRQn         43
#define TIM8_UP_TIM13_IRQn          44
#define TIM8_TRG_COM_TIM14_IRQn     45
#define TIM8_CC_IRQn                46

#define DMA1_Stream7_IRQn           47
#define FMC_IRQn                    48
#define SDIO_IRQn                   49
#define TIM5_IRQn                   50
#define SPI3_IRQn                   51
#define UART4_IRQn                  52
#define UART5_IRQn                  53
#define TIM6_DAC_IRQn               54
#define TIM7_IRQn                   55

#define DMA2_Stream0_IRQn           56
#define DMA2_Stream1_IRQn           57
#define DMA2_Stream2_IRQn           58
#define DMA2_Stream3_IRQn           59
#define DMA2_Stream4_IRQn           60

#define CAN2_TX_IRQn                63
#define CAN2_RX0_IRQn               64
#define CAN2_RX1_IRQn               65
#define CAN2_SCE_IRQn               66

#define OTG_FS_IRQn                 67
#define DMA2_Stream5_IRQn           68
#define DMA2_Stream6_IRQn           69
#define DMA2_Stream7_IRQn           70
#define USART6_IRQn                 71

#define I2C3_EV_IRQn                72
#define I2C3_ER_IRQn                73
#define FPU_IRQn                    81
#define SPI4_IRQn                   84

/*
 * Generic macros
 */
#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET

#define FLAG_RESET				RESET
#define FLAG_SET				SET

#define LOW 			        DISABLE
#define BTN_PRESSED 	        LOW
#define HSI_CLK_VALUE           (16000000)
#define HSE_CLK_VALUE           (8000000)

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
