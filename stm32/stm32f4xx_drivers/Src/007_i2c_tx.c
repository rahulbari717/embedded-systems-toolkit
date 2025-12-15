/*
 * 006_i2c_master_tx.c
 *  Created on: __Dec__ 3, 2025
 *  Author: __Rahul__ B.
 *
 *  PB6 -> I2C1_SCL  (AF4)
 *  PB7 -> I2C1_SDA  (AF4)
 */

#include "stm32f446xx.h"
#include <string.h>

I2C_Handle_t I2C1Handle;

/*********************************************************************
 * @__fn__              - I2C1_GPIOInits
 *
 * @brief           - Initialize PB6, PB7 for I2C1 AF4
 *********************************************************************/
void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    // Enable GPIOB Clock
    GPIOB_PCLK_EN();

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;

    // SCL -> PB6
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    // SDA -> PB7
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);
}

/*********************************************************************
 * @__fn__              - I2C1___Inits__
 *
 * @brief           - Initialize I2C1 peripheral as master
 *********************************************************************/
void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_AckControl   = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress= 0x61;
    I2C1Handle.I2C_Config.I2C_SCLSpeed     = I2C_SCL_SPEED_SM;
    I2C1Handle.I2C_Config.I2C_FMDutyCycle  = I2C_FM_DUTY_2;

    I2C_Init(&I2C1Handle);
}

int main(void)
{
    char data[] = "Hello World";
    uint8_t i;

    // 1) Init GPIO
    I2C1_GPIOInits();

    // 2) Init I2C1
    I2C1_Inits();

    // 3) Enable I2C
    I2C_PeripheralControl(I2C1, ENABLE);

    // 4) SEND DATA - ONE TIME ONLY

    // Generate START
    I2C1->CR1 |= (1 << 8);  // Set START bit

    // Wait for SB flag (START generated)
    while(!(I2C1->SR1 & (1 << 0)));

    // Send slave address
    I2C1->DR = (0x68 << 1);  // Address with write bit

    // Small delay
    for(volatile uint32_t d=0; d<100000; d++);

    // Read SR1 and SR2 to clear ADDR flag (even though NAK)
    volatile uint32_t temp;
    temp = I2C1->SR1;
    temp = I2C1->SR2;

    // NOW SEND EACH CHARACTER
    for(i = 0; i < 11; i++)  // "Hello World" = 11 chars
    {
        // Wait for TXE (transmit buffer empty)
        while(!(I2C1->SR1 & (1 << 7)));

        // Send the character
        I2C1->DR = data[i];
    }

    // Wait a bit for last byte
    for(volatile uint32_t d=0; d<100000; d++);

    // Generate STOP
    I2C1->CR1 |= (1 << 9);  // Set STOP bit

    // DONE - Check Logic Analyzer now!
    while(1);

    return 0;
}
