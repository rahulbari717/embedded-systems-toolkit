/*
 * 006_i2c_master_tx.c
 *  Created on: Dec 3, 2025
 *  Author: Rahul B.
 *
 *  PB6 -> I2C1_SCL  (AF4)
 *  PB7 -> I2C1_SDA  (AF4)
 */

#include "stm32f446xx.h"
#include <string.h>

I2C_Handle_t I2C1Handle;

void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

/*********************************************************************
 * @fn              - I2C1_GPIOInits
 *
 * @brief           - Initialize PB6, PB7 for I2C1 AF4
 *********************************************************************/
void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF4_I2C;       // AF4 = I2C
    I2CPins.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_OD;    // Open Drain
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;        // Pull-up for I2C lines
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;

    // SCL -> PB6
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    // SDA -> PB7
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);
}

/*********************************************************************
 * @fn              - I2C1_Inits
 *
 * @brief           - Initialize I2C1 peripheral as master
 *********************************************************************/
void I2C1_Inits(void)
{
    

    I2C1Handle.pI2Cx = I2C1;

    I2C1Handle.I2C_Config.I2C_AckControl   = I2C_ACK_ENABLE;
    I2C1Handle.I2C_Config.I2C_DeviceAddress= 0x61;       // own address (not used for master)
    I2C1Handle.I2C_Config.I2C_SCLSpeed     = I2C_SCL_SPEED_SM;   // 100 kHz
    I2C1Handle.I2C_Config.I2C_FMDutyCycle  = I2C_FM_DUTY_2;

    I2C_Init(&I2C1Handle);
}

int main(void)
{
    char data[] = "Hello World";

    // 1) Init GPIO for I2C
    I2C1_GPIOInits();

    // 2) Init I2C1 peripheral
    I2C1_Inits();

    // 3) Enable the I2C peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // 4) Send data to slave (example slave address = 0x68)
//    I2C_MasterSendData(I2C1, (uint8_t*)data, strlen(data), 0x68, I2C_DISABLE_SR);
//    I2C_MasterSendData(&I2C1Handle, (uint8_t*)data, strlen(data), 0x68, I2C_DISABLE_SR);

    while(1)
        {
            // Just toggle START and STOP to see basic signals
            I2C1->CR1 |= (1 << I2C_CR1_START);
            for(uint32_t i=0; i<100000; i++);  // Delay

            I2C1->CR1 |= (1 << I2C_CR1_STOP);
            for(uint32_t i=0; i<100000; i++);  // Delay
        }


    while(1);
    return 0;
}
