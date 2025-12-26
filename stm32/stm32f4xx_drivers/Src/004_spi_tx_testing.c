/*
 * 004_SPI_Testing.c
 *
 *  Created on: Nov 26, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx.h"
#include <string.h>

/*********************************************************************
 * @fn              - SPI_GPIOInits
 *
 * @brief           - This function initializes the GPIO pins to be used as SPI2 pins
 *
 * @param[in]       - none
 *
 * @return          - none
 *
 * @Note            - SPI2 pins configuration:
 *                    PB12 --> SPI2_NSS
 *                    PB13 --> SPI2_SCLK
 *                    PB14 --> SPI2_MISO
 *                    PB15 --> SPI2_MOSI
 *                    Alternate function mode: AF5
 */
void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    memset(&SPIPins, 0, sizeof(SPIPins));

    // Configure common settings for all SPI pins
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF5_SPI;  // AF5 for SPI2
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK (PB13)
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI (PB15)
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO (PB14)
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // NSS (PB12)
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;

}

/*********************************************************************
 * @fn              - SPI2_Inits
 *
 * @brief           - This function initializes the SPI2 peripheral
 *
 * @param[in]       - none
 *
 * @return          - none
 *
 * @Note            - Configures SPI2 in master mode with standard settings
 */
void SPI2_Inits(void)
{
    SPI_Handle_t SPI2handle;
    memset(&SPI2handle, 0, sizeof(SPI2handle));

    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;         // Full duplex
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;   // Master mode
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;       // SCLK = APB1/2 (8MHz if APB1=16MHz)
    SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;                    // 8-bit data frame
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                   // Clock polarity low
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;                   // Clock phase low (first edge)
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;                      // Software slave management enabled

    SPI_Init(&SPI2handle);
}

int main(void)
{
    char user_data[] = "Hello Rahul\n";

    // Initialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInits();

    // Initialize the SPI2 peripheral parameters
    SPI2_Inits();

    // This makes NSS signal internally high and avoids MODF error
    SPI_SSIConfig(SPI2, ENABLE);

    // Enable the SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    // Send data
    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

    // Wait until SPI is not busy
    while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

    // Disable the SPI2 peripheral
    SPI_PeripheralControl(SPI2, DISABLE);

    while(1);

    return 0;
}





