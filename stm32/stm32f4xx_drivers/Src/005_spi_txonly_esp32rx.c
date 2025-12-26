/*
 * 005_spi_txonly_esp32rx.c
 *
 *  Created on: Dec 26, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx.h"
#include <string.h>

void delay(void){
	for(int i=0; i<500000; i++);
}

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

    // SCLK (PB13) - Input for slave
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI (PB15) - Input for slave
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO (PB14) - Output for slave
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // NSS (PB12) - Input for slave
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
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

    SPI2handle.pSPIx = SPI2;
    SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;         // Full duplex
    SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;   // Master mode
    SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;       // SCLK = APB1/8 (2MHz if APB1=16MHz)
    SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;                    // 8-bit data frame
    SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                   // Clock polarity low
    SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;                   // Clock phase low (first edge)
    SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                      // Hardware slave management enabled

    SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioButton);
}


int main(void)
{
    char user_data[] = "Hello Rahul, how are you ? We are testing stm32 and esp32 spi communication";

    GPIO_ButtonInit();
    // Initialize the GPIO pins to behave as SPI2 pins
    SPI2_GPIOInits();

    // Initialize the SPI2 peripheral parameters
    SPI2_Inits();

    SPI_SSOEConfig(SPI2, ENABLE);

    while(1){
    	// wait till button pressed
    	if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED){
			delay();
			// Enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			// first send lenght info
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2, &dataLen, 1);
			// Send data
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

			// Wait until SPI is not busy
			while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

			// Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, DISABLE);

		}
    }

    while(1);

    return 0;
}
