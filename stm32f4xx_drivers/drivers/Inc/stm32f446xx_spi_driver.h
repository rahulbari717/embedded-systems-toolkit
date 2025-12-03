/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Rahul B.
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
    uint8_t SPI_DeviceMode;         /* Possible values from @SPI_DeviceMode */
    uint8_t SPI_BusConfig;          /* Possible values from @SPI_BusConfig */
    uint8_t SPI_SclkSpeed;          /* Possible values from @SPI_SclkSpeed */
    uint8_t SPI_DFF;                /* Possible values from @SPI_DFF */
    uint8_t SPI_CPOL;               /* Possible values from @SPI_CPOL */
    uint8_t SPI_CPHA;               /* Possible values from @SPI_CPHA */
    uint8_t SPI_SSM;                /* Possible values from @SPI_SSM */
} SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;            /* Base address of SPIx peripheral */
    SPI_Config_t SPIConfig;         /* SPIx configuration settings */
    uint8_t *pTxBuffer;             /* Application Tx buffer address */
    uint8_t *pRxBuffer;             /* Application Rx buffer address */
    uint32_t TxLen;                 /* Tx transfer length */
    uint32_t RxLen;                 /* Rx transfer length */
    uint8_t TxState;                /* Tx state (READY, BUSY_IN_TX, etc.) */
    uint8_t RxState;                /* Rx state (READY, BUSY_IN_RX, etc.) */
} SPI_Handle_t;

/*
 * Possible SPI Application states
 */
#define SPI_READY           0
#define SPI_BUSY_IN_RX      1
#define SPI_BUSY_IN_TX      2

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER      1
#define SPI_DEVICE_MODE_SLAVE       0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD           1       /* Full duplex */
#define SPI_BUS_CONFIG_HD           2       /* Half duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RX   3       /* Simplex RX only */

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2         0       /* fPCLK/2 */
#define SPI_SCLK_SPEED_DIV4         1       /* fPCLK/4 */
#define SPI_SCLK_SPEED_DIV8         2       /* fPCLK/8 */
#define SPI_SCLK_SPEED_DIV16        3       /* fPCLK/16 */
#define SPI_SCLK_SPEED_DIV32        4       /* fPCLK/32 */
#define SPI_SCLK_SPEED_DIV64        5       /* fPCLK/64 */
#define SPI_SCLK_SPEED_DIV128       6       /* fPCLK/128 */
#define SPI_SCLK_SPEED_DIV256       7       /* fPCLK/256 */

/*
 * @SPI_DFF (Data Frame Format)
 */
#define SPI_DFF_8BIT                0
#define SPI_DFF_16BIT               1

/*
 * @SPI_CPOL (Clock Polarity)
 */
#define SPI_CPOL_LOW                0       /* Clock idle state is low */
#define SPI_CPOL_HIGH               1       /* Clock idle state is high */

/*
 * @SPI_CPHA (Clock Phase)
 */
#define SPI_CPHA_LOW                0       /* Data captured on first clock edge */
#define SPI_CPHA_HIGH               1       /* Data captured on second clock edge */

/*
 * @SPI_SSM (Software Slave Management)
 */
#define SPI_SSM_DI                  0       /* Hardware slave management */
#define SPI_SSM_EN                  1       /* Software slave management enabled */

/*
 * SPI related status flags definitions (bit positions in SR register)
 */
#define SPI_TXE_FLAG        (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG       (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG       (1 << SPI_SR_BSY)


/*
 * Bit position definitions of SPI peripheral (CR1 register)
 */
#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR        2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_LSBFIRST    7
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15

/*
 * Bit position definitions of SPI peripheral (CR2 register)
 */
#define SPI_CR2_RXDMAEN     0
#define SPI_CR2_TXDMAEN     1
#define SPI_CR2_SSOE        2
#define SPI_CR2_FRF         4
#define SPI_CR2_ERRIE       5
#define SPI_CR2_RXNEIE      6
#define SPI_CR2_TXEIE       7

/*
 * Bit position definitions of SPI peripheral (SR register)
 */
#define SPI_SR_RXNE         0
#define SPI_SR_TXE          1
#define SPI_SR_CHSIDE       2
#define SPI_SR_UDR          3
#define SPI_SR_CRCERR       4
#define SPI_SR_MODF         5
#define SPI_SR_OVR          6
#define SPI_SR_BSY          7
#define SPI_SR_FRE          8

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_ERR   3
#define SPI_EVENT_CRC_ERR   4

/******************************************************************************************
 *                           APIs supported by this driver
 *             For more information about the APIs, check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data Send and Receive with Interrupt
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
