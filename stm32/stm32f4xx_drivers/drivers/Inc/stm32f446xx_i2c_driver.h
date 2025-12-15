/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 3rd Dec, 2025
 *      Author: Rahul B.
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;          /* Possible values from @I2C_SCL_SPEED */
    uint8_t  I2C_DeviceAddress;     /* Device own address (7-bit or 10-bit) */
    uint8_t  I2C_AckControl;        /* ACK control: ENABLE or DISABLE */
    uint8_t  I2C_FMDutyCycle;       /* Fast mode duty cycle: 2 or 16/9 */
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
    I2C_RegDef_t    *pI2Cx;         /* Base address of I2Cx peripheral */
    I2C_Config_t    I2C_Config;     /* I2C configuration settings */
    uint8_t         *pTxBuffer;     /* Pointer to Tx buffer */
    uint8_t         *pRxBuffer;     /* Pointer to Rx buffer */
    uint32_t        TxLen;          /* Tx data length */
    uint32_t        RxLen;          /* Rx data length */
    uint8_t         TxRxState;      /* Communication state */
    uint8_t         DevAddr;        /* Slave/Device address */
    uint32_t        RxSize;         /* Rx size */
    uint8_t         Sr;             /* Repeated start value */

    uint8_t TxBusy;            
    uint8_t RxBusy;            

} I2C_Handle_t;

/*
 * @I2C_SCL_SPEED
 * I2C Speed modes
 */
#define I2C_SCL_SPEED_SM                100000      /* Standard Mode: 100kHz */
#define I2C_SCL_SPEED_FM2K              200000      /* Fast Mode: 200kHz */
#define I2C_SCL_SPEED_FM4K              400000      /* Fast Mode: 400kHz */

/*
 * @I2C_ACK_CONTROL
 * I2C ACK control
 */
#define I2C_ACK_ENABLE                  1
#define I2C_ACK_DISABLE                 0

/*
 * @I2C_FM_DUTY_CYCLE
 * I2C FM Duty Cycle
 */
#define I2C_FM_DUTY_2                   0
#define I2C_FM_DUTY_16_9                1

/*
 * I2C Application States
 */
#define I2C_READY                       0
#define I2C_BUSY_IN_RX                  1
#define I2C_BUSY_IN_TX                  2

/*
 * I2C Application Events
 */
#define I2C_EV_TX_CMPLT                 0
#define I2C_EV_RX_CMPLT                 1
#define I2C_EV_STOP                     2
#define I2C_ERROR_BERR                  3
#define I2C_ERROR_ARLO                  4
#define I2C_ERROR_AF                    5
#define I2C_ERROR_OVR                   6
#define I2C_ERROR_TIMEOUT               7
#define I2C_EV_DATA_REQ                 8
#define I2C_EV_DATA_RCV                 9

/*
 * I2C Repeated Start
 */
#define I2C_DISABLE_SR                  0
#define I2C_ENABLE_SR                   1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE                    (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE                   (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB                     (1 << I2C_SR1_SB)
#define I2C_FLAG_OVR                    (1 << I2C_SR1_OVR)
#define I2C_FLAG_AF                     (1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO                   (1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR                   (1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF                  (1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10                  (1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF                    (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR                   (1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT                (1 << I2C_SR1_TIMEOUT)

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE                      0
#define I2C_CR1_SMBUS                   1
#define I2C_CR1_SMBTYPE                 3
#define I2C_CR1_ENARP                   4
#define I2C_CR1_ENPEC                   5
#define I2C_CR1_ENGC                    6
#define I2C_CR1_NOSTRETCH               7
#define I2C_CR1_START                   8
#define I2C_CR1_STOP                    9
#define I2C_CR1_ACK                     10
#define I2C_CR1_POS                     11
#define I2C_CR1_PEC                     12
#define I2C_CR1_ALERT                   13
#define I2C_CR1_SWRST                   15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ                    0
#define I2C_CR2_ITERREN                 8
#define I2C_CR2_ITEVTEN                 9
#define I2C_CR2_ITBUFEN                 10
#define I2C_CR2_DMAEN                   11
#define I2C_CR2_LAST                    12

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0                   0
#define I2C_OAR1_ADD1                   1
#define I2C_OAR1_ADD7_1                 1
#define I2C_OAR1_ADD9_8                 8
#define I2C_OAR1_ADDMODE                15

/*
 * Bit position definitions I2C_OAR2
 */
#define I2C_OAR2_ENDUAL                 0
#define I2C_OAR2_ADD2                   1

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB                      0
#define I2C_SR1_ADDR                    1
#define I2C_SR1_BTF                     2
#define I2C_SR1_ADD10                   3
#define I2C_SR1_STOPF                   4
#define I2C_SR1_RXNE                    6
#define I2C_SR1_TXE                     7
#define I2C_SR1_BERR                    8
#define I2C_SR1_ARLO                    9
#define I2C_SR1_AF                      10
#define I2C_SR1_OVR                     11
#define I2C_SR1_PECERR                  12
#define I2C_SR1_TIMEOUT                 14
#define I2C_SR1_SMBALERT                15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL                     0
#define I2C_SR2_BUSY                    1
#define I2C_SR2_TRA                     2
#define I2C_SR2_GENCALL                 4
#define I2C_SR2_SMBDEFAULT              5
#define I2C_SR2_SMBHOST                 6
#define I2C_SR2_DUALF                   7
#define I2C_SR2_PEC                     8

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR                     0
#define I2C_CCR_DUTY                    14
#define I2C_CCR_FS                      15

/*
 * Bit position definitions I2C_TRISE
 */
#define I2C_TRISE_TRISE                 0

/*
 * Bit position definitions I2C_FLTR
 */
#define I2C_FLTR_DNF                    0
#define I2C_FLTR_ANOFF                  4

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

/******************************************************************************************
 *                           APIs supported by this driver
 *             For more information about the APIs, check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data Send and Receive with Interrupt
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */