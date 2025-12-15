/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Nov 25, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx_spi_driver.h"

/*
* Helper function prototypes (private to driver)
*/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
static void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
static void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

/*********************************************************************
 * @fn              - SPI_PeriClockControl
 *
 * @brief           - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]       - Base address of the SPI peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if(pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if(pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

/*********************************************************************
 * @fn              - SPI_Init
 *
 * @brief           - This function initializes the given SPI peripheral
 *
 * @param[in]       - Pointer to SPI handle structure
 *
 * @return          - none
 *
 * @Note            - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // Enable the peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // Configure the SPI_CR1 register
    uint32_t tempreg = 0;

    // 1. Configure the device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // BIDI mode should be cleared (2-line unidirectional)
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // BIDI mode should be set (1-line bidirectional)
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
    {
        // BIDI mode should be cleared
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        // RXONLY bit must be set
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure the SPI serial clock speed (baud rate)
    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. Configure the DFF (Data Frame Format)
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. Configure the CPOL (Clock Polarity)
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6. Configure the CPHA (Clock Phase)
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 7. Configure the SSM (Software Slave Management)
    tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn              - SPI_DeInit
 *
 * @brief           - This function de-initializes the given SPI peripheral
 *
 * @param[in]       - Base address of the SPI peripheral
 *
 * @return          - none
 *
 * @Note            - This function resets all the registers of the SPI peripheral
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if(pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if(pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if(pSPIx == SPI4)
    {
        SPI4_REG_RESET();
    }
}

/*********************************************************************
 * @fn              - SPI_GetFlagStatus
 *
 * @brief           - This function returns the status of the given flag in SPI status register
 *
 * @param[in]       - Base address of the SPI peripheral
 * @param[in]       - Flag name (bit position in SR register)
 *
 * @return          - FLAG_SET or FLAG_RESET
 *
 * @Note            - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}
/*********************************************************************
 * @fn              - SPI_SendData
 *
 * @brief           - This function sends data over SPI peripheral (blocking/polling mode)
 *
 * @param[in]       - Base address of the SPI peripheral
 * @param[in]       - Pointer to Tx buffer
 * @param[in]       - Length of data to be transmitted (in bytes)
 *
 * @return          - none
 *
 * @Note            - This is a blocking call (polling based)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // 1. Wait until TXE is set (Tx buffer is empty)
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            // Load the data into the DR (Data Register)
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        }
        else
        {
            // 8-bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}

/*********************************************************************
 * @fn              - SPI_ReceiveData
 *
 * @brief           - This function receives data over SPI peripheral (blocking/polling mode)
 *
 * @param[in]       - Base address of the SPI peripheral
 * @param[in]       - Pointer to Rx buffer
 * @param[in]       - Length of data to be received (in bytes)
 *
 * @return          - none
 *
 * @Note            - This is a blocking call (polling based)
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // 1. Wait until RXNE is set (Rx buffer is not empty)
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            // Load the data from DR to Rx buffer
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        }
        else
        {
            // 8-bit DFF
            *pRxBuffer = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

/*********************************************************************
 * @fn              - SPI_PeripheralControl
 *
 * @brief           - This function enables or disables the SPI peripheral
 *
 * @param[in]       - Base address of the SPI peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - SPE bit in CR1 register controls the peripheral enable/disable
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/*********************************************************************
 * @fn              - SPI_SSIConfig
 *
 * @brief           - This function configures the SSI (Internal Slave Select) bit
 *
 * @param[in]       - Base address of the SPI peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - This bit has an effect only when SSM bit is set
 *                    The value of this bit is forced onto the NSS pin and the I/O value of the NSS pin is ignored
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

///*********************************************************************
// * @fn              - SPI_SSOEConfig
// *
// * @brief           - This function configures the SSOE (SS Output Enable) bit
// *
// * @param[in]       - Base address of the SPI peripheral
// * @param[in]       - ENABLE or DISABLE macros
// *
// * @return          - none
// *
// * @Note            - When SSOE is set, NSS pin is automatically managed by hardware
// *                    NSS output is enabled in master mode and disabled in slave mode
// */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

///*********************************************************************
// * @fn              - SPI_IRQInterruptConfig
// *
// * @brief           - This function configures the IRQ (enables or disables)
// *
// * @param[in]       - IRQ number
// * @param[in]       - ENABLE or DISABLE macros
// *
// * @return          - none
// *
// * @Note            - Configures the NVIC registers
// */
//void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
//{
//    if(EnorDi == ENABLE)
//    {
//        if(IRQNumber <= 31)
//        {
//            // Program ISER0 register
//            *NVIC_ISER0 |= (1 << IRQNumber);
//        }
//        else if(IRQNumber > 31 && IRQNumber < 64)
//        {
//            // Program ISER1 register
//            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
//        }
//        else if(IRQNumber >= 64 && IRQNumber < 96)
//        {
//            // Program ISER2 register
//            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
//        }
//    }
//    else
//    {
//        if(IRQNumber <= 31)
//        {
//            // Program ICER0 register
//            *NVIC_ICER0 |= (1 << IRQNumber);
//        }
//        else if(IRQNumber > 31 && IRQNumber < 64)
//        {
//            // Program ICER1 register
//            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
//        }
//        else if(IRQNumber >= 64 && IRQNumber < 96)
//        {
//            // Program ICER2 register
//            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
//        }
//    }
//}
//
/*********************************************************************
* @fn              - SPI_IRQPriorityConfig
*
* @brief           - This function configures the priority of the IRQ
*
* @param[in]       - IRQ number
* @param[in]       - IRQ priority (0 to 15)
*
* @return          - none
*
* @Note            - Configures the NVIC priority registers
*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
   // 1. Find out the IPR register
   uint8_t iprx = IRQNumber / 4;
   uint8_t iprx_section = IRQNumber % 4;

   uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
   *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn              - SPI_SendDataIT
 *
 * @brief           - This function sends data over SPI peripheral (interrupt mode)
 *
 * @param[in]       - Pointer to SPI handle structure
 * @param[in]       - Pointer to Tx buffer
 * @param[in]       - Length of data to be transmitted (in bytes)
 *
 * @return          - State (SPI_READY or SPI_BUSY_IN_TX)
 *
 * @Note            - This is a non-blocking call (interrupt based)
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if(state != SPI_BUSY_IN_TX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that
        //    no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

        // 4. Data transmission will be handled by the ISR code
    }

    return state;
}

/*********************************************************************
 * @fn              - SPI_ReceiveDataIT
 *
 * @brief           - This function receives data over SPI peripheral (interrupt mode)
 *
 * @param[in]       - Pointer to SPI handle structure
 * @param[in]       - Pointer to Rx buffer
 * @param[in]       - Length of data to be received (in bytes)
 *
 * @return          - State (SPI_READY or SPI_BUSY_IN_RX)
 *
 * @Note            - This is a non-blocking call (interrupt based)
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if(state != SPI_BUSY_IN_RX)
    {
        // 1. Save the Rx buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        // 2. Mark the SPI state as busy in reception so that
        //    no other code can take over same SPI peripheral until reception is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

        // 4. Data reception will be handled by the ISR code
    }

    return state;
}

/*********************************************************************
 * @fn              - SPI_IRQHandling
 *
 * @brief           - This function handles the SPI interrupt
 *
 * @param[in]       - Pointer to SPI handle structure
 *
 * @return          - none
 *
 * @Note            - Checks which event caused the interrupt and handles it accordingly
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t temp1, temp2;

    // Check for TXE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if(temp1 && temp2)
    {
        // Handle TXE
        spi_txe_interrupt_handle(pHandle);
    }

    // Check for RXNE
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if(temp1 && temp2)
    {
        // Handle RXNE
        spi_rxne_interrupt_handle(pHandle);
    }

    // Check for OVR flag
    temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if(temp1 && temp2)
    {
        // Handle OVR error
        spi_ovr_err_interrupt_handle(pHandle);
    }
}

/*********************************************************************
 * @fn              - SPI_ClearOVRFlag
 *
 * @brief           - This function clears the OVR (Overrun) flag
 *
 * @param[in]       - Base address of the SPI peripheral
 *
 * @return          - none
 *
 * @Note            - OVR flag is cleared by reading DR and then reading SR
 */
static void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp; // To avoid unused variable warning
}

/*********************************************************************
 * @fn              - SPI_CloseTransmission
 *
 * @brief           - This function closes the SPI transmission
 *
 * @param[in]       - Pointer to SPI handle structure
 *
 * @return          - none
 *
 * @Note            - Disables TXEIE interrupt and resets Tx state variables
 */
static void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

/*********************************************************************
 * @fn              - SPI_CloseReception
 *
 * @brief           - This function closes the SPI reception
 *
 * @param[in]       - Pointer to SPI handle structure
 *
 * @return          - none
 *
 * @Note            - Disables RXNEIE interrupt and resets Rx state variables
 */
static void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

/*********************************************************************
 * @fn              - SPI_ApplicationEventCallback
 *
 * @brief           - This function is the application callback
 *
 * @param[in]       - Pointer to SPI handle structure
 * @param[in]       - Application event
 *
 * @return          - none
 *
 * @Note            - This is a weak implementation. The application may override this function
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // This is a weak implementation. The application may override this function.
}

/*
 * Some helper function implementations (these are private to driver)
 */

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // Check the DFF bit in CR1
    if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16-bit DFF
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    }
    else
    {
        // 8-bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if(!pSPIHandle->TxLen)
    {
        // TxLen is zero, so close the SPI transmission and inform the application that
        // TX is over
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // Check the DFF bit in CR1
    if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        // 16-bit DFF
        *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        (uint16_t*)pSPIHandle->pRxBuffer++;
    }
    else
    {
        // 8-bit DFF
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if(!pSPIHandle->RxLen)
    {
        // Reception is complete
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
   uint8_t temp;
   // 1. Clear the OVR flag
   if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
   {
       SPI_ClearOVRFlag(pSPIHandle->pSPIx);
   }
   (void)temp;

   // 2. Inform the application
   SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}