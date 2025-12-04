/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Nov 26, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx.h" 

uint16_t AHB_Prescaler[] = {2,4,8,16,64,128,256,512};
uint8_t APB_Prescaler[] = {2,4,8,16}; 

/* ----------- STATIC HELPER FUNCTION PROTOTYPES ----------- */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx); 
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx); 
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx); 

/*********************************************************************
 * @fn              - I2C_GenerateStartCondition
 *
 * @brief           - Generates the START condition on I2C bus  
 *                    (Master mode only)
 *
 * @param[in]       pI2Cx : I2C peripheral base address
 *
 * @return          - None
 *********************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    /* Set START bit in CR1 register */
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/*********************************************************************
 * @fn              - I2C_GenerateStopCondition
 *
 * @brief           - Generates the STOP condition on the I2C bus
 *                    (Releases the bus)
 *
 * @param[in]       pI2Cx : I2C peripheral base address
 *
 * @return          - None
 *********************************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    /* Set STOP bit in CR1 register */
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


/*********************************************************************
 * @fn              - I2C_ExecuteAddressPhase
 *
 * @brief           - Sends the slave address with write bit (R/W = 0)
 *
 * @param[in]       pI2Cx     : I2C peripheral base address
 * @param[in]       SlaveAddr : Slave 7-bit address
 *
 * @return          - None
 *
 * @note            - Before sending, address is shifted left and
 *                    R/W bit is cleared for write mode.
 *********************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    /* Shift left for R/W bit */
    SlaveAddr = SlaveAddr << 1;

    /* Clear R/W bit → Write operation */
    SlaveAddr &= ~(1);

    /* Load address into DR */
    pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    /* Shift left for R/W bit */
    SlaveAddr = SlaveAddr << 1;

    /* Set R/W bit → Read operation so make it 1 */
    SlaveAddr |= 1;

    /* Load address into DR */
    pI2Cx->DR = SlaveAddr;
}
/*********************************************************************
 * @fn              - I2C_ClearADDRFlag
 *
 * @brief           - Clears the ADDR flag by reading SR1 and SR2
 *
 * @param[in]       pI2Cx : I2C peripheral base address
 *
 * @return          - None
 *
 * @note            - Mandatory sequence per RM: 
 *                      1) Read SR1
 *                      2) Read SR2
 *                    This clears the ADDR flag.
 *********************************************************************/
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
    uint32_t dummyRead = pI2Cx->SR1;  /* Read SR1 */
    dummyRead = pI2Cx->SR2;           /* Read SR2 */
    (void) dummyRead;                 /* Prevent unused variable warning */
}


/*********************************************************************
 * @fn              - I2C_PeriClockControl
 *
 * @brief           - This function enables or disables peripheral
 *                    clock for the given I2C peripheral
 *
 * @param[in]       - Base address of the I2C peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pI2Cx == I2C1) I2C1_PCLK_EN();
        else if(pI2Cx == I2C2) I2C2_PCLK_EN();
        else if(pI2Cx == I2C3) I2C3_PCLK_EN();
    }
    else
    {
        if(pI2Cx == I2C1) I2C1_PCLK_DI();
        else if(pI2Cx == I2C2) I2C2_PCLK_DI();
        else if(pI2Cx == I2C3) I2C3_PCLK_DI();
    }
}

/*********************************************************************
 * @fn              - RCC_GetPLLOutputClock
 ********************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
    uint32_t pllclk, pll_m, pll_n, pll_p;
    uint32_t temp;

    temp = RCC->PLLCFGR;

    pll_m = temp & 0x3F;
    pll_n = (temp >> 6) & 0x1FF;
    pll_p = ((temp >> 16) & 0x3) + 1;
    pll_p = pll_p * 2; // actual P division factor

    uint32_t clk_src = (temp >> 22) & 0x1;

    if(clk_src == 0)
    {
        pllclk = HSI_CLK_VALUE;
    }
    else
    {
        pllclk = HSE_CLK_VALUE;
    }

    return (pllclk / pll_m) * pll_n / pll_p;
}

/*********************************************************************
 * @fn              - RCC_GetPCLK1Value
 ********************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t systemClk = 0, pclk1;
    uint32_t clksrc, temp, ahbp, apb1p;

    /* Get system clock source */
    clksrc = (RCC->CFGR >> 2) & 0x3;

    if(clksrc == 0)
        systemClk = HSI_CLK_VALUE;
    else if(clksrc == 1)
        systemClk = HSE_CLK_VALUE;
    else if(clksrc == 2)
        systemClk = RCC_GetPLLOutputClock();

    /* Get AHB prescaler */
    temp = (RCC->CFGR >> 4) & 0xF;
    if(temp < 8) ahbp = 1;
    else ahbp = AHB_Prescaler[temp - 8];

    /* Get APB1 prescaler */
    temp = (RCC->CFGR >> 10) & 0x7;
    if(temp < 4) apb1p = 1;
    else apb1p = APB_Prescaler[temp - 4];

    pclk1 = (systemClk / ahbp) / apb1p;

    return pclk1;
}

/*********************************************************************
 * @fn              - I2C_Init
 *
 * @brief           - This function initializes the given I2C peripheral
 *
 * @param[in]       - Pointer to I2C handle structure
 *
 * @return          - none
 *
 * @Note            - Configure:
 *                    - ACK Control
 *                    - Device own address
 *                    - Clock Control registers
 *                    - TRISE configuration
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // Enable peripheral clock
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    /*************** ACK Control ***************/
    tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
    pI2CHandle->pI2Cx->CR1 = tempreg;

    /*************** Configure FREQ field ***************/
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

    /*************** Device Own Address ***************/
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14);  // Enable ACK for own address
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    /*************** CCR Configuration ***************/
    uint16_t ccr_value = 0;
    uint32_t pclk1 = RCC_GetPCLK1Value();

    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // Standard mode
        ccr_value = (pclk1 / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        // pI2CHandle->pI2Cx->CCR = (ccr_value & 0xFFF);
        
        pI2CHandle->pI2Cx->CCR &= ~(0xFFF);      // Clear 12 bits
        pI2CHandle->pI2Cx->CCR |= (ccr_value & 0xFFF);

    }
    else
    {
        pI2CHandle->pI2Cx->CCR = (1 << 15) | (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (pclk1 / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        else
        {
            ccr_value = (pclk1 / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }

        pI2CHandle->pI2Cx->CCR |= (ccr_value & 0xFFF);
    }

    /*************** TRISE Configuration ***************/
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        tempreg = 0; 
        tempreg = (pclk1 / 1000000U) + 1;
        
    }
    else
    {
        // pI2CHandle->pI2Cx->TRISE = ((pclk1 / 1000000U) * 300 / 1000) + 1;
        tempreg = 0; 
        tempreg = ((pclk1 / 1000000U) * 300 / 1000) + 1;

    }

    pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F); 
}

/*********************************************************************
 * @fn              - I2C_DeInit
 *
 * @brief           - This function de-initializes the I2C peripheral
 *
 * @param[in]       - Base address of the I2C peripheral
 *
 * @return          - none
 *
 * @Note            - Resets peripheral registers
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    if(pI2Cx == I2C1)       I2C1_REG_RESET();
    else if(pI2Cx == I2C2)  I2C2_REG_RESET();
    else if(pI2Cx == I2C3)  I2C3_REG_RESET();
}

/*********************************************************************
 * @fn              - I2C_IRQPriorityConfig
 *
 * @brief           - This function configures the priority of the IRQ
 *
 * @param[in]       - IRQNumber     : IRQ position in NVIC
 * @param[in]       - IRQPriority   : Priority value (0 to 15 typically)
 *
 * @return          - none
 *
 * @Note            - Each IPR register is 32-bit, storing priority of 4 IRQs.
 *                    - iprx         = IPR register number
 *                    - iprx_section = section inside IPR (0–3)
 *                    - shift_amount = number of bits to left-shift priority
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. Identify which Interrupt Priority Register (IPR) to configure
    uint8_t iprx = IRQNumber / 4;

    // 2. Identify the section inside the IPR register
    uint8_t iprx_section = IRQNumber % 4;

    // 3. ARM Cortex-M implements only upper bits (e.g., 4 bits)
    //    So we need to shift accordingly
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    // 4. Write the priority into the IPR register
    // *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
    *(NVIC_PR_BASE_ADDR + iprx) &= ~(0xFF << (8 * iprx_section));  // Clear first
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);   // Then set
}


/*********************************************************************
 * @fn              - I2C_PeripheralControl
 *
 * @brief           - Enable or Disable the I2C Peripheral
 *
 * @param[in]       - pI2Cx   : Base address of I2C (I2C1, I2C2...)
 * @param[in]       - EnOrDi  : ENABLE (1) or DISABLE (0)
 *
 * @return          - none
 *
 * @Note            - CR1 register controls the PE (Peripheral Enable) bit
 *                    PE = 1 → I2C peripheral enabled
 *                    PE = 0 → I2C peripheral disabled
 ***********************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Set the PE bit in CR1 register to enable the peripheral
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        // Clear the PE bit to disable the peripheral
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

/*********************************************************************
 * @fn              - I2C_GetFlagStatus
 *
 * @brief           - This function returns the status of the given flag in I2C status register
 *
 * @param[in]       - Base address of the i2C peripheral
 * @param[in]       - Flag name (bit position in SR register)
 *
 * @return          - FLAG_SET or FLAG_RESET
 *
 * @Note            - none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if(pI2Cx->SR1 & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/*********************************************************************
 * @fn              - I2C_MasterSendData
 *
 * @brief           - Sends data from Master to Slave over I2C
 *
 * @param[in]       - pI2CHandle : I2C handle structure
 * @param[in]       - pTxBuffer  : Pointer to data buffer
 * @param[in]       - Len        : Number of bytes to send
 * @param[in]       - SlaveAddr  : 7-bit slave address
 * @param[in]       - Sr         : Repeated start enable/disable
 *                                 I2C_ENABLE_SR  → Do NOT generate STOP
 *                                 I2C_DISABLE_SR → Generate STOP
 *
 * @return          - none
 *********************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,
                        uint8_t *pTxBuffer,
                        uint32_t Len,
                        uint8_t SlaveAddr,
                        uint8_t Sr)
{
    /* 1. Generate START condition */
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    /* 2. Wait for SB flag = START generated */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    /* 3. Send Slave Address + Write bit */
    I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

    /* 4. Wait for ADDR flag = address sent */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    /* 5. Clear ADDR by reading SR1 + SR2 */
    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

    /* 6. Send data bytes */
    while(Len > 0)
    {
        /* Wait until TXE = 1 (DR empty) */
        while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

        pI2CHandle->pI2Cx->DR = *pTxBuffer;

        pTxBuffer++;
        Len--;
    }

    /* 7. Wait for BTF = 1 (Shift Register empty) */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    /* 8. STOP CONDITION: Generate only if Sr == DISABLE */
    if(Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

    /* Transmission complete */
}

/*********************************************************************
 * @fn              - I2C_MasterReceiveData
 *
 * @brief           - Master reads data from Slave over I2C
 *
 * @param[in]       - pI2CHandle : I2C handle structure
 * @param[in]       - pRxBuffer  : Buffer to store received bytes
 * @param[in]       - Len        : Number of bytes to read
 * @param[in]       - SlaveAddr  : 7-bit slave address
 * @param[in]       - Sr         : Repeated start enable/disable
 *
 * @return          - none
 *********************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,
                           uint8_t *pRxBuffer,
                           uint8_t Len,
                           uint8_t SlaveAddr,
                           uint8_t Sr)
{
    // 1) Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2) Confirm that START is generated (SB = 1)
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

    // 3) Send slave address with READ (1)
    I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

    // 4) Wait until address phase completes (ADDR = 1)
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

    // ------- SINGLE BYTE READ CASE -------
    if(Len == 1)
    {
        // Disable ACK (NACK will be sent for last byte)
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // Clear ADDR flag by reading SR1 & SR2
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        // Wait until RXNE becomes 1 (data is ready)
        while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

        // Generate STOP if repeated start is not requested
        if(Sr == I2C_DISABLE_SR)
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // Read the data
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

    }
    // ------- MULTI BYTE READ CASE -------
    else if (Len > 1)
    {
        // Clear ADDR flag
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        // Read remaining bytes
        for(uint32_t i = Len; i > 0; i--)
        {
            // Wait until data byte arrives
            while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

            if(i == 2)
            {
                // 2nd last byte received → disable ACK for last byte
                I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                // Generate STOP if repeated start not required
                if(Sr == I2C_DISABLE_SR)
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }

            // Read data
            *pRxBuffer = pI2CHandle->pI2Cx->DR;
            pRxBuffer++;
        }
    }

    // Re-enable ACKing (optional)
    if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}

/*********************************************************************
 * @fn              - I2C_MasterSendDataIT
 *
 * @brief           - Non-blocking API to send data over I2C (Master mode)
 *
 * @return          - I2C_BUSY_IN_TX or I2C_READY
 *********************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,
                             uint8_t *pTxbuffer,
                             uint32_t Len,
                             uint8_t SlaveAddr,
                             uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if(busystate != I2C_BUSY_IN_TX && busystate != I2C_BUSY_IN_RX)
    {
        /* 1. Save buffer and length */
        pI2CHandle->pTxBuffer = pTxbuffer;
        pI2CHandle->TxLen = Len;

        /* 2. Save slave address */
        pI2CHandle->DevAddr = SlaveAddr;

        /* 3. Repeated start option */
        pI2CHandle->Sr = Sr;

        /* 4. Mark handle state as TX busy */
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;

        /* 5. Generate START condition */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* 6. Enable IT events: 
              - ITEVTEN = event interrupt
              - ITBUFEN = buffer interrupt
              - ITERREN = error interrupt */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

/*********************************************************************
 * @fn              - I2C_MasterReceiveDataIT
 *
 * @brief           - Non-blocking API to receive data over I2C (Master mode)
 *
 * @return          - I2C_BUSY_IN_RX or I2C_READY
 *********************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,
                                uint8_t *pRxBuffer,
                                uint32_t Len,
                                uint8_t SlaveAddr,
                                uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if(busystate != I2C_BUSY_IN_TX && busystate != I2C_BUSY_IN_RX)
    {
        /* 1. Save buffer and len */
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;

        /* 2. Reset RxSize (used in state machine) */
        pI2CHandle->RxSize = Len;

        /* 3. Save slave address */
        pI2CHandle->DevAddr = SlaveAddr;

        /* 4. Repeated start */
        pI2CHandle->Sr = Sr;

        // /* 5. Mark handle busy */
        // pI2CHandle->TxRxState = I2C_BUSY_IN_RX;

        // /* 6. Generate START */
        // I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* 5. Mark handle busy */
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;

        /* 5a. Enable ACK if configured */
        if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
        {
            I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
        }

        /* 6. Generate START */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* 7. Enable interrupts: event, buffer, error */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}



/*********************************************************************
 * @fn              - I2C_ManageAcking
 *
 * @brief           - Enable or Disable ACKing in I2C peripheral
 *
 * @param[in]       - pI2Cx  : I2C peripheral base address
 * @param[in]       - EnOrDi : I2C_ACK_ENABLE or I2C_ACK_DISABLE
 *
 * @return          - none
 *********************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == I2C_ACK_ENABLE)
    {
        // Set ACK bit (bit 10 in CR1)
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        // Clear ACK bit to disable acknowledge
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

/*
 * Enable or Disable IRQ
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber >= 32 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
    }
    else    // Disable
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber >= 32 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}

/*********************************************************************
 * @fn      		  - I2C_EV_IRQHandling
 *
 * @brief             - Handles I2C event interrupts for both master and slave modes
 *
 * @param[in]         - pI2CHandle : Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Interrupt handling for different I2C events (refer SR1)
 */

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//   Note: SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		// SB flag is set
		// Execute address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//   Note: When master mode : Address is sent
	//         When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// ADDR flag is set
		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				// BTF and TXE both are set
				if(pI2CHandle->TxLen == 0)
				{
					// Close the transmission
					// 1. Generate STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// 2. Reset all member elements of handle structure
					I2C_CloseSendData(pI2CHandle);

					// 3. Notify application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Do nothing (reception handled by RXNE)
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	//   Note: Stop detection flag is applicable only slave mode. For master this flag will never be set
	if(temp1 && temp3)
	{
		// STOPF flag is set
		// Clear the STOPF (read SR1, write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		// Check device mode (master mode)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// TXE flag is set
			// Do data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave mode
			// Make sure slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		// Check device mode (master mode)
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// RXNE flag is set
			// Do data reception
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave mode
			// Make sure slave is in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             - Handles I2C error interrupts
 *
 * @param[in]         - pI2CHandle : Pointer to I2C handle structure
 *
 * @return            - none
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7
 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	// Know the status of ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		// This is Bus error

		// Implement the code to clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		// This is arbitration lost error

		// Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/***********************Check for ACK failure error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		// This is ACK failure error

		// Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		// This is Overrun/underrun

		// Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		// This is Time out error

		// Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/*********************************************************************
 * @fn      		  - I2C_SlaveSendData
 *
 * @brief             - Slave sends data to master
 *
 * @param[in]         - pI2C : I2C peripheral base address
 * @param[in]         - data : Data byte to send
 *
 * @return            - none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2C, uint8_t data)
{
	pI2C->DR = data;
}

/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief             - Slave receives data from master
 *
 * @param[in]         - pI2C : I2C peripheral base address
 *
 * @return            - Received data byte
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR;
}