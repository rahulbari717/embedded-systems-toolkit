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
static void I2C_GenrateStartCondition(I2C_RegDef_t *pI2Cx); 
static void I2C_GenrateStopCondition(I2C_RegDef_t *pI2Cx); 
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx); 

/*********************************************************************
 * @fn              - I2C_GenrateStartCondition
 *
 * @brief           - Generates the START condition on I2C bus  
 *                    (Master mode only)
 *
 * @param[in]       pI2Cx : I2C peripheral base address
 *
 * @return          - None
 *********************************************************************/
static void I2C_GenrateStartCondition(I2C_RegDef_t *pI2Cx)
{
    /* Set START bit in CR1 register */
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/*********************************************************************
 * @fn              - I2C_GenrateStopCondition
 *
 * @brief           - Generates the STOP condition on the I2C bus
 *                    (Releases the bus)
 *
 * @param[in]       pI2Cx : I2C peripheral base address
 *
 * @return          - None
 *********************************************************************/
static void I2C_GenrateStopCondition(I2C_RegDef_t *pI2Cx)
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
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
    /* Shift left for R/W bit */
    SlaveAddr = SlaveAddr << 1;

    /* Clear R/W bit → Write operation */
    SlaveAddr &= ~(1);

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
        // Fast mode
        // pI2CHandle->pI2Cx->CCR |= (1 << 15);
        // pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

    	// Fast mode
    	    pI2CHandle->pI2Cx->CCR &= ~(0xFFF);  // Add this line to clear first
    	    pI2CHandle->pI2Cx->CCR |= (1 << 15);
    	    pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

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
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
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
    I2C_GenrateStartCondition(pI2CHandle->pI2Cx);

    /* 2. Wait for SB flag = START generated */
    while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

    /* 3. Send Slave Address + Write bit */
    I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

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
        I2C_GenrateStopCondition(pI2CHandle->pI2Cx);
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
// void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,
//                            uint8_t *pRxBuffer,
//                            uint8_t Len,
//                            uint8_t SlaveAddr,
//                            uint8_t Sr)
