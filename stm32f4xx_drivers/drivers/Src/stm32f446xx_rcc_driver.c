/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Dec 4, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx_rcc_driver.h"

uint16_t AHB_Prescaler[] = {2,4,8,16,64,128,256,512};
uint8_t APB_Prescaler[] = {2,4,8,16}; 

/*
 * Returns APB1 peripheral clock value (PCLK1) in Hz
 */
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

/*
 * Returns APB2 peripheral clock value (PCLK2) in Hz
 */

uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t systemClk = 0, pclk2;
    uint32_t clksrc, temp, ahbp, apb2p;

    clksrc = (RCC->CFGR >> 2) & 0x3;

    if(clksrc == 0)
        systemClk = HSI_CLK_VALUE;
    else if(clksrc == 1)
        systemClk = HSE_CLK_VALUE;
    else if(clksrc == 2)
        systemClk = RCC_GetPLLOutputClock();

    /* AHB Prescaler */
    temp = (RCC->CFGR >> 4) & 0xF;
    if(temp < 8)
        ahbp = 1;
    else
        ahbp = AHB_Prescaler[temp - 8];

    /* APB2 Prescaler (PPRE2 bits 13:11) */
    temp = (RCC->CFGR >> 11) & 0x7;
    if(temp < 4)
        apb2p = 1;
    else
        apb2p = APB_Prescaler[temp - 4];

    pclk2 = (systemClk / ahbp) / apb2p;

    return pclk2;
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
