/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Dec 4, 2025
 *      Author: Rahul B.
 */

#include "stm32f446xx_usart_driver.h"

/*********************************************************************
 * @fn              - USART/UART_PeriClockControl
 *
 * @brief           - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]       - Base address of the UART peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pUSARTx == USART1) USART1_PCLK_EN();
        else if (pUSARTx == USART2) USART2_PCLK_EN();
        else if (pUSARTx == USART3) USART3_PCLK_EN();
        else if (pUSARTx == UART4)  UART4_PCLK_EN();
        else if (pUSARTx == UART5)  UART5_PCLK_EN();
        else if (pUSARTx == USART6) USART6_PCLK_EN();
    }
    else
    {
        if (pUSARTx == USART1) USART1_PCLK_DI();
        else if (pUSARTx == USART2) USART2_PCLK_DI();
        else if (pUSARTx == USART3) USART3_PCLK_DI();
        else if (pUSARTx == UART4)  UART4_PCLK_DI();
        else if (pUSARTx == UART5)  UART5_PCLK_DI();
        else if (pUSARTx == USART6) USART6_PCLK_DI();
    }
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Initializes USART peripheral with given configuration
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle structure
 *
 * @return            - none
 *
 * @Note              - Configures CR1, CR2, CR3 and BRR registers
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Enable the Receiver bit field (RE = bit 2)
		tempreg |= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Enable the Transmitter bit field (TE = bit 3)
		tempreg |= (1 << USART_CR1_TE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Enable both Transmitter and Receiver bit fields (TE=3, RE=2)
		tempreg |= ( (1 << USART_CR1_TE) | (1 << USART_CR1_RE) );
	}

    //Configure the Word length configuration item (M bit = bit 12)
	tempreg |= ( (uint32_t)pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M );

    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Enable the parity control (PCE = bit 10)
		tempreg |= (1U << USART_CR1_PCE);

		//Even parity selected by default when PCE=1 and PS=0

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Enable the parity control (PCE = bit 10)
	    tempreg |= (1U << USART_CR1_PCE);

	    //Enable ODD parity (PS = bit 9)
	    tempreg |= (1U << USART_CR1_PS);
	}

   //Program the CR1 register 
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Configure the number of stop bits inserted during USART frame transmission
	//STOP bits are at CR2 bits 13:12
	tempreg |= ((uint32_t)pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Program the CR2 register 
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;
	
	//Configuration of USART hardware flow control 
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Enable CTS flow control (CTSE = bit 9)
		tempreg |= ( 1U << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Enable RTS flow control (RTSE = bit 8)
		tempreg |= ( 1U << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Enable both CTS and RTS Flow control (CTSE=9, RTSE=8)
		tempreg |= ( (1U << USART_CR3_CTSE) | (1U << USART_CR3_RTSE) );
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}

/*
 * De-init: reset peripheral registers using RCC reset bits
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
    if (pUSARTx == USART1)          USART1_REG_RESET();
    else if (pUSARTx == USART2)     USART2_REG_RESET();
    else if (pUSARTx == USART3)     USART3_REG_RESET();
    else if (pUSARTx == UART4)      UART4_REG_RESET();
    else if (pUSARTx == UART5)      UART5_REG_RESET();
    else if (pUSARTx == USART6)     USART6_REG_RESET();
}


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Sends data over USART (blocking mode)
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle
 * @param[in]         - pTxBuffer: Pointer to transmit buffer
 * @param[in]         - Len: Number of bytes to send
 *
 * @return            - none
 *
 * @Note              - Blocking call (waits for each byte transmission)
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Wait until TXE flag is set in the SR (TXE = bit 7)
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1U << USART_SR_TXE)));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{   
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits 
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);
			
			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Increment pTxBuffer twice 
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer 
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
			
			//Increment the buffer address
			pTxBuffer++;
		}
	}

	// Wait till TC flag is set in the SR (TC = bit 6)
    while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1U << USART_SR_TC)) );

}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Receives data over USART (blocking mode)
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle
 * @param[in]         - pRxBuffer: Pointer to receive buffer
 * @param[in]         - Len: Number of bytes to receive
 *
 * @return            - none
 *
 * @Note              - Blocking call (waits for each byte reception)
 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait till RXNE flag is set in the SR (RXNE = bit 5)
        while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, (1U << USART_SR_RXNE))){

        }

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (uint16_t)(pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 
				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - Sends data using interrupt mode (non-blocking)
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle
 * @param[in]         - pTxBuffer: Pointer to transmit buffer
 * @param[in]         - Len: Number of bytes to send
 *
 * @return            - Current TX state
 *
 * @Note              - Non-blocking API
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE (TXEIE = CR1 bit 7)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
		

		//Enable interrupt for TC (TCIE = CR1 bit 6)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);		
		

	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Receives data using interrupt mode (non-blocking)
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle
 * @param[in]         - pRxBuffer: Pointer to receive buffer
 * @param[in]         - Len: Number of bytes to receive
 *
 * @return            - Current RX state
 *
 * @Note              - Non-blocking API
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Enable interrupt for RXNE (RXNEIE = CR1 bit 5)
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;

}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - Configures the baud rate for USART
 *
 * @param[in]         - pUSARTx: USART peripheral base address
 * @param[in]         - BaudRate: Desired baud rate (e.g., 9600, 115200)
 *
 * @return            - none
 *
 * @Note              - Calculates and programs BRR register based on 
 *                      APB clock and oversampling mode
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}else
	{
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * 8) + 50) / 100) & ((uint8_t)0x07);

	}else
	{
		//over sampling by 16
		F_part = ((( F_part * 16) + 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             - Handles USART interrupts
 *
 * @param[in]         - pUSARTHandle: Pointer to USART handle
 *
 * @return            - none
 *
 * @Note              - Handles TC, TXE, RXNE, CTS, IDLE, ORE flags
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	
    uint32_t temp1 , temp2;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);
	
	//Check the state of TCIE bit 
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC
		
		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
				
				//Clear the TCIE control bit 
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);
				
				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;
				
				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;
				
				//Reset the length to zero
				pUSARTHandle->TxLen = 0;
				
				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	// Check the state of TXE bit in the SR (TXE = bit 7)
    temp1 = pUSARTHandle->pUSARTx->SR & (1U << USART_SR_TXE);
	
	// Check the state of TXEIE bit in CR1 (TXEIE = bit 7)
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1U << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE
		
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					
					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						
						//Decrement the length
						pUSARTHandle->TxLen--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						
						//Decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer)  & (uint8_t)0xFF);

					//Increment the buffer address
					pUSARTHandle->pTxBuffer++;
					
					//Decrement the length
					pUSARTHandle->TxLen--;
				}
				
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero 
				//Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}
	
/*************************Check for RXNE flag ********************************************/

	// Check the state of RXNE bit in SR (RXNE = bit 5)
    temp1 = pUSARTHandle->pUSARTx->SR & (1U << USART_SR_RXNE);

    // Check the state of RXNEIE bit in CR1 (RXNEIE = bit 5)
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1U << USART_CR1_RXNEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//RXNE is set so receive data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						
						//Decrement the length
						pUSARTHandle->RxLen--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 
						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;
						 
						 //Decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					
					//Decrement the length
					pUSARTHandle->RxLen--;
				}		
					
					
			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}
	
/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);
	
	//Check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
	
	//Check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	if (pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS))
    {
        // CTS event handling (if you ever need it)
    }



	if(temp1  && temp2 )
	{
		//Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);
		
		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	// Check the status of IDLE flag bit in SR (IDLE = bit 4)
    temp1 = pUSARTHandle->pUSARTx->SR & (1U << USART_SR_IDLE);

    // Check the state of IDLEIE bit in CR1 (IDLEIE = bit 4)
    temp2 = pUSARTHandle->pUSARTx->CR1 & (1U << USART_CR1_IDLEIE);

	if(temp1 && temp2)
    {
        // Clear the IDLE flag by reading SR followed by DR (as per RM)
        (void)pUSARTHandle->pUSARTx->SR;  // Read SR
        (void)pUSARTHandle->pUSARTx->DR;  // Read DR

        // This interrupt is because of IDLE
        USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
    }

/*************************Check for Overrun detection flag ********************************************/

	//Check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;
	
	//Check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag . 
		
		//this interrupt is because of Overrun error 
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}

/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The below code will get executed in only if multibuffer mode is used. 

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			(void)pUSARTHandle->pUSARTx->DR; // Clear the flag
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			(void)pUSARTHandle->pUSARTx->DR; // Clear the flag
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			(void)pUSARTHandle->pUSARTx->DR; // Clear the flag
            USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}

}

/*
 * Peripheral control: enable/disable the USART (UE bit)
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        // Set the UE bit (bit 13) to enable USART
        pUSARTx->CR1 |= (1U << USART_CR1_UE);
    }
    else
    {
        // Clear the UE bit to disable USART
        pUSARTx->CR1 &= ~(1U << USART_CR1_UE);
    }
}

/*
 * Return flag status (SR)
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
    if (pUSARTx->SR & FlagName)
        return 1;
    return 0;
}

/*
 * Clear certain flags (basic approach)
 * Note: some flags are cleared by specific sequences (read SR then DR).
 * We handle the most common ones safely here.
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    /* For flags cleared by reading SR then DR, do that */
    if (StatusFlagName == USART_FLAG_ORE || StatusFlagName == USART_FLAG_PE ||
        StatusFlagName == USART_FLAG_FE || StatusFlagName == USART_FLAG_NE)
    {
        volatile uint32_t tmp;
        tmp = pUSARTx->SR;
        tmp = pUSARTx->DR;
        (void)tmp;
        return;
    }

    /* For other flags, write 0 to the bit (clearing) by writing to SR */
    pUSARTx->SR &= ~(StatusFlagName);
}

/*
 * IRQ config using CMSIS NVIC helpers (cast IRQNumber to IRQn_Type)
 */

/*
 * Enable or Disable USART IRQ
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
    }
    else    // Disable
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber >= 32 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}

/*
 * Configure USART IRQ priority (manual NVIC register method)
 *
 * IRQNumber   : IRQ position in NVIC (0-239)
 * IRQPriority : Priority value (0-15 typically)
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // 1. Identify which Interrupt Priority Register (IPR) to configure
    uint8_t iprx = IRQNumber / 4;

    // 2. Identify the section inside the IPR register
    uint8_t iprx_section = IRQNumber % 4;

    // 3. ARM Cortex-M implements only upper bits (NO_PR_BITS_IMPLEMENTED)
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    // 4. Write the priority into the IPR register
    *(NVIC_PR_BASE_ADDR + iprx) &= ~(0xFF << (8 * iprx_section));  // Clear first
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);   // Then set
}


/*
 * Weak callback - user should override this in application
 */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
    /* User implementation */
    (void)pUSARTHandle;
    (void)AppEv;
}
