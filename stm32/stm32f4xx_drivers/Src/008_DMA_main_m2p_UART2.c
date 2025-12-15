/*
 * 008_DMA_main_m2p_UART2.c
 *
 *  Created on: Dec 13, 2025
 *      Author: Rahul B.
 */

#include <stm32f446xx.h>

void button_init();
void uart2_init();
void dma1_init(); 


/**
 * @brief Initializes the User button (PC13) for EXTI Falling Edge Interrupt.
 */
void button_init() {
    GPIO_Handle_t GpioBtn;

    // 1. Configure PC13 for Interrupt (Falling Edge)
    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // This triggers all EXTI setup in GPIO_Init
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; 

    // 2. This single line handles MODER, SYSCFG, FTSR, and IMR setup
    GPIO_Init(&GpioBtn); 
    
    // 3. This line enables the interrupt in the NVIC
    GPIO_IRQInterruptConfig(EXTI15_10_IRQn, ENABLE);
}

/**
 * @brief Initializes UART2 for transmission
 * Pins: TX on PA2, configured for Alternate Function AF7 (for USART2)
 */
void uart2_init() {
    GPIO_Handle_t UsartPins;
    USART_Handle_t Usart2Handle;

    // 1. Enable clock for GPIO Port A and USART2
    GPIO_PeriClockControl(GPIOA, ENABLE);
    USART_PeriClockControl(USART2, ENABLE);

    // 2. Configure GPIO Pins (PA2 - TX)
    UsartPins.pGPIOx = GPIOA;
    UsartPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    UsartPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF7; // USART2 AF is 7
    UsartPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    UsartPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up recommended for TX
    UsartPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // TX Pin (PA2)
    UsartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&UsartPins);

    // 3. Configure USART2 Peripheral
    Usart2Handle.pUSARTx = USART2;
    Usart2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    Usart2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    Usart2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    Usart2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    Usart2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    Usart2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

    USART_Init(&Usart2Handle);
    
    // 4. Enable the USART2 Peripheral
    USART_PeripheralControl(USART2, ENABLE);
}

/**
 * @brief Initializes DMA1 Stream 6 Channel 4 for UART2_TX (Memory-to-Peripheral)
 * DMA1 Stream 6 is dedicated to Channel 4 for USART2_TX (check RM0390 section 8.3.11).
 * 
 */
/* void dma1_init() {
    // 1. Enable DMA1 peripheral clock
    // RCC->AHB1ENR (AHB1 Peripheral Clock Enable Register)
    RCC->AHB1ENR |= (1 << 21); // Set DMA1EN bit (Bit 21)

    // 2. Disable DMA Stream 6 (write 0 to EN bit in CR register)
    // The DMA Stream must be disabled before configuring the registers
    DMA1_Stream6->CR &= ~(1 << 0);
    // Wait for the stream to be disabled (optional but good practice)
    while (DMA1_Stream6->CR & (1 << 0));

    // 3. Clear all interrupt flags for Stream 6 in the Low Interrupt Flag Register (LIFCR)
    // Clear FEIF6 (0), DMEIF6 (2), TEIF6 (3), HTIF6 (4), TCIF6 (5)
    DMA1->HIFCR = (0b111101 << 16); // Bits 16 to 21 for Stream 6 flags in HIFCR

    // 4. Configure the Stream Control Register (CR)
    // * Channel Selection: Channel 4 (for USART2_TX)
    DMA1_Stream6->CR &= ~(0b111 << 25); // Clear bits 25-27 (CHSEL)
    DMA1_Stream6->CR |= (4 << 25);      // Set CHSEL to 4

    // * Data Transfer Direction: Memory-to-Peripheral (M2P)
    DMA1_Stream6->CR &= ~(0b11 << 6); // Clear bits 6-7 (DIR)
    DMA1_Stream6->CR |= (1 << 6);     // Set DIR to 01 (M2P)

    // * Peripheral Increment Mode: Fixed address (Peripheral is DR)
    DMA1_Stream6->CR &= ~(1 << 9);    // Clear PINC bit

    // * Memory Increment Mode: Increment address (Memory is Tx_buffer)
    DMA1_Stream6->CR |= (1 << 10);    // Set MINC bit

    // * Peripheral Data Size (PSIZE) & Memory Data Size (MSIZE): 8-bit (Byte)
    DMA1_Stream6->CR &= ~(0b11 << 11); // PSIZE = 00 (Byte)
    DMA1_Stream6->CR &= ~(0b11 << 13); // MSIZE = 00 (Byte)

    // * Transfer Mode: Normal Mode (Direct Mode is fine for this)
    DMA1_Stream6->CR &= ~(1 << 4);    // Clear PFCTRL (Peripheral flow controller)

    // 5. Configure the number of data items to transfer (NDTR)
    DMA1_Stream6->NDTR = len;

    // 6. Configure the Peripheral Address Register (PAR)
    // The peripheral address is the USART2 Data Register (DR)
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

    // 7. Configure the Memory 0 Address Register (M0AR)
    // The memory address is the start of our Tx_buffer
    DMA1_Stream6->M0AR = (uint32_t)Tx_buffer;

    // 8. Enable the DMA Stream (optional here, we enable it right before transfer)
    // DMA1_Stream6->CR |= (1 << 0);
}
 */
/**
 * @brief Main function
 * The transfer will be triggered by a button press (PC13 is LOW).
 */
int main() {
    // 1. Initialize Peripherals
    // uart2_init();
    button_init();
    printf("Button init...");
    // dma1_init();

/*     // 2. Enable DMA transfer request in USART2 (CR3 register)
    // DMAT bit (Bit 7) in USART_CR3
    USART2->CR3 |= (1 << USART_CR3_DMAT);

    // 3. Main Loop
    while (1) {
        // Read the button state (PC13 - Active LOW)
        // GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) returns 1 (high) if not pressed, 0 (low) if pressed.
        if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0) {
            // Button is pressed (Active LOW)

            // Re-configure NDTR with the total length before enabling the DMA stream
            DMA1_Stream6->NDTR = len;

            // Enable the DMA Stream (Starts the transfer)
            DMA1_Stream6->CR |= (1 << 0);

            // Wait until the DMA transfer is complete (TCIF6 flag is set)
            // The TCIF6 flag is bit 21 in the HISR (High Interrupt Status Register) or bit 5 in the status word
            while (!(DMA1->HISR & (1 << 21))); // HISR bit 21 is TCIF6

            // Clear the Transfer Complete Flag (TCIF6)
            DMA1->HIFCR |= (1 << 21);

            // Disable the DMA Stream after transfer is complete
            DMA1_Stream6->CR &= ~(1 << 0);

            // Add a debounce/delay to prevent multiple transfers from a single button press
            delay_ms(300);
        }
    }

    // This line is unreachable, but good practice
    return 0; */
    while(1);
}
