#include "uart.h"
#include <stm32f4xx.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

#include "../gpio/gpio.h"


usart_err_t USART_init(USART_TypeDef *USARTx, uint32_t baud, uint32_t mode, uint32_t stop_bits, uint32_t parity_enable, uint32_t parity_even_odd) {

    if (USARTx == USART1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART1_PORT, USART1_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART1_PORT, USART1_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART1_PORT, USART1_RX, GPIO_AF07);
        GPIO_select_alternate(USART1_PORT, USART1_TX, GPIO_AF07);
        USARTx->BRR = USART_compute_div(USARTx_CLK, baud);
    } else if (USARTx == USART2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART2_PORT, USART2_RX, GPIO_ALTERNATE);
        GPIO_enable(USART2_PORT, USART2_TX, GPIO_ALTERNATE);
        GPIO_select_alternate(USART2_PORT, USART2_RX, GPIO_AF07);
        GPIO_select_alternate(USART2_PORT, USART2_TX, GPIO_AF07);
        USARTx->BRR = USART_compute_div(USART2_CLK, baud);
    }
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USARTx->CR1 = 0x00;
    USARTx->CR2 = 0x00;
    USARTx->CR3 = 0x00;


    USARTx->CR1 |= parity_enable | parity_even_odd;
    USARTx->CR2 |= stop_bits;


    // TODO: calculations for USART_BRR register to select baud rate 

    USARTx->CR1 |= mode;
    USARTx->CR1 |= USART_EN;
    return USART_OK;
}

uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud) {
    return (periph_clk + (baud/2U)) / baud; 
}

void USART_write(USART_TypeDef *USARTx, int ch) {
    volatile int i = 0;
    while(!(USARTx->SR & USART_SR_TXE)) {
        if (i >= USART_TXE_TIMEOUT) {
            return;
        }
        i++;
    }

    USARTx->DR = (ch & 0xFF);
}


uint8_t USART_read(USART_TypeDef *USARTx) {
    while(!(USARTx->SR & USART_SR_RXNE));
    return USARTx->DR;
}


void USART_printf(USART_TypeDef *USARTx, const char *format, ...) {
    char buff[USART_CHAR_BUFFER_LEN];

    va_list args;
    va_start(args, format);
    
    vsprintf(buff, format, args);    

    for (int i = 0; i < strlen(buff); i++) {
        if ( buff[i] == '\n') {
            USART_write(USARTx, '\r');
        }
        USART_write(USARTx,buff[i]);
    }
    va_end(args);
}




void USART_scanf(USART_TypeDef *USARTx, char *buff) {
    int counter = 0, c;

    while ( counter < 255) {
        c = USART_read(USARTx);
        if (c == '\n') {
            buff[counter] = '\0';
            break;
        }
        buff[counter] = c;
        counter++;
    }
}


bool USART_has_input(USART_TypeDef *USARTx) {
    if (USARTx->SR & USART_SR_RXNE) {
        return true;
    }
    return false;
}
