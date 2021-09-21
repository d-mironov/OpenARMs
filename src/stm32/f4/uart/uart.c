#include "uart.h"
#include <stm32f4xx.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <core_cm4.h>

#include "../gpio/gpio.h"


usart_err_t USART_init_bak(USART_TypeDef *USARTx, uint32_t baud, uint32_t mode, uint32_t stop_bits, uint32_t parity_enable, uint32_t parity_even_odd) {
    if (USARTx == USART1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART1_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART1_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART1_RX, GPIO_AF07);
        GPIO_select_alternate(USART1_TX, GPIO_AF07);
        USARTx->BRR = USART_compute_div(USARTx_CLK, baud);
    } else if (USARTx == USART2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART2_RX, GPIO_ALTERNATE);
        GPIO_enable(USART2_TX, GPIO_ALTERNATE);
        GPIO_select_alternate(USART2_RX, GPIO_AF07);
        GPIO_select_alternate(USART2_TX, GPIO_AF07);
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        USARTx->BRR = USART_compute_div(USART2_CLK, baud);
    }

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


/**
 * USART
 * Initializes the USART with the given `port`
 * `mode`, `stop_bits`, `parity_enable` and `parity_even_odd` can 
 * remain on the default value if no change is needed.
 *
 * @param port USART port with settings
 *
 * @return usart_err_t `USART_UNDEFINED` if `port.usart=NULL`, `USART_OK` on success
 */
usart_err_t USART_init(USART_port *port) {
    if (port == NULL) {
        return USART_UNDEFINED;
    }

    if ( port->usart == USART1) {
        //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART1_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART1_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART1_RX, GPIO_AF07);
        GPIO_select_alternate(USART1_TX, GPIO_AF07);
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        (port->usart)->BRR = USART_compute_div(USARTx_CLK, port->baud); 
    } else if (port->usart == USART2) {
        //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART2_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART2_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART2_RX, GPIO_AF07);
        GPIO_select_alternate(USART2_TX, GPIO_AF07);
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        (port->usart)->BRR = USART_compute_div(USART2_CLK, port->baud); 
    } else if (port->usart == USART6) {
        //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        GPIO_enable(USART6_RX, GPIO_ALTERNATE); 
        GPIO_enable(USART6_TX, GPIO_ALTERNATE); 
        GPIO_select_alternate(USART6_RX, GPIO_AF07);
        GPIO_select_alternate(USART6_TX, GPIO_AF07);
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        (port->usart)->BRR = USART_compute_div(USARTx_CLK, port->baud); 
    } else {
        return USART_UNDEFINED;
    }

    (port->usart)->CR1 = 0x00;
    (port->usart)->CR2 = 0x00;
    (port->usart)->CR3 = 0x00;


    (port->usart)->CR1 |= port->parity_enable | port->parity_even_odd;
    (port->usart)->CR2 |= port->stop_bits;
    if (port->interrupt_driven) {
        (port->usart)->CR1 |= USART_TXE_IE;
        (port->usart)->CR1 |= USART_RXNE_IE;
        if (port->usart == USART1) {
            NVIC_EnableIRQ(USART1_IRQn);         
        } else if (port->usart == USART2) {
            NVIC_EnableIRQ(USART2_IRQn); 
        } else if (port->usart == USART6) {
            NVIC_EnableIRQ(USART6_IRQn);
        }
    }
     
    if (port->mode == 0) {
        (port->usart)->CR1 |= USART_RX_TX_MODE;
    } else {
        (port->usart)->CR1 |= port->mode;
    }

    (port->usart)->CR1 |= USART_EN;

    return USART_OK;
}

/**
 * USART write
 * \brief Blocking USART function call. Sends data when `TXE` bit is set
 *
 * @param port Port of USART (needs to be initialized)
 * @param ch character to write into data-register
 */
usart_err_t USART_write(USART_port *port, int ch) {
    if (!port->interrupt_driven) {
        while(!((port->usart)->SR & USART_SR_TXE));
        (port->usart)->DR = (ch & 0xFF);
        return USART_OK;
    } else {
        __usart_it_buf *buf;
        if (port->usart == USART2) {
            buf = &__buf_usart2; 
            buf->tx_buf[ buf->tx_in & (USART_IT_TX_BUF_SIZE-1) ] = ch;
            buf->tx_in++;
            if (buf->tx_restart) {
                buf->tx_restart = 0;
                USART2->CR1 |= USART_FLAG_TXE;
            }
        } else if (port->usart == USART1) {
            buf = &__buf_usart1; 
            buf->tx_buf[ buf->tx_in & (USART_IT_TX_BUF_SIZE-1) ] = ch;
            buf->tx_in++;
            if (buf->tx_restart) {
                buf->tx_restart = 0;
                USART1->CR1 |= USART_FLAG_TXE;
            }
    // only available if USART6 is defined
        }
    #ifdef USART6
        else if (port->usart == USART6) {
            buf = &__buf_usart6; 
            buf->tx_buf[ buf->tx_in & (USART_IT_TX_BUF_SIZE-1) ] = ch;
            buf->tx_in++;
            if (buf->tx_restart) {
                buf->tx_restart = 0;
                USART6->CR1 |= USART_FLAG_TXE;
            }
        }
    #endif
    }
    return USART_OK;
}

/**
 * USART computer divider
 * Computes divider on given peripheral clock speed and baud rate
 * 
 * @param periph_clk clock speed of given USART
 * @param baud baud rate
 */
uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud) {
    return (periph_clk + (baud/2U)) / baud; 
}

void USART_write_bak(USART_TypeDef *USARTx, int ch) {
    while(!(USARTx->SR & USART_SR_TXE));
    USARTx->DR = (ch & 0xFF);
}


uint8_t USART_read_bak(USART_TypeDef *USARTx) {
    while(!(USARTx->SR & USART_SR_RXNE));
    return USARTx->DR;
}


uint8_t USART_read(USART_port *port) {
    while(!((port->usart)->SR & USART_SR_RXNE));
    return (port->usart)->DR;
}



void USART_printf_bak(USART_TypeDef *USARTx, const char *format, ...) {
    char buff[USART_CHAR_BUFFER_LEN];

    va_list args;
    va_start(args, format);
    
    vsprintf(buff, format, args);    

    for (int i = 0; i < strlen(buff); i++) {
        if ( buff[i] == '\n') {
            USART_write_bak(USARTx, '\r');
        }
        USART_write_bak(USARTx,buff[i]);
    }
    va_end(args);
}


void USART_printf(USART_port *port, const char *format, ...) {
    char buff[USART_CHAR_BUFFER_LEN];

    va_list args;
    va_start(args, format);
    
    vsprintf(buff, format, args);    

    for (int i = 0; i < strlen(buff); i++) {
        if ( buff[i] == '\n') {
            USART_write(port, '\r');
        }
        USART_write(port, buff[i]);
    }
    va_end(args);
}


void USART_scanf_bak(USART_TypeDef *USARTx, char *buff) {
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


void USART_scanf(USART_port *port, char *buff) {
    int counter = 0, c;

    while ( counter < 255) {
        c = USART_read(port);
        if (c == '\n') {
            buff[counter] = '\0';
            break;
        }
        buff[counter] = c;
        counter++;
    }
}


bool USART_has_input_bak(USART_TypeDef *USARTx) {
    if (USARTx->SR & USART_SR_RXNE) {
        return true;
    }
    return false;
}


bool USART_has_input(USART_port *port) {
    if ((port->usart)->SR & USART_SR_RXNE) {
        return true;
    }
    return false;
}



void USART_interrupt_enable(USART_port *port) {
    if (port->usart == USART1) {
        NVIC_EnableIRQ(USART1_IRQn);
    } else if (port->usart == USART2) {
        NVIC_EnableIRQ(USART2_IRQn);
    } else if (port->usart == USART6) {
        NVIC_EnableIRQ(USART6_IRQn);
    }
}


void USART_interrupt_disable(USART_port *port) {
    if (port->usart == USART1) {
        NVIC_DisableIRQ(USART1_IRQn);
    } else if (port->usart == USART2) {
        NVIC_DisableIRQ(USART2_IRQn);
    } else if (port->usart == USART6) {
        NVIC_DisableIRQ(USART6_IRQn);
    }
}



void USART_disable_bak(USART_TypeDef *USARTx) {
    USARTx->CR1 &= ~(USART_CR1_UE);
    if ( USARTx == USART1) {
        RCC->APB1RSTR |= RCC_APB2RSTR_USART1RST;
    } else if ( USARTx == USART2 ) {
        RCC->APB2ENR |= RCC_APB1RSTR_USART2RST;
    } else if ( USARTx == USART6 ) {
        RCC->APB1RSTR |= RCC_APB2RSTR_USART6RST;
    }
}



void USART_disable(USART_port *port) {
    (port->usart)->CR1 &= ~(USART_CR1_UE);
    if ( port->usart == USART1) {
        RCC->APB1RSTR |= RCC_APB2RSTR_USART1RST;
    } else if ( port->usart == USART2 ) {
        RCC->APB2ENR |= RCC_APB1RSTR_USART2RST;
    } else if ( port->usart == USART6 ) {
        RCC->APB1RSTR |= RCC_APB2RSTR_USART6RST;
    }
    
}

void USART1_IRQHandler() {
    //TODO
    __usart_it_buf *buf;
    if (USART1->SR & USART_FLAG_TXE) {
        USART1->SR &= ~USART_FLAG_TXE;
        buf = &__buf_usart1;
        if (buf->tx_in != buf->tx_out) {
            USART1->DR = (buf->tx_buf[ buf->tx_out & (USART_IT_TX_BUF_SIZE-1)] & 0x1FF); 
            buf->tx_out++;
            buf->tx_restart = false;
        } else {
            buf->tx_restart = true;
            USART1->CR1 &= ~USART_FLAG_TXE;
        }
    }
}

void USART2_IRQHandler() {
    //GPIO_toggle(PB8);
    __usart_it_buf *buf;

    if (USART2->SR & USART_FLAG_TXE) {
        USART2->SR &= ~USART_FLAG_TXE;
        buf = &__buf_usart2;
        if (buf->tx_in != buf->tx_out) {
            USART2->DR = (buf->tx_buf[ buf->tx_out & (USART_IT_TX_BUF_SIZE-1)] & 0x1FF); 
            buf->tx_out++;
            buf->tx_restart = false;
        } else {
            buf->tx_restart = true;
            USART2->CR1 &= ~USART_FLAG_TXE;
        }
    }
}

void USART6_IRQHandler() {
    //TODO
    __usart_it_buf *buf;
    if (USART6->SR & USART_FLAG_TXE) {
        USART6->SR &= ~USART_FLAG_TXE;
        buf = &__buf_usart6;
        if (buf->tx_in != buf->tx_out) {
            USART6->DR = (buf->tx_buf[ buf->tx_out & (USART_IT_TX_BUF_SIZE-1)] & 0x1FF); 
            buf->tx_out++;
            buf->tx_restart = false;
        } else {
            buf->tx_restart = true;
            USART6->CR1 &= ~USART_FLAG_TXE;
        }
    }
}
