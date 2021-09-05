/* OpenARMs usart.c - USART API
 * @author: SL7
 *
 * Changelog v2:
 *   |__v2.1
 *   |    |__ changed to USART port struct
 *
 *
 */




#ifndef _STM_UART_H
#define _STM_UART_H

#include <stm32f4xx.h>
#include <stdbool.h>

#define USART2_CLK  16000000
#define USARTx_CLK  50000000


#define USART1_PORT     GPIOA
#define USART2_PORT     GPIOA
#define USART6_PORT     GPIOC
// USART1 pins
#define USART1_RX       PA10
#define USART1_TX       PA9
#define USART1_CTS      PA11
#define USART1_CK       PA8
#define USART1_RTS      PA12
// USART2 pins
#define USART2_CTS      PA0
#define USART2_RTS      PA1
#define USART2_TX       PA2
#define USART2_RX       PA3
#define USART2_CK       PA4
// USART6 pins
#define USART6_TX       PA11
#define USART6_RX       PA12
// USART settings
#define USART_CR2_STOPBITS_OFFSET       0x0C
#define USART_CR1_WORDLEN_OFFSET        0x0C
#define USART_PARITY_EN_OFFSET          0x0A
#define USART_PARITY_EVEN_ODD_OFFSET    0x09

#define USART_TX_MODE       USART_CR1_TE
#define USART_RX_MODE       USART_CR1_RE
#define USART_RX_TX_MODE    (USART_TX_MODE | USART_RX_MODE)
#define USART_EN            USART_CR1_UE
#define USART_LIN_EN        USART_CR2_LINEN

#define USART_STOPBITS_1    (0x00 << USART_CR2_STOPBITS_OFFSET)
#define USART_STOPBITS_0_5  (0x01 << USART_CR2_STOPBITS_OFFSET)
#define USART_STOPBITS_2    (0x02 << USART_CR2_STOPBITS_OFFSET)
#define USART_STOPBITS_1_5  (0x03 << USART_CR2_STOPBITS_OFFSET)

#define USART_PARITY_NEN    (0x00 << USART_PARITY_EN_OFFSET)
#define USART_PARITY_EN     (0x01 << USART_PARITY_EN_OFFSET)

#define USART_PARITY_EVEN   (0x00 << USART_PARITY_EVEN_ODD_OFFSET)
#define USART_PARITY_ODD    (0x01 << USART_PARITY_EVEN_ODD_OFFSET)

#define USART_TXE_TIMEOUT       100000
#define USART_RXNE_TIMEOUT      USART_TXE_TIMEOUT
// USART buffer length
#define USART_CHAR_BUFFER_LEN   255

// USART TX/RX Stream numbers
#define USART1_TX_DMA_STREAM  	7
#define USART1_RX_DMA_STREAM 	5
#define USART2_TX_DMA_STREAM  	6
#define USART2_RX_DMA_STREAM 	5
#define USART6_TX_DMA_STREAM  	7
#define USART6_RX_DMA_STREAM 	2
// USARTx DMA channel
#define USART1_DMA_CHANNEL 	4
#define USART2_DMA_CHANNEL 	4
#define USART6_DMA_CHANNEL  5
// USARTx DMA controller
#define USART1_DMA 	DMA2
#define USART2_DMA 	DMA1
#define USART6_DMA 	DMA2

#define USART_BUF_SIZE  	1024

#define ENABLE      1
#define DISABLE     0

typedef struct _USART_port {
    USART_TypeDef *usart;
    uint32_t baud;
    uint32_t mode;
    uint32_t stop_bits; 
    uint32_t parity_enable;
    uint32_t parity_even_odd;
    bool _txeie;
    bool _rxneie;
    bool _clken;
    bool _dmaen;
} USART_port;


typedef enum usart_err {
    USART_OK,
    USART_UNDEFINED,
} usart_err_t;


static char _send_buff[1024];
static char _recv_buff[1024];

//static char _uart_buf_write[UART_BUF_SIZE];
//static char _uart_buf_read[UART_BUF_SIZE];

uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud);


// NEW FUNCTIONS
usart_err_t USART_init(USART_port *port);

void USART_write(USART_port *port, int ch);
void USART_printf(USART_port *port, const char *format, ...);

uint8_t USART_read(USART_port *port);
void USART_scanf(USART_port *port, char *buff);

bool USART_has_input(USART_port *port);

void USART_interrupt_enable(USART_port *port);
void USART_interrupt_disable(USART_port *port);

void USART_TXE_interrupt_enable(USART_port *port);
void USART_TXE_interrupt_disable(USART_port *port);

void USART_RXNE_interrupt_enable(USART_port *port);
void USART_RXNE_interrupt_disable(USART_port *port);

void USART_CLK_enable(USART_port *port);
void USART_CLK_disable(USART_port *port);

void USART_DMA_enable(USART_port *port);
void USART_DMA_disable(USART_port *port);

void USART_disable(USART_port *port);
#endif
