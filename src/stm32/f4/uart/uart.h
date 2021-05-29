#ifndef _STM_UART_H
#define _STM_UART_H

#include <stm32f4xx.h>
#include <stdbool.h>

#define USART2_CLK  16000000
#define USARTx_CLK  50000000

#define USART1_PORT     GPIOA
#define USART2_PORT     GPIOA
#define USART6_PORT     GPIOC

#define USART1_RX       0x0A
#define USART1_TX       0x09
#define USART1_CTS      0x0B
#define USART1_CK       0x08
#define USART1_RTS      0x0C

#define USART2_CTS      0x00
#define USART2_RTS      0x01
#define USART2_TX       0x02
#define USART2_RX       0x03
#define USART2_CK       0x04

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

#define USART_CHAR_BUFFER_LEN   255

#define ENABLE      1
#define DISABLE     0


typedef enum usart_err {
    USART_OK
} usart_err_t;

usart_err_t USART_init(USART_TypeDef *USARTx, uint32_t baud, uint32_t mode, uint32_t stop_bits, uint32_t parity_enable, uint32_t parity_even_odd);
uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud);


void USART_write(USART_TypeDef *USARTx, int ch);
void USART_printf(USART_TypeDef *USARTx, const char *format, ...);

uint8_t USART_read(USART_TypeDef *USARTx);
void USART_scanf(USART_TypeDef *USARTx, char *buff);

bool USART_has_input(USART_TypeDef *USARTx);


#endif
