/* OpenARMs usart.c - USART API
 * @author: SL7
 *
 * Changelog v2:
 *   |__v2.1
 *   |    |__ changed to USART port struct
 *
 * TODO v2.2:
 * - [ ] test functionallity
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

#define USART1_RX       PA10
#define USART1_TX       PA9
#define USART1_CTS      PA11
#define USART1_CK       PA8
#define USART1_RTS      PA12

#define USART2_CTS      PA0
#define USART2_RTS      PA1
#define USART2_TX       PA2
#define USART2_RX       PA3
#define USART2_CK       PA4

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

typedef struct _USART_port {
    USART_TypeDef *usart;
    uint32_t baud;
    uint32_t mode;
    uint32_t stop_bits; 
    uint32_t parity_enable;
    uint32_t parity_even_odd;
} USART_port;


typedef enum usart_err {
    USART_OK,
    USART_UNDEFINED,
} usart_err_t;

usart_err_t USART_init_bak(USART_TypeDef *USARTx, uint32_t baud, uint32_t mode, uint32_t stop_bits, uint32_t parity_enable, uint32_t parity_even_odd);
uint16_t USART_compute_div(uint32_t periph_clk, uint32_t baud);


void USART_write_bak(USART_TypeDef *USARTx, int ch);
void USART_printf_bak(USART_TypeDef *USARTx, const char *format, ...);

uint8_t USART_read_bak(USART_TypeDef *USARTx);
void USART_scanf_bak(USART_TypeDef *USARTx, char *buff);

bool USART_has_input_bak(USART_TypeDef *USARTx);

void USART_interrupt_enable_bak(USART_TypeDef *USARTx);
void USART_interrupt_disable_bak(USART_TypeDef *USARTx);

void USART_disable_bak(USART_TypeDef *USARTx);


// NEW FUNCTIONS
usart_err_t USART_init(USART_port *port);

void USART_write(USART_port *port, int ch);
void USART_printf(USART_port *port, const char *format, ...);

uint8_t USART_read(USART_port *port);
void USART_scanf(USART_port *port, char *buff);

bool USART_has_input(USART_port *port);

void USART_interrupt_enable(USART_port *port);
void USART_interrupt_disable(USART_port *port);

void USART_disable(USART_port *port);
#endif
