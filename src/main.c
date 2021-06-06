#include <stm32f4xx.h>
#include <stdbool.h>
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"
#include "stm32/f4/uart/uart.h"
#include "stm32/f4/rcc/rcc.h"


int main(void) {
    SystemInit();
    //set_system_clock(); 
    GPIO_enable(GPIOB, 8, GPIO_OUTPUT);        
    USART_init(USART2, 115200, USART_RX_TX_MODE, USART_STOPBITS_1, USART_PARITY_NEN, USART_PARITY_EVEN); 

    //const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    //char usart_test[255];
     
    while (1) {
        /*
        GPIO_write(GPIOB, 8, GPIO_ON);
        delayMs(500);
        GPIO_write(GPIOB, 8, GPIO_OFF);
        delayMs(500);
        */
        GPIO_toggle(GPIOB, 8);
        //USART_write(USART2, 'A');
        USART_printf(USART2, "Hello World\n");
        USART_printf(USART2, "This is a value: %.3f\n", 2.123);
        //USART_printf(USART2, "System Core Clock: %ld\n", test->ahb_freq);
        //USART_printf(USART2, "USART has input: %d\n", USART_has_input(USART2));
        //USART_scanf(USART2, usart_test);
        //USART_printf(USART2, "You typed: %s\n", usart_test);
        delayMs(500);
    }
}
