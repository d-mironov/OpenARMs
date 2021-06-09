#include <stm32f4xx.h>
#include <stdbool.h>
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"
#include "stm32/f4/uart/uart.h"
#include "stm32/f4/rcc/rcc.h"
#include "stm32/f4/exti/exti.h"


int main(void) {
    SystemInit();
    //set_system_clock(); 
    GPIO_enable(GPIOB, 8, GPIO_OUTPUT);        

    USART_init(USART2, 115200, USART_RX_TX_MODE, USART_STOPBITS_1, USART_PARITY_NEN, USART_PARITY_EVEN); 
    if( EXTI_attach_gpio(GPIOB, 7, EXTI_FALLING_EDGE) != EXTI_OK) {
        USART_printf(USART2, "EXTI init failed...\n");
    }


    //const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    char usart_test[255];
     
    while (1) {
        //GPIO_toggle(GPIOB, 8);
        //USART_printf(USART2, "Hello World\n");
        //USART_printf(USART2, "This is a value: %.3f\n", 2.123);
        if ( USART_has_input(USART2) ) {
            USART_scanf(USART2, usart_test);
            USART_printf(USART2, "You typed: %s\n", usart_test);
        }
    }
}



static void exti_callback(void) {
    USART_printf(USART2, "Hello from EXIT07\n");
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1<<7)) {
        // Clear PR flag
        EXTI->PR |= (1<<7);
        // Do something...
        exti_callback();
    }
}
