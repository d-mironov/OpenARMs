#include <stm32f4xx.h>
#include <stdbool.h>
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"
#include "stm32/f4/uart/uart.h"
#include "stm32/f4/twowire/twowire.h"
//#include "stm32/f4/rcc/rcc.h"
//#include "stm32/f4/exti/exti.h"

#define MPU_ADDR    0x68


uint16_t read_x(I2C_port *port) {
    uint8_t data1, data2;
    data1 = I2C_read(port, MPU_ADDR, 0x3B);
    data2 = I2C_read(port, MPU_ADDR, 0x3C);
    return (data1 << 8) | data2;
}



int main(void) {
    SystemInit();
    //set_system_clock(); 
    //GPIO_enable(GPIOB, 8, GPIO_OUTPUT);        
    I2C_port i2c1 = {
        .i2c = I2C1,
        .frequency = 16,
        .mode = I2C_STD_MODE,
        .duty = 0,
    };

    USART_port port = {
        .usart = USART2,
        .baud = 115200,
        .mode = USART_RX_TX_MODE,
        .stop_bits = 0,
        .parity_enable = 0,
        .parity_even_odd = 0,
    };

    USART_init(&port);
    I2C_init(&i2c1);

    //USART_init(USART2, 115200, USART_RX_TX_MODE, USART_STOPBITS_1, USART_PARITY_NEN, USART_PARITY_EVEN); 
    //if( EXTI_attach_gpio(GPIOB, 7, EXTI_FALLING_EDGE) != EXTI_OK) {
    //    USART_printf(USART2, "EXTI init failed...\n");
    //}
    uint8_t init[2] = {0x00, 0x00};
    I2C_write_burst(&i2c1, MPU_ADDR, 0x6B, 1, init); 


    //const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    char usart_test[255];
    GPIO_enable(PB8, GPIO_OUTPUT);
    uint64_t cycle = 0; 

    USART_printf(&port, "CCR value: %.12f\n", _I2C_ccr_calc(&i2c1));
    USART_printf(&port, "TRISE value: %.12f\n", _I2C_trise_calc(&i2c1));
    while (1) {
        GPIO_toggle(PB8);
        delayMs(1000);
        //GPIO_toggle(GPIOB, 8);
        //USART_printf(&port, "Cycle: %d\n", cycle++);
        //USART_printf(USART2, "This is a value: %.3f\n", 2.123);
        //if ( USART_has_input(USART2) ) {
        //    USART_scanf(USART2, usart_test);
        //    USART_printf(USART2, "You typed: %s\n", usart_test);
        //}
        USART_printf(&port, "x: %6d\n", read_x(&i2c1));
    }
}




//static void exti_callback(void) {
//    USART_printf(USART2, "Hello from EXIT07\n");
//}
//
//void EXTI9_5_IRQHandler(void) {
//    if (EXTI->PR & (1<<7)) {
//        // Clear PR flag
//        EXTI->PR |= (1<<7);
//        // Do something...
//        exti_callback();
//    }
//}
