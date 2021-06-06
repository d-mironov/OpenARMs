#include "exti.h"
#include "../gpio/gpio.h"
#include <stm32f4xx.h>



exti_err_t EXTI_select_trigger(uint32_t lines, exti_trigger_t trigger) {
    if (lines & EXTI_RESERVED) {
        return EXTI_LINES_RESERVED;
    }

    switch (trigger) {
        case EXTI_RISING_EDGE:
            EXTI->RTSR &= ~(lines);
            EXTI->RTSR |= lines;
            break;
        case EXTI_FALLING_EDGE:
            EXTI->FTSR &= ~(lines);
            EXTI->FTSR |= lines;
            break;
        default:
            EXTI->RTSR &= ~(lines);
            EXTI->FTSR &= ~(lines);
            EXTI->RTSR |= lines;
            EXTI->FTSR |= lines;
            break;
    }

    return EXTI_OK;
}



exti_err_t EXTI_unmask(uint32_t lines) {
    if (lines & EXTI_RESERVED) {
        return EXTI_LINES_RESERVED;
    }

    EXTI->IMR |= lines;

    return EXTI_OK;
}


exti_err_t EXTI_attach_gpio(GPIO_TypeDef *port, uint8_t pin, exti_trigger_t trigger) {
    if (pin > 15) {
        return EXTI_PIN_TOO_HIGH;
    }

    __disable_irq();
    
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    GPIO_enable(port, pin, GPIO_INPUT);

    // uint8_t cr = (pin / 4) % 4;
    uint8_t exti_port = ((uint64_t) port - AHB1PERIPH_BASE) / 0x00400UL;
    // Calculations for port and pin on the EXTI line selection
    SYSCFG->EXTICR[(pin/4) % 4] |= (exti_port << ((pin % SYSCFG_EXTI_PORTS_PER_REG) * SYSCFG_EXTI_BITNUM)); 

    if ( EXTI_select_trigger((1<<pin), trigger) != EXTI_OK ) {
        __enable_irq();
        return EXTI_LINES_RESERVED;
    }

    if ( EXTI_unmask( (1 << pin) ) != EXTI_OK) {
        __enable_irq();
        return EXTI_LINES_RESERVED;
    }

    // TODO: enable specific IRQ in NVIC
    __enable_irq();
    return EXTI_OK;
}
