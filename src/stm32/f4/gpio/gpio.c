#include "gpio.h"
#include <stm32f4xx.h>


void ADC1_enable(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
}


/**
 * GPIO enable function
 *
 * enable and select mode on selected GPIO Port and Pin
 *
 * @param `port` - GPIO Port of Pin
 * @param `pin_num` - Pin on the Port
 * @param `mode` - mode of Port
 *
 * @return GPIO_OK on success, GPIO_PIN_TOO_HIGH on `pin_num`>15
 */
gpio_err_t GPIO_enable(GPIO_TypeDef *port, const uint8_t pin_num, gpio_mode_t mode) {
    if ( pin_num > 15 ) {
        return GPIO_PIN_TOO_HIGH;
    }

    if ( GPIOA == port ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if ( GPIOB == port ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if ( GPIOC == port ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if ( GPIOD == port ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else if ( GPIOE == port ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    } else if ( GPIOH == port ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    }

    if (mode == GPIO_ANALOG) {
        // TODO: analog init
    }

    /* Reset the old mode */
    port->MODER &= ~(3 << (pin_num * 2)); 
    /* Set the new mode */
    port->MODER |= (mode << (pin_num * 2));
    return GPIO_OK;
}


gpio_err_t GPIO_select_alternate(GPIO_TypeDef *port, const uint8_t pin, const uint8_t af) {
    if ( pin > 15 ) {
        return GPIO_PIN_TOO_HIGH;
    }
    if (af > 15) {
        return GPIO_ALTERNATE_FUNC_TOO_HIGH;
    }

    /*
    if (!((2<<(2*pin)) & port->MODER)) {
        return GPIO_ALTERNATE_NOT_SELECTED;
    }
    */

    if ( pin <= 7 ){
        port->AFR[0] &= ~(15 << (pin * 4));
        port->AFR[0] |= (af << (pin * 4));
    } else {
        port->AFR[1] &= ~(15 << (pin * 4));
        port->AFR[1] |= (af << (pin * 4));
    }

    return GPIO_OK;
}

/**
 * GPIO settings function
 *
 * Set Speed and select Pull-Up,-Down 
 *
 * @param `port` - Port of Pin to do settings
 * @param `pin` - Pin to do settings
 * @param `speed` - Speed of pin (GPIO_LOW_SPEED, GPIO_MEDIUM_SPEED, GPIO_FAST_SPEED, GPIO_HIGH_SPEED)
 * @param `pull_up_down` - Pull-up,-down settings (GPIO_NO_PULL_UP_DOWN, GPIO_PULL_UP, GPIO_PULL_DOWN)
 *
 * @return GPIO_OK on success, GPIO_PIN_TOO_HIGH when `pin`>15
 */
gpio_err_t GPIO_settings(GPIO_TypeDef *port, const uint8_t pin, const uint8_t speed, const uint8_t pull_up_down) {
    if (pin > 15) {
        return GPIO_PIN_TOO_HIGH;
    }
    port->OSPEEDR &=    (3 << (pin*2));
    port->OSPEEDR |=    (speed << (pin*2));
    
    port->PUPDR &=      (3 << (pin*2));
    port->PUPDR |=      (pull_up_down << (pin*2));
    
    return GPIO_OK;
}

/**
 * GPIO Toggle function
 *
 * Toggle Output of GPIO pin
 *
 * @param `port` - Port of Pin to toggle
 * @param `pin` - Pin to toggle
 *
 * @return GPIO_OK
 */
gpio_err_t GPIO_toggle(GPIO_TypeDef *port, const uint8_t pin) {
    port->ODR ^= (1 << pin);
    return GPIO_OK;
}

/**
 * GPIO Write function
 *
 * Write the specified value to the GPIO (GPIO_ON, GPIO_OFF)
 *
 * @param `port` - Port of pin to write
 * @param `pin` - Pin to write
 * @param `on_off` - value to write (GPIO_ON, GPIO_OFF)
 *
 * @return GPIO_OK
 */
gpio_err_t GPIO_write(GPIO_TypeDef *port, const uint8_t pin, const uint8_t on_off) {
    if (on_off == GPIO_OFF) {
        port->BSRR |= (1 << (pin+16));
    } else {
        port->BSRR |= (1 << pin);
    }
    return GPIO_OK;
}

/**
 * GPIO digital read function
 *
 * Read GPIOx_IDR register of specified Pin
 *
 * @param `port` - Port to of pin to read
 * @param `pin` - Pin to read
 *
 * @return digital value of pin
 */
uint8_t GPIO_read_digital(GPIO_TypeDef *port, const uint8_t pin) {
    return (port->IDR & (1 << pin));
}

/**
 * GPIO analog read function
 *
 * Read analog value of GPIO pin
 *
 * @param `port` - Port of pin to read
 * @param `pin` - Pin to read
 *
 * @return analog value of Pin
 */
uint16_t GPIO_read_analog(GPIO_TypeDef *port, const uint8_t pin) {
    // TODO
}

/**
 * Lock GPIO configuration 
 *
 * @param port - Port of GPIO
 * @param pin - pin to lock
 *
 * @return GPIO_PIN_TOO_HIGH if port is too high, GPIO_OK on success
 */
gpio_err_t GPIO_lock(GPIO_TypeDef *port, const uint8_t pin) {
    if ( pin > 15 ) {
        return GPIO_PIN_TOO_HIGH;
    }
    port->LCKR |= (1 << pin); 
    return GPIO_OK;
}

