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
gpio_err_t GPIO_enable(const gpio_pin_t pin, gpio_mode_t mode) {
    if ( pin > PH15 ) {
        return GPIO_PIN_TOO_HIGH;
    }
    GPIO_TypeDef *port;

    if ( pin < PB0 ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        port = GPIOA;
    } else if ( pin < PC0 ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        port = GPIOB;
    } else if ( pin < PD0 ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        port = GPIOC;
    } else if ( pin < PE0) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        port = GPIOD;
    } else if ( pin < PH0 ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        port = GPIOE;
    } else if ( pin <= PH15 ) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        port = GPIOH;
    }

    if (mode == GPIO_ANALOG) {
        // TODO: analog init
    }

    /* Reset the old mode */
    port->MODER &= ~(3 << ((pin % PINS_PER_PORT) * 2)); 
    /* Set the new mode */
    port->MODER |= (mode << ((pin % PINS_PER_PORT) * 2));
    return GPIO_OK;
}


gpio_err_t GPIO_select_alternate(const gpio_pin_t pin, const uint8_t af) {
    if ( pin > PH15 ) {
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
    uint8_t raw_pin = pin % PINS_PER_PORT;
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return GPIO_PIN_TOO_HIGH;
    }
    if ( raw_pin <= 7 ){
        port->AFR[0] &= ~(15 << (raw_pin * 4));
        port->AFR[0] |= (af << (raw_pin * 4));
    } else {
        port->AFR[1] &= ~(15 << (raw_pin * 4));
        port->AFR[1] |= (af << (raw_pin * 4));
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
gpio_err_t GPIO_settings(const gpio_pin_t pin, const uint8_t speed, const uint8_t pull_up_down, const uint8_t push_pull_open_drain) {
    if (pin > PH15) {
        return GPIO_PIN_TOO_HIGH;
    }
    
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return GPIO_PIN_TOO_HIGH;
    }
    
    port->OSPEEDR &=    ~(3 << ((pin & PINS_PER_PORT)*2));
    port->OSPEEDR |=    (speed << ((pin % PINS_PER_PORT)*2));

    port->OTYPER &=     ~(1 << (pin % PINS_PER_PORT));
    port->OTYPER |=     (push_pull_open_drain << (pin % PINS_PER_PORT));
    
    port->PUPDR &=      ~(3 << ((pin % PINS_PER_PORT)*2));
    port->PUPDR |=      (pull_up_down << ((pin % PINS_PER_PORT)*2));
    
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
gpio_err_t GPIO_toggle(const gpio_pin_t pin) {
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return GPIO_PIN_TOO_HIGH;
    }
    port->ODR ^= (1 << (pin % PINS_PER_PORT));
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
gpio_err_t GPIO_write(const gpio_pin_t pin, const uint8_t on_off) {
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return GPIO_PIN_TOO_HIGH;
    }
    if (on_off == GPIO_OFF) {
        port->BSRR |= (1 << ((pin % PINS_PER_PORT)+16));
    } else {
        port->BSRR |= (1 << (pin % PINS_PER_PORT));
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
uint8_t GPIO_read_digital(const gpio_pin_t pin) {
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return 0;
    }
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
uint16_t GPIO_read_analog(const gpio_pin_t pin) {
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
gpio_err_t GPIO_lock(const gpio_pin_t pin) {
    if ( pin > PH15 ) {
        return GPIO_PIN_TOO_HIGH;
    }
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return GPIO_PIN_TOO_HIGH;
    }
    port->LCKR |= (1 << (pin % PINS_PER_PORT)); 
    return GPIO_OK;
}


GPIO_TypeDef *_GPIO_fetch_port(const gpio_pin_t pin) {
    GPIO_TypeDef *port = NULL;
    

    if ( pin < PB0 ) {
        port = GPIOA;
    } else if ( pin < PC0 ) {
        port = GPIOB;
    } else if ( pin < PD0 ) {
        port = GPIOC;
    } else if ( pin < PE0) {
        port = GPIOD;
    } else if ( pin < PH0 ) {
        port = GPIOE;
    } else if ( pin <= PH15 ) {
        port = GPIOH;
    }
    return port; 
}
