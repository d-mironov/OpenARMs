#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <stm32f4xx.h>

/* Values to write to GPIO */
#define GPIO_ON     0x01
#define GPIO_OFF    0x00

// How many pins there are per port
#define PINS_PER_PORT   0x0F

/* Speeds of GPIO */
#define GPIO_LOW_SPEED      0x00
#define GPIO_MEDIUM_SPEED   0x01
#define GPIO_FAST_SPEED     0x02
#define GPIO_HIGH_SPEED     0x03

/* Pull up/down of GPIO */
#define GPIO_NO_PULL_UP_DOWN    0x00
#define GPIO_PULL_UP            0x01
#define GPIO_PULL_DOWN          0x02

// Alternate function selection of GPIO
#define GPIO_AF00       0x00
#define GPIO_AF01       0x01
#define GPIO_AF02       0x02
#define GPIO_AF03       0x03
#define GPIO_AF04       0x04
#define GPIO_AF05       0x05
#define GPIO_AF06       0x06
#define GPIO_AF07       0x07
#define GPIO_AF08       0x08
#define GPIO_AF09       0x09
#define GPIO_AF10       0x0A
#define GPIO_AF11       0x0B
#define GPIO_AF12       0x0C
#define GPIO_AF13       0x0D
#define GPIO_AF14       0x0E
#define GPIO_AF15       0x0F

#define ADC_PA0     0x00
#define ADC_PA1     0x01
#define ADC_PA2     0x02
#define ADC_PA3     0x03
#define ADC_PA4     0x04
#define ADC_PA5     0x05
#define ADC_PA6     0x06
#define ADC_PA7     0x07
#define ADC_PB0     0x08
#define ADC_PB1     0x09
#define ADC_PC0     0x0A
#define ADC_PC1     0x0B
#define ADC_PC2     0x0C
#define ADC_PC3     0x0D
#define ADC_PC4     0x0E
#define ADC_PC5     0x0F




typedef enum gpio_mode {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_ALTERNATE,
    GPIO_ANALOG
} gpio_mode_t;

typedef enum gpio_err {
    GPIO_OK,
    GPIO_PIN_TOO_HIGH,
    GPIO_ALTERNATE_FUNC_TOO_HIGH,
    GPIO_ALTERNATE_NOT_SELECTED
} gpio_err_t;

typedef enum _gpio_pin_t {
    PA0,PA1,PA2,PA3,PA4,PA5,PA6,PA7,
    PA8,PA9,PA10,PA11,PA12,PA13,PA14,PA15,
    PB0,PB1,PB2,PB3,PB4,PB5,PB6,PB7,
    PB8,PB9,PB10,PB11,PB12,PB13,PB14,PB15,
    PC0,PC1,PC2,PC3,PC4,PC5,PC6,PC7,
    PC8,PC9,PC10,PC11,PC12,PC13,PC14,PC15,
    PD0,PD1,PD2,PD3,PD4,PD5,PD6,PD7,
    PD8,PD9,PD10,PD11,PD12,PD13,PD14,PD15 
} gpio_pin_t;




gpio_err_t GPIO_enable(GPIO_TypeDef *port, const uint8_t pin_num, gpio_mode_t mode);
gpio_err_t GPIO_settings(GPIO_TypeDef *port, const uint8_t pin_num, const uint8_t speed, const uint8_t pull_up_down);

gpio_err_t GPIO_toggle(GPIO_TypeDef *port, const uint8_t pin_num);
gpio_err_t GPIO_write(GPIO_TypeDef *port, const uint8_t pin_num, const uint8_t on_off);
uint8_t GPIO_read_digital(GPIO_TypeDef *port, const uint8_t pin);
uint16_t GPIO_read_analog(GPIO_TypeDef *port, const uint8_t pin);
gpio_err_t GPIO_select_alternate(GPIO_TypeDef *port, const uint8_t pin, const uint8_t af);

gpio_err_t GPIO_lock(GPIO_TypeDef *port, const uint8_t pin);



#endif
