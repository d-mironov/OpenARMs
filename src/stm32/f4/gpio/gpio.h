#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <stm32f4xx.h>

/* Values to write to GPIO */
#define GPIO_ON     0x01
#define GPIO_OFF    0x00

/* Speeds of GPIO */
#define GPIO_LOW_SPEED      0x00
#define GPIO_MEDIUM_SPEED   0x01
#define GPIO_FAST_SPEED     0x02
#define GPIO_HIGH_SPEED     0x03

/* Pull up/down of GPIO */
#define GPIO_NO_PULL_UP_DOWN    0x00
#define GPIO_PULL_UP            0x01
#define GPIO_PULL_DOWN          0x02

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




void ADC1_enable(void);
gpio_err_t GPIO_enable(GPIO_TypeDef *port, const uint8_t pin_num, gpio_mode_t mode);
gpio_err_t GPIO_settings(GPIO_TypeDef *port, const uint8_t pin_num, const uint8_t speed, const uint8_t pull_up_down);

gpio_err_t GPIO_toggle(GPIO_TypeDef *port, const uint8_t pin_num);
gpio_err_t GPIO_write(GPIO_TypeDef *port, const uint8_t pin_num, const uint8_t on_off);
uint8_t GPIO_read_digital(GPIO_TypeDef *port, const uint8_t pin);
uint16_t GPIO_read_analog(GPIO_TypeDef *port, const uint8_t pin);
gpio_err_t GPIO_select_alternate(GPIO_TypeDef *port, const uint8_t pin, const uint8_t af);




#endif
