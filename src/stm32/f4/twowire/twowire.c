#include "twowire.h"
#include "../gpio/gpio.h"
#include <stm32f4xx.h>


twowire_err_t I2C_init(I2C_TypeDef *I2Cx) {
    if (I2Cx == I2C1) {
        // enabling GPIO 6,7, selecting alternate function, setting speed
        GPIO_enable(GPIOB, 6, GPIO_ALTERNATE);
        GPIO_enable(GPIOB, 7, GPIO_ALTERNATE);
        GPIO_settings(GPIOB, 6, GPIO_HIGH_SPEED, GPIO_PULL_UP);
        GPIO_settings(GPIOB, 7, GPIO_HIGH_SPEED, GPIO_PULL_UP);
        GPIO_select_alternate(GPIOB, 6, GPIO_AF04);
        GPIO_select_alternate(GPIOB, 7, GPIO_AF04);  
    } else if (I2Cx == I2C2) {
        GPIO_enable(GPIOB, 10, GPIO_ALTERNATE);
        GPIO_enable(GPIOB, 11, GPIO_ALTERNATE);
        GPIO_settings(GPIOB, 10, GPIO_HIGH_SPEED, GPIO_PULL_UP);
        GPIO_settings(GPIOB, 11, GPIO_HIGH_SPEED, GPIO_PULL_UP);
        GPIO_select_alternate(GPIOB, 10, GPIO_AF04);
        GPIO_select_alternate(GPIOB, 11, GPIO_AF04);  
    } else if (I2Cx == I2C3) {
        GPIO_enable(GPIOA, 8, GPIO_ALTERNATE);
        GPIO_enable(GPIOC, 9, GPIO_ALTERNATE);
        GPIO_settings(GPIOA, 8, GPIO_HIGH_SPEED, GPIO_PULL_UP);
        GPIO_settings(GPIOC, 9, GPIO_HIGH_SPEED, GPIO_PULL_UP);
        GPIO_select_alternate(GPIOA, 8, GPIO_AF04);
        GPIO_select_alternate(GPIOC, 9, GPIO_AF04);  
    } else {
        return I2C_PORT_NOT_AVAILABLE;
    }
    
    return I2C_Ok;
}
