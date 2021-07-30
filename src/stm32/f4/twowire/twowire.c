#include "twowire.h"
#include "../gpio/gpio.h"
#include <stm32f4xx.h>


twowire_err_t I2C_init(I2C_TypeDef *I2Cx) {
    if (I2Cx == I2C1) {
        // enabling GPIO 6,7, selecting alternate function, setting speed
        GPIO_enable(PB6, GPIO_ALTERNATE);
        GPIO_enable(PB7, GPIO_ALTERNATE);
        GPIO_settings(PB6, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_settings(PB7, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_select_alternate(PB6, GPIO_AF04);
        GPIO_select_alternate(PB7, GPIO_AF04);  
    } else if (I2Cx == I2C2) {
        GPIO_enable(PB10, GPIO_ALTERNATE);
        GPIO_enable(PB11, GPIO_ALTERNATE);
        GPIO_settings(PB10, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_settings(PB11, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_select_alternate(PB10, GPIO_AF04);
        GPIO_select_alternate(PB11, GPIO_AF04);  
    } else if (I2Cx == I2C3) {
        GPIO_enable(PB8, GPIO_ALTERNATE);
        GPIO_enable(PC9, GPIO_ALTERNATE);
        GPIO_settings(PA8, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_settings(PC9, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        GPIO_select_alternate(PA8, GPIO_AF04);
        GPIO_select_alternate(PC9, GPIO_AF04);  
    } else {
        return I2C_PORT_NOT_AVAILABLE;
    }
    
    return I2C_Ok;
}
