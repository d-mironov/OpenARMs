/*
 * TwoWire interface for STM32F4xx boards
 *
 * This will support both SMBus and I2C
 * 
 *
 * changelog:
 * |_____v0.1
 * |       |_____
 *
 * @author: cosmicraccoon (aka Daniel Mironow)
 * @version: v0.1
 *
 */
#ifndef _TWOWIRE_H
#define _TWOWIRE_H

#include <stm32f4xx.h>


typedef enum _twowire_err_h {
    I2C_Ok,
    I2C_PORT_NOT_AVAILABLE
} twowire_err_t;


twowire_err_t I2C_Init(I2C_TypeDef *I2Cx);
uint8_t I2C_read(I2C_TypeDef *I2Cx, uint8_t slave, uint8_t memaddr);
uint8_t I2C_read_burst(I2C_TypeDef *I2Cx, uint8_t slave, uint8_t memaddr, uint8_t n, uint8_t *data);
void I2C_write(I2C_TypeDef *I2Cx, uint8_t slave, uint8_t memaddr, uint8_t data);
void I2C_write_burst(I2C_TypeDef *I2Cx, uint8_t slave, uint8_t memaddr, uint8_t n, uint8_t *data);



#endif
