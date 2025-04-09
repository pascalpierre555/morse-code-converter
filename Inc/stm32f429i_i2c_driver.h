#ifndef STM32F429_I2C_DRIVER_H
#define STM32F429_I2C_DRIVER_H

#include <stdint.h>

/*
 * Base addresses
 */
#define I2C1                ((I2C_RegDef_t*) I2C1_BASE)

/*
 * I2C Register Map
 */
typedef struct {
    volatile uint32_t CR[2];
    volatile uint32_t OAR[2];
    volatile uint32_t DR;
    volatile uint32_t SR[2];
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_RegDef_t;

/*
 * Function declarations
 */
void I2C1_Init(void);
void I2C1_SendByte(uint8_t slave_address, uint8_t data);

#endif
