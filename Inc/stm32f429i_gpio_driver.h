/*
 * stm32f429i_gpio_driver.h
 *
 *  Created on: Apr 6, 2025
 *      Author: peng
 */

#ifndef STM32F429I_GPIO_DRIVER_H_
#define STM32F429I_GPIO_DRIVER_H_

/*
 * gpio_driver.h
 *
 *  Created on: Apr 4, 2025
 *      Author: peng
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include <stm32f429i.h>
#include <stdint.h>

// GPIO 暫存器位址偏移量
#define GPIO_MODER_OFFSET	(0x00)
#define GPIO_OTYPER_OFFSET  (0x04)
#define GPIO_OSPEEDR_OFFSET (0x08)
#define GPIO_PUPDR_OFFSET  	(0x0C)
#define GPIO_IDR_OFFSET   	(0x10)
#define GPIO_ODR_OFFSET    	(0x14)
#define GPIO_BSRR_OFFSET  	(0x18)
#define GPIO_LCKR_OFFSET  	(0x1C)
#define GPIO_AFRL_OFFSET   	(0x20)
#define GPIO_AFRH_OFFSET   	(0x24)

// RCC AHB1ENR 暫存器位址偏移量
#define RCC_AHB1ENR_OFFSET (0x30)

//GPIO
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct {
    uint32_t pin;
    uint32_t mode;
    uint32_t otype;
    uint32_t ospeed;
    uint32_t pupd;
    uint32_t af;
} GPIO_Config_t;

// GPIO 結構定義
typedef struct {
    GPIO_RegDef_t *port;
    GPIO_Config_t config;
} GPIO_Handle_t;

// GPIO 模式定義
#define GPIO_MODE_INPUT     (0x00)
#define GPIO_MODE_OUTPUT    (0x01)
#define GPIO_MODE_AF        (0x02)
#define GPIO_MODE_ANALOG    (0x03)
#define GPIO_MODE_IT_FT     (0x04) // Interrupt on falling edge
#define GPIO_MODE_IT_RT     (0x05) // Interrupt on rising edge
#define GPIO_MODE_IT_RFT    (0x06) // Interrupt on rising and falling edge

// GPIO 輸出類型定義
#define GPIO_OTYPE_PP       (0x00) // Push-pull
#define GPIO_OTYPE_OD       (0x01) // Open-drain

// GPIO 輸出速度定義
#define GPIO_OSPEED_LOW     (0x00)
#define GPIO_OSPEED_MEDIUM  (0x01)
#define GPIO_OSPEED_HIGH    (0x02)
#define GPIO_OSPEED_VERY_HIGH (0x03)

// GPIO 上拉/下拉定義
#define GPIO_PUPD_NONE      (0x00)
#define GPIO_PUPD_UP        (0x01)
#define GPIO_PUPD_DOWN      (0x02)

//GPIO 引腳定義
#define GPIO_PIN_0         (0x0000)
#define GPIO_PIN_1         (0x0001)
#define GPIO_PIN_2         (0x0002)
#define GPIO_PIN_3         (0x0003)
#define GPIO_PIN_4         (0x0004)
#define GPIO_PIN_5         (0x0005)	
#define GPIO_PIN_6         (0x0006)
#define GPIO_PIN_7         (0x0007)
#define GPIO_PIN_8         (0x0008)
#define GPIO_PIN_9         (0x0009)
#define GPIO_PIN_10        (0x000A)
#define GPIO_PIN_11        (0x000B)
#define GPIO_PIN_12        (0x000C)
#define GPIO_PIN_13        (0x000D)
#define GPIO_PIN_14        (0x000E)
#define GPIO_PIN_15        (0x000F)

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* GPIO_DRIVER_H_ */

#endif /* STM32F429I_GPIO_DRIVER_H_ */
