/*
 * stm32f429i.h
 *
 *  Created on: Apr 4, 2025
 *      Author: peng
 */

#ifndef STM32F429I_H_
#define STM32F429I_H_

//Bus base addresses
#define APB1BASEADDR	0x40000000U
#define APB2BASEADDR	0x40010000U
#define AHB1BASEADDR	0x40020000U
#define AHB2BASEADDR	0x50000000U
#define AHB3BASEADDR	0xA0000000U

// GPIO base addresses
#define GPIOA_BASE  (AHB1BASEADDR + 0x0000)
#define GPIOB_BASE  (AHB1BASEADDR + 0x0400)
#define GPIOC_BASE  (AHB1BASEADDR + 0x0800)
#define GPIOD_BASE  (AHB1BASEADDR + 0x0C00)
#define GPIOE_BASE  (AHB1BASEADDR + 0x1000)
#define GPIOF_BASE  (AHB1BASEADDR + 0x1400)
#define GPIOG_BASE  (AHB1BASEADDR + 0x1800)
#define GPIOH_BASE  (AHB1BASEADDR + 0x1C00)
#define GPIOI_BASE  (AHB1BASEADDR + 0x2000)
#define GPIOJ_BASE  (AHB1BASEADDR + 0x2400)
#define GPIOK_BASE  (AHB1BASEADDR + 0x2800)

//I2C base addresses
#define I2C1_BASE	(APB1BASEADDR + 0x5400)
#define I2C2_BASE	(APB1BASEADDR + 0x5800)
#define I2C3_BASE	(APB1BASEADDR + 0x5C00)

//I2C clock enable macros
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1<<21))

//I2C clock disable macros
#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1<<21))

//RCC base address
#define RCC_BASE	(AHB1BASEADDR + 0x3800)

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASE)
#define GPIOJ	((GPIO_RegDef_t*)GPIOJ_BASE)
#define GPIOK	((GPIO_RegDef_t*)GPIOK_BASE)

// GPIO Clock Enable Macros
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()		(RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()		(RCC->AHB1ENR |= (1<<10))

// GPIO Clock Disable Macros
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<10))

//GPIO register reset macros
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0))}while (0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1))}while (0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2))}while (0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3))}while (0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4))}while (0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5))}while (0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6))}while (0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7))}while (0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8))}while (0)
#define GPIOJ_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1<<9))}while (0)
#define GPIOK_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<10)); (RCC->AHB1RSTR &= ~(1<<10))}while (0)

//RCC
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
} RCC_RegDef_t;

#define RCC	((RCC_RegDef_t*)RCC_BASE)

#define ENABLE 1
#define DISABLE 0

#endif /* STM32F429I_H_ */
