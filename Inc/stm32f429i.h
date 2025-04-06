/*
 * stm32f429i.h
 *
 *  Created on: Apr 6, 2025
 *      Author: peng
 */

#ifndef STM32F429I_H_
#define STM32F429I_H_

#include <stdint.h>

// ARM Cortex-M4 core NVIC ISERx register addresses
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)

// ARM Cortex-M4 core NVIC ICERx register addresses
#define NVIC_ICER0		((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*)0xE000E188)

// ARM Cortex-M4 core priority register addresses
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4

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

#define EXTI_BASE	(APB2BASEADDR + 0x3C00)

//I2C clock enable macros
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1<<21))

//I2C clock disable macros
#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1<<21))

//RCC base address
#define RCC_BASE	(AHB1BASEADDR + 0x3800)

//SYSCFG base address
#define SYSCFG_BASE	(APB2BASEADDR + 0x3800)

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

//SYSCFG Clock Enable Macros
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<<14))

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

//STSCFG CLock Disable Macros
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<<14))

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

//GPIO base address to port code conversion macros
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : \
									(x == GPIOI) ? 8 : \
									(x == GPIOJ) ? 9 : \
									10)

//RCC
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
} RCC_RegDef_t;

//EXTI
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegDef_t;

//IRQ number macros
#define IRQ_NO_EXTI0	6
#define IRQ_NO_EXTI1	7
#define IRQ_NO_EXTI2	8
#define IRQ_NO_EXTI3	9
#define IRQ_NO_EXTI4	10
#define IRQ_NO_EXTI5	23
#define IRQ_NO_EXTI6	23
#define IRQ_NO_EXTI7	23
#define IRQ_NO_EXTI8	23
#define IRQ_NO_EXTI9_5	23
#define IRQ_NO_EXTI10_15	40

//IRQ priority macros
#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI1	1
#define NVIC_IRQ_PRI2	2
#define NVIC_IRQ_PRI3	3
#define NVIC_IRQ_PRI4	4
#define NVIC_IRQ_PRI5	5
#define NVIC_IRQ_PRI6	6
#define NVIC_IRQ_PRI7	7
#define NVIC_IRQ_PRI8	8
#define NVIC_IRQ_PRI9	9
#define NVIC_IRQ_PRI10	10
#define NVIC_IRQ_PRI11	11
#define NVIC_IRQ_PRI12	12
#define NVIC_IRQ_PRI13	13
#define NVIC_IRQ_PRI14	14
#define NVIC_IRQ_PRI15	15

//SYSCFG
typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;

#define RCC	((RCC_RegDef_t*)RCC_BASE)
#define EXTI	((EXTI_RegDef_t*)EXTI_BASE)
#define SYSCFG	((SYSCFG_RegDef_t*)(SYSCFG_BASE))

#define ENABLE 1
#define DISABLE 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0

#endif /* STM32F429I_H_ */
