/*
 * stm32f429i_timer_driver.h
 *
 *  Created on: Apr 7, 2025
 *      Author: peng
 */

#ifndef STM32F429I_TIMER_DRIVER_H_
#define STM32F429I_TIMER_DRIVER_H_

#include <stm32f429i.h>

//capture/comapre channels
#define TIM_CC1    0
#define TIM_CC2    1
#define TIM_CC3    2
#define TIM_CC4    3

//define capture/compare selection
#define TIM_CC_SELECTION_OUTPUT         0
#define TIM_CC_SELECTION_INPUT_DEFAULT  1
#define TIM_CC_SELECTION_INPUT_TOGGLE   2
#define TIM_CC_SELECTION_INPUT_TRC      3

//input capture modes
#define TIM_IC_MODE_RISING_EDGE   0
#define TIM_IC_MODE_FALLING_EDGE  1
#define TIM_IC_MODE_RF            3

typedef struct {
    uint32_t prescaler; // Prescaler value
    uint32_t period;    // Auto-reload value
    uint32_t mode;      // Timer mode (e.g., up, down, etc.)
    uint32_t ccm;       // Capture/Compare mode
    TIM_ChannelConfig_t channelConfig[4]; // Channel configurations
} TIM_Config_t;

typedef struct {
	volatile uint32_t CR[2];
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR[2];
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	uint32_t RESERVED1;
	volatile uint32_t CCR[4];
	uint32_t RESERVED2;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t OR;
} TIM_RegDef_t;

typedef struct {
    uint32_t channel;    // Timer channel
    uint32_t ccm;        // Capture/Compare mode
    uint32_t ic_mode;    // Input capture mode
} TIM_ChannelConfig_t;

typedef struct {
    TIM_RegDef_t *pTIMx; // Pointer to the timer peripheral
    TIM_Config_t config; // Timer configuration
} TIM_Handle_t;

void TIM_Init(TIM_Handle_t *pTIMHandle);
void TIM_Start(TIM_Handle_t *pTIMHandle);
void TIM_Stop(TIM_Handle_t *pTIMHandle);
void TIM_SetPrescaler(TIM_Handle_t *pTIMHandle, uint32_t prescaler);

#endif /* STM32F429I_TIMER_DRIVER_H_ */
