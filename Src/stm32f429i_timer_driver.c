/*
 * stm32f429i_timer_driver.c
 *
 *  Created on: Apr 7, 2025
 *      Author: peng
 */

#include "stm32f429i.h"
#include "stm32f429i_timer_driver.h"

void TIM_Init(TIM_Handle_t *pTIMHandle) {
    // Enable the peripheral clock for the timer
    TIM2_PCLK_EN(); // Enable TIM2 clock

    // Configure the timer
    pTIMHandle->pTIMx->PSC = pTIMHandle->config.prescaler - 1; // Set prescaler
    pTIMHandle->pTIMx->ARR = pTIMHandle->config.period - 1;    // Set auto-reload value
}

void TIM_DeInit(TIM_Handle_t *pTIMHandle) {
    // Disable the peripheral clock for the timer
    TIM2_PCLK_DI(); // Disable TIM2 clock
}

void TIM_SetPrescaler(TIM_Handle_t *pTIMHandle, uint32_t prescaler) {
    pTIMHandle->pTIMx->PSC = prescaler - 1; // Set prescaler
}

void TIM_Start(TIM_Handle_t *pTIMHandle) {
    pTIMHandle->pTIMx->CR[0] |= (1 << 0); // Enable the timer
}

void TIM_Stop(TIM_Handle_t *pTIMHandle) {
    pTIMHandle->pTIMx->CR[0] &= ~(1 << 0); // Disable the timer
}
void TIM_SetupChannel(TIM_Handle_t *pTIMHandle, uint32_t channel) {
    // Configure the channel
    pTIMHandle->pTIMx->CCMR[channel] &= ~(0x03 << (8 * channel)); // Clear the bits
    pTIMHandle->pTIMx->CCMR[channel] |= (pTIMHandle->config.channelConfig[channel].ccm << (8 * channel)); // Set Capture/Compare mode
    pTIMHandle->pTIMx->CCER &= ~(0x03 << (4 * channel) + 1); // Clear the CCxP bits
    pTIMHandle->pTIMx->CCER |= (pTIMHandle->config.channelConfig[channel].ic_mode << (4 * channel) + 1); // Set Input Capture mode
    pTIMHandle->pTIMx->CCER |= (1 << (4 * channel)); // Enable the channel
}