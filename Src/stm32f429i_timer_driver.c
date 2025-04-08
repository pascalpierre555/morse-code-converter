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
    uint8_t temp1 = channel / 2;
    uint8_t temp2 = channel % 2;
    // Configure the channel
    pTIMHandle->pTIMx->CCMR[temp1] &= ~(0x03 << (4 * temp2)); // Clear the bits
    pTIMHandle->pTIMx->CCMR[temp1] |= (pTIMHandle->config.channelConfig[channel].ccm << temp2); // Set Capture/Compare mode
    pTIMHandle->pTIMx->CCER &= ~(0x06 << (4 * channel)); // Clear the CCxP bits
    pTIMHandle->pTIMx->CCER |= (pTIMHandle->config.channelConfig[channel].ic_mode << (4 * channel)); // Set Input Capture mode
    pTIMHandle->pTIMx->CCER |= (1 << (4 * channel)); // Enable the channel
}

uint32_t TIM_GetCCRValue(TIM_Handle_t *pTIMHandle, uint32_t channel) {
    // Read the Capture/Compare register value
    return pTIMHandle->pTIMx->CCR[channel];
}

// void TIM2_IRQHandling(TIM_Handle_t *pTIMHandle) {
//     // Check if the interrupt flag is set
//     if (pTIMHandle->pTIMx->SR & TIM_FLAG_CC1IF) {
//         // Clear the interrupt flag
//         pTIMHandle->pTIMx->SR &= ~TIM_FLAG_CC1IF;
//         // Handle the interrupt (e.g., read captured value)
//     }
// }

void TIM_EnableInterrupt(TIM_Handle_t *pTIMHandle, uint32_t channel) {
    pTIMHandle->pTIMx->DIER |= (1 << (channel + 1)); // Enable interrupt for the specified channel
}

void TIM_DisableInterrupt(TIM_Handle_t *pTIMHandle, uint32_t channel) {
    pTIMHandle->pTIMx->DIER &= ~(1 << (channel + 1)); // Disable interrupt for the specified channel
}
