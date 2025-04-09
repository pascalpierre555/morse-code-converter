/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f429i.h"
#include "stm32f429i_gpio_driver.h"
#include "stm32f429i_timer_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

uint32_t press_duration = 0; // Variable to store the duration of the button press
uint32_t start_time[2] = {0}; // Variable to store the press time
uint32_t end_time[2] = {0}; // Variable to store the release time

void GPIO_InitConfig(GPIO_Handle_t *GpioBtn, GPIO_Handle_t *GpioLED1, GPIO_Handle_t *GpioLED2) {
	GPIOA_PCLK_EN();
    // Initialize GPIOA as alternate function mode for TIM2_CH1
	GpioBtn->port = GPIOA;
	GpioBtn->config.pin = 0;
	GpioBtn->config.mode = GPIO_MODE_AF; // Falling edge trigger
	GpioBtn->config.otype = GPIO_OTYPE_PP; // Push-pull
	GpioBtn->config.ospeed = GPIO_OSPEED_LOW; // Fast speed
	GpioBtn->config.pupd = GPIO_PUPD_UP; // Pull-up
	GpioBtn->config.af = 1; // Alternate function 1 (AF1)
	GPIO_Init(GpioBtn);

    // 初始化 GPIO2 和 GPIO3 為輸出模式
    GpioLED1->port = GPIOA;
    GpioLED1->config.pin = 4; // GPIO2
    GpioLED1->config.mode = GPIO_MODE_OUTPUT;
    GpioLED1->config.otype = GPIO_OTYPE_PP;
    GpioLED1->config.ospeed = GPIO_OSPEED_LOW;
	GpioLED1->config.pupd = GPIO_PUPD_NONE;
    GPIO_Init(GpioLED1);

    GpioLED2->port = GPIOA;
    GpioLED2->config.pin = 5; // GPIO3
    GpioLED2->config.mode = GPIO_MODE_OUTPUT;
    GpioLED2->config.otype = GPIO_OTYPE_PP;
    GpioLED2->config.ospeed = GPIO_OSPEED_LOW;
    GpioLED2->config.pupd = GPIO_PUPD_NONE;
    GPIO_Init(GpioLED2);
}

// TIM2 configuration
void TIM_Config(TIM_Handle_t *TIMBtn, TIM_Handle_t *TIMLED1, TIM_Handle_t *TIMLED2) {
	// Initialize TIM2 for TIMBtn
	TIMBtn->pTIMx = TIM2;
	TIMBtn->config.prescaler = 16000; // Prescaler value
	TIMBtn->config.period = 0xFFFF; // Auto-reload value
	TIMBtn->config.mode = 0; // Timer mode (up)
	TIMBtn->config.channelConfig[0].ccm = TIM_CC_SELECTION_INPUT_DEFAULT; // 默認輸入捕捉
	TIMBtn->config.channelConfig[0].ic_mode = TIM_IC_MODE_RF; // 捕捉上升沿
	TIM_Init(TIMBtn); // Initialize TIM2
	TIM2_PCLK_EN();

	// Initialize TIM3 for LED1
	TIMLED1->pTIMx = TIM3;
	TIM3->ARR = 600;
	TIM3->PSC = 16000 - 1; // Prescaler value
	TIM3->CR[0] = 0; // Timer mode (up)
	TIM3_PCLK_EN();

	// Initialize TIM4 for LED2
	TIMLED2->pTIMx = TIM4;
	TIMLED2->config.prescaler = 16000; // Prescaler value
	TIMLED2->config.period = 1200; // Auto-reload value
	TIMLED2->config.mode = 0;
	TIM_Init(TIMLED2); // Initialize TIM4
	TIM4_PCLK_EN();

	TIM_SetupChannel(TIMBtn, TIM_CC1); // Setup input channel 0

	TIM_EnableInterrupt(TIMBtn, TIM_CC1); // Enable interrupt for channel 0
	TIMLED1->pTIMx->DIER |= (1 << 0); // Enable interrupt for TIM3
	TIMLED2->pTIMx->DIER |= (1 << 0); // Enable interrupt for TIM4
	TIM_Start(TIMBtn); // Start the timer
	GPIO_IRQPriorityConfig(IRQ_NO_TIM2, NVIC_IRQ_PRI0); // Set TIM2 interrupt priority
	GPIO_IRQPriorityConfig(IRQ_NO_TIM3, NVIC_IRQ_PRI1); // Set TIM3 interrupt priority
	GPIO_IRQPriorityConfig(IRQ_NO_TIM4, NVIC_IRQ_PRI2); // Set TIM4 interrupt priority
	GPIO_IRQConfig(IRQ_NO_TIM2, ENABLE); // Enable TIM2 interrupt in NVIC
	GPIO_IRQConfig(IRQ_NO_TIM3, ENABLE); // Enable TIM3 interrupt in NVIC
	GPIO_IRQConfig(IRQ_NO_TIM4, ENABLE); // Enable TIM4 interrupt in NVIC
}

void Timer3_StartCountdown(uint32_t milliseconds) {
    TIM3->CR[0] &= ~(1 << 0);        // 確保 Timer 先停住
    TIM3->CR[0] |= (1 << 2);
	TIM3->PSC = 16000 - 1;         // 設定 Prescaler
	TIM3->ARR = milliseconds - 1;    // 設定 ARR
    TIM3->CNT = 0;                   // 重設計數器
	TIM3->EGR |= (1 << 0);
    TIM3->SR &= ~TIM_FLAG_UIF;       // 清除中斷旗標
    TIM3->CR[0] |= (1 << 0);         // 啟動 Timer
}

void Timer4_StartCountdown(uint32_t milliseconds) {
	TIM4->CR[0] &= ~(1 << 0); // 確保 Timer 先停住
	TIM4->CR[0] |= (1 << 2);
	TIM4->PSC = 16000 - 1; // 設定 Prescaler
	TIM4->ARR = milliseconds - 1;
	TIM4->CNT = 0;
	TIM4->EGR |= (1 << 0); // 更新計數器
	TIM4->SR &= ~TIM_FLAG_UIF; // 清除 update flag
	TIM4->CR[0] |= (1 << 0); // 啟動 Timer
}

void Timer3_Stop(void) {
    TIM3->CR[0] &= ~(1 << 0); // 停止計數
    TIM3->CNT = 0;
    TIM3->SR &= ~TIM_FLAG_UIF;   // 清除旗標
	GPIOA->ODR &= ~(1 << 4); // Reset GPIO2
}

void Timer4_Stop(void) {
	TIM4->CR[0] &= ~(1 << 0); // 停止計數
	TIM4->CNT = 0;
	TIM4->SR &= ~TIM_FLAG_UIF;   // 清除旗標
	GPIOA->ODR &= ~(1 << 5); // Reset GPIO2
}

void TIM2_IRQHandler(void) {
	static volatile uint32_t overflow_count = 0; // Overflow count
    if (TIM2->SR & TIM_FLAG_CC1IF) {
        // Clear the interrupt flag
		if (GPIOA->IDR & (1 << 0)) { // Check if the button is pressed
			end_time[0] = TIM2->CCR[0]; // Read the captured value
			end_time[1] = overflow_count; // Store the overflow count
			press_duration = end_time[0] - start_time[0];
			Timer3_Stop(); // Stop the countdown
			Timer3_StartCountdown(600);
			Timer4_StartCountdown(1200);
		}
		else { // Check if the button is released
			start_time[0] = TIM2->CCR[0]; // Read the captured value
			start_time[1] = overflow_count; // Store the overflow count
			Timer3_Stop();
			Timer4_Stop();
			Timer3_StartCountdown(600); //Start 600ms countdown to determine it's a short press or a long press
		}
		TIM2->SR &= ~TIM_FLAG_CC1IF;
    }
	else if (TIM2->SR & TIM_FLAG_UIF) {
		// Clear the update interrupt flag
		TIM2->SR &= ~TIM_FLAG_UIF;
		overflow_count++;
	}
    return;
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_FLAG_UIF) {
        TIM3->SR &= ~TIM_FLAG_UIF; // 清中斷旗標
		TIM3->CR[0] &= ~(1 << 0); // 停止計數
    	TIM3->CNT = 0;
        GPIOA->ODR |= (1 << 4); // Set GPIO3
    }
    return;
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_FLAG_UIF) {
        TIM4->SR &= ~TIM_FLAG_UIF; // 清中斷旗標
		TIM4->CR[0] &= ~(1 << 0); // 停止計數
    	TIM4->CNT = 0;
        GPIOA->ODR |= (1 << 5); // Set GPIO3
    }
    return;
}


int main(void) {
	// TIMER and GPIO handle;
	TIM_Handle_t TimBtn, TimLED1, TimLED2;
	GPIO_Handle_t GPIOBtn, GPIOLED1, GPIOLED2;
	uint32_t t;
	GPIO_InitConfig(&GPIOBtn, &GPIOLED1, &GPIOLED2); // Initialize GPIO configuration
	TIM_Config(&TimBtn, &TimLED1, &TimLED2); // Initialize TIM2 configuration
	while(1){
		t = press_duration;
	}
	return 0;
}

