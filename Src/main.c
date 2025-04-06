/*
 * main.c
 *
 *  Created on: Apr 4, 2025
 *      Author: peng
 */

 #include "stm32f429i.h"
 #include "stm32f429i_gpio_driver.h"
 #include "stm32f429i_gpio_driver.c"

int main(void) {
	return 0;
}

void EXTI0_IRQHandler(void) {
	if (EXTI->PR & (1 << 0)) {
		EXTI->PR |= (1 << 0);
	}
}