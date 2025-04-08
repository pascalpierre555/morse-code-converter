/*
 * stm32F429i_gpio_driver.c
 *
 *  Created on: Apr 6, 2025
 *      Author: peng
 */

#include "stm32f429i.h"
#include "stm32f429i_gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
        }
        else if (pGPIOx == GPIOJ) {
            GPIOJ_PCLK_EN();
        }
        else if (pGPIOx == GPIOK) {
            GPIOK_PCLK_EN();
        }
        else {
        }
    }
    else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_DI();
        }
        else if (pGPIOx == GPIOJ) {
            GPIOJ_PCLK_DI();
        }
        else if (pGPIOx == GPIOK) {
            GPIOK_PCLK_DI();
        }
        else {
        }
    }
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    uint32_t temp = 0;


    // Configure the mode
    if (pGPIOHandle->config.mode <= GPIO_MODE_ANALOG) {
        temp = (pGPIOHandle->config.mode << (2 * pGPIOHandle->config.pin));
        pGPIOHandle->port->MODER &= ~(0x03 << (2 * pGPIOHandle->config.pin)); // Clear the bits
        pGPIOHandle->port->MODER |= temp;
    }
    else {
        if (pGPIOHandle->config.mode == GPIO_MODE_IT_FT) {
            // Configure for falling edge trigger
            EXTI->FTSR |= (1 << pGPIOHandle->config.pin);
            EXTI->RTSR &= ~(1 << pGPIOHandle->config.pin); // Clear the rising edge trigger
        }
        else if (pGPIOHandle->config.mode == GPIO_MODE_IT_RT) {
            // Configure for rising edge trigger
            EXTI->RTSR |= (1 << pGPIOHandle->config.pin);
            EXTI->FTSR &= ~(1 << pGPIOHandle->config.pin); // Clear the falling edge trigger
        }
        else if (pGPIOHandle->config.mode == GPIO_MODE_IT_RFT) {
            // Configure for both edges trigger
            EXTI->RTSR |= (1 << pGPIOHandle->config.pin);
            EXTI->FTSR |= (1 << pGPIOHandle->config.pin);
        }

        //configure gpio portselection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->config.pin / 4;
        uint8_t temp2 = pGPIOHandle->config.pin % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->port);
        SYSCFG_PCLK_EN(); // Enable SYSCFG clock
        SYSCFG->EXTICR[temp1] &= ~(0x0F << (temp2 * 4)); // Clear the bits
        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4); // Set the bits

        // Enable the interrupt
        EXTI->IMR |= (1 << pGPIOHandle->config.pin); // Unmask the interrupt
    }
    temp = 0;

    // Configure the output type
    if (pGPIOHandle->config.otype <= GPIO_OTYPE_OD) {
        temp = (pGPIOHandle->config.otype << pGPIOHandle->config.pin);
        pGPIOHandle->port->OTYPER &= ~(0x01 << pGPIOHandle->config.pin); // Clear the bit
        pGPIOHandle->port->OTYPER |= temp; // Set the bit
    }
    temp = 0;

    // Configure the speed
    if (pGPIOHandle->config.ospeed <= 3) {
        temp = (pGPIOHandle->config.ospeed << (2 * pGPIOHandle->config.pin));
        pGPIOHandle->port->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->config.pin)); // Clear the bits
        pGPIOHandle->port->OSPEEDR |= temp; // Set the bits
    }
    temp = 0;

    // Configure the pull-up/pull-down
    if (pGPIOHandle->config.pupd <= 3) {
        temp = (pGPIOHandle->config.pupd << (2 * pGPIOHandle->config.pin));
        pGPIOHandle->port->PUPDR &= ~(0x03 << (2 * pGPIOHandle->config.pin)); // Clear the bits
        pGPIOHandle->port->PUPDR |= temp; // Set the bits
    }
    temp = 0;

    // Configure the alternate function
    if (pGPIOHandle->config.mode == GPIO_MODE_AF) {
        if (pGPIOHandle->config.pin < 8) {
            temp = (pGPIOHandle->config.af << (4 * pGPIOHandle->config.pin));
            pGPIOHandle->port->AFR[0] &= ~(0x0F << (4 * pGPIOHandle->config.pin)); // Clear the bits
            pGPIOHandle->port->AFR[0] |= temp;
        }
        else {
            temp = (pGPIOHandle->config.af << (4 * (pGPIOHandle->config.pin - 8)));
            pGPIOHandle->port->AFR[1] &= ~(0x0F << (4 * (pGPIOHandle->config.pin - 8))); // Clear the bits
            pGPIOHandle->port->AFR[1] |= temp;
        }
    }
    temp = 0;
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_PCLK_DI();
    }
    else if (pGPIOx == GPIOB) {
        GPIOB_PCLK_DI();
    }
    else if (pGPIOx == GPIOC) {
        GPIOC_PCLK_DI();
    }
    else if (pGPIOx == GPIOD) {
        GPIOD_PCLK_DI();
    }
    else if (pGPIOx == GPIOE) {
        GPIOE_PCLK_DI();
    }
    else if (pGPIOx == GPIOF) {
        GPIOF_PCLK_DI();
    }
    else if (pGPIOx == GPIOG) {
        GPIOG_PCLK_DI();
    }
    else if (pGPIOx == GPIOH) {
        GPIOH_PCLK_DI();
    }
    else if (pGPIOx == GPIOI) {
        GPIOI_PCLK_DI();
    }
    else if (pGPIOx == GPIOJ) {
        GPIOJ_PCLK_DI();
    }
    else if (pGPIOx == GPIOK) {
        GPIOK_PCLK_DI();
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
    if (Value == GPIO_PIN_SET) {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQConfig (uint8_t IRQNumber, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (IRQNumber <= 31) {
            // Program ISER0 register
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64) {
            // Program ISER1 register
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96) {
            // Program ISER2 register
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else {
        if (IRQNumber <= 31) {
            // Program ICER0 register
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64) {
            // Program ICER1 register
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96) {
            // Program ICER2 register
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    // Calculate the priority group
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber) {
    // Clear the EXTI line pending register
    if (EXTI->PR & (1 << PinNumber)) {
        EXTI->PR |= (1 << PinNumber);
    }
}
