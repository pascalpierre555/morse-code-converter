/*
 * stm32f429i_gpio_driver.c
 *
 *  Created on: Apr 4, 2025
 *      Author: peng
 */
#include <stm32f429i.h>
#include <stm32f429i_gpio_driver.h>

void GPIO_PeriClockControl(GPIO_Config_t *pGPIOx, uint8_t EnorDi) {
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

    // Enable the peripheral clock for the GPIO port
    GPIO_PeriClockControl(pGPIOHandle->port, ENABLE);

    // Configure the mode
    if (pGPIOHandle->config.mode <= GPIO_MODE_ANALOG) {
        temp = (pGPIOHandle->config.mode << (2 * pGPIOHandle->config->pin));
        pGPIOHandle->port->MODER &= ~(0x03 << (2 * pGPIOHandle->config->pin)); // Clear the bits
        pGPIOHandle->port->MODER |= temp; 
    }
    else {
    }
    temp = 0;

    // Configure the output type
    if (pGPIOHandle->config->otype <= GPIO_OTYPE_OD) {
        temp = (pGPIOHandle->config->otype << pGPIOHandle->config->pin);
        pGPIOHandle->port->OTYPER &= ~(0x01 << pGPIOHandle->config->pin); // Clear the bit
        pGPIOHandle->port->OTYPER |= temp; // Set the bit
    }
    temp = 0;

    // Configure the speed
    if (pGPIOHandle->config->ospeed <= 3) {
        temp = (pGPIOHandle->config->ospeed << (2 * pGPIOHandle->config->pin));
        pGPIOHandle->port->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->config->pin)); // Clear the bits
        pGPIOHandle->port->OSPEEDR |= temp; // Set the bits
    }
    temp = 0;

    // Configure the pull-up/pull-down
    if (pGPIOHandle->config->pupd <= 3) {
        temp = (pGPIOHandle->config->pupd << (2 * pGPIOHandle->config->pin));
        pGPIOHandle->port->PUPDR &= ~(0x03 << (2 * pGPIOHandle->config->pin)); // Clear the bits
        pGPIOHandle->port->PUPDR |= temp; // Set the bits
    }
    temp = 0;

    // Configure the alternate function
    if (pGPIOHandle->config->mode == GPIO_MODE_AF) {
        if (pGPIOHandle->config->pin < 8) {
            temp = (pGPIOHandle->config->af << (4 * pGPIOHandle->config->pin));
            pGPIOHandle->port->AFR[0] &= ~(0x0F << (4 * pGPIOHandle->config->pin)); // Clear the bits
            pGPIOHandle->port->AFR[0] |= temp;
        }
        else {
            temp = (pGPIOHandle->config->af << (4 * (pGPIOHandle->config->pin - 8)));
            pGPIOHandle->port->AFR[1] &= ~(0x0F << (4 * (pGPIOHandle->config->pin - 8))); // Clear the bits
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