/*
 * gpio_driver.h
 *
 *  Created on: Apr 4, 2025
 *      Author: peng
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include "stm32f429i.h"

// GPIO 暫存器位址偏移量 (根據來源 [1-5])
#define GPIO_MODER_OFFSET   (0x00)
#define GPIO_OTYPER_OFFSET  (0x04)
#define GPIO_OSPEEDR_OFFSET (0x08)
#define GPIO_PUPDR_OFFSET  (0x0C)
#define GPIO_IDR_OFFSET    (0x10)
#define GPIO_ODR_OFFSET    (0x14)
#define GPIO_BSRR_OFFSET   (0x18)
#define GPIO_LCKR_OFFSET   (0x1C)
#define GPIO_AFRL_OFFSET   (0x20)
#define GPIO_AFRH_OFFSET   (0x24)

// RCC AHB1ENR 暫存器位址偏移量 (根據來源 [6])
#define RCC_AHB1ENR_OFFSET (0x30)

// GPIO 周邊基底位址 (根據 stm32f4xx.h 或您使用的庫，這裡僅為範例)
#define GPIOA_BASE (GPIOA_BASE)
#define GPIOB_BASE (GPIOB_BASE)
#define GPIOC_BASE (GPIOC_BASE)
#define GPIOD_BASE (GPIOD_BASE)
#define GPIOE_BASE (GPIOE_BASE)
#define GPIOF_BASE (GPIOF_BASE)
#define GPIOG_BASE (GPIOG_BASE)
#define GPIOH_BASE (GPIOH_BASE)
#define GPIOI_BASE (GPIOI_BASE)
#ifdef GPIOJ_BASE
#define GPIOJ_BASE (GPIOJ_BASE)
#endif
#ifdef GPIOK_BASE
#define GPIOK_BASE (GPIOK_BASE)
#endif

// GPIO 結構定義
typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
    uint32_t mode;
    uint32_t otype;
    uint32_t ospeed;
    uint32_t pupd;
    uint32_t af;
} GPIO_InitTypeDef;

// GPIO 模式定義
#define GPIO_MODE_INPUT     (0x00)
#define GPIO_MODE_OUTPUT    (0x01)
#define GPIO_MODE_AF        (0x02)
#define GPIO_MODE_ANALOG    (0x03)

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

// 內部函式：設定位元
static void GPIO_SetBit(volatile uint32_t *reg, uint32_t bit) {
    *reg |= (1 << bit);
}

// 內部函式：清除位元
static void GPIO_ClrBit(volatile uint32_t *reg, uint32_t bit) {
    *reg &= ~(1 << bit);
}

// 內部函式：設定欄位值
static void GPIO_SetField(volatile uint32_t *reg, uint32_t value, uint32_t offset, uint32_t mask) {
    *reg = (*reg & ~(mask << offset)) | (value << offset);
}

// 初始化 GPIO
void GPIO_Init(GPIO_InitTypeDef *GPIO_InitStruct) {
    // 啟用 GPIO 時脈
    uint32_t rcc_ahb1enr = RCC->AHB1ENR;
    if (GPIO_InitStruct->port == GPIOA) {
        GPIO_SetBit(&rcc_ahb1enr, 0); // GPIOAEN
    } else if (GPIO_InitStruct->port == GPIOB) {
        GPIO_SetBit(&rcc_ahb1enr, 1); // GPIOBEN
    } else if (GPIO_InitStruct->port == GPIOC) {
        GPIO_SetBit(&rcc_ahb1enr, 2); // GPIOCEN
    } else if (GPIO_InitStruct->port == GPIOD) {
        GPIO_SetBit(&rcc_ahb1enr, 3); // GPIODEN
    } else if (GPIO_InitStruct->port == GPIOE) {
        GPIO_SetBit(&rcc_ahb1enr, 4); // GPIOEEN
    } else if (GPIO_InitStruct->port == GPIOF) {
        GPIO_SetBit(&rcc_ahb1enr, 5); // GPIOFEN
    } else if (GPIO_InitStruct->port == GPIOG) {
        GPIO_SetBit(&rcc_ahb1enr, 6); // GPIOGEN
    } else if (GPIO_InitStruct->port == GPIOH) {
        GPIO_SetBit(&rcc_ahb1enr, 7); // GPIOHEN
    } else if (GPIO_InitStruct->port == GPIOI) {
        GPIO_SetBit(&rcc_ahb1enr, 8); // GPIOIEN
    }
#ifdef GPIOJ_BASE
    else if (GPIO_InitStruct->port == GPIOJ) {
        GPIO_SetBit(&rcc_ahb1enr, 9); // GPIOJEN
    }
#endif
#ifdef GPIOK_BASE
    else if (GPIO_InitStruct->port == GPIOK) {
        GPIO_SetBit(&rcc_ahb1enr, 10); // GPIOKEN
    }
#endif
    RCC->AHB1ENR = rcc_ahb1enr;

    uint32_t pin = GPIO_InitStruct->pin;

    // 設定 GPIO 模式 (根據來源 [7, 8])
    GPIO_SetField(&GPIO_InitStruct->port->MODER, GPIO_InitStruct->mode, pin * 2, 0x03);

    // 設定輸出類型 (僅在輸出模式或 AF 模式下有效) (根據來源 [7, 9])
    if (GPIO_InitStruct->mode == GPIO_MODE_OUTPUT || GPIO_InitStruct->mode == GPIO_MODE_AF) {
        GPIO_SetBit(&GPIO_InitStruct->port->OTYPER, GPIO_InitStruct->otype << pin);
    }

    // 設定輸出速度 (僅在輸出模式或 AF 模式下有效) (根據來源 [7, 10])
    if (GPIO_InitStruct->mode == GPIO_MODE_OUTPUT || GPIO_InitStruct->mode == GPIO_MODE_AF) {
        GPIO_SetField(&GPIO_InitStruct->port->OSPEEDR, GPIO_InitStruct->ospeed, pin * 2, 0x03);
    }

    // 設定上拉/下拉 (根據來源 [7, 10, 11, 22])
    GPIO_SetField(&GPIO_InitStruct->port->PUPDR, GPIO_InitStruct->pupd, pin * 2, 0x03);

    // 設定替代功能 (僅在 AF 模式下有效) (根據來源 [4, 5, 13, 14, 23-25])
    if (GPIO_InitStruct->mode == GPIO_MODE_AF) {
        if (pin < 8) {
            GPIO_SetField(&GPIO_InitStruct->port->AFR, GPIO_InitStruct->af, pin * 4, 0x0F); // GPIOx_AFRL
        } else {
            GPIO_SetField(&GPIO_InitStruct->port->AFR[26], GPIO_InitStruct->af, (pin - 8) * 4, 0x0F); // GPIOx_AFRH
        }
    } else if (GPIO_InitStruct->mode == GPIO_MODE_ANALOG) {
        // 根據來源 [27]，類比模式會禁用輸出緩衝區和上拉/下拉
    } else {
        // 根據來源 [28]，重置後預設為輸入浮動模式
    }
}

// 讀取 GPIO 輸入 (根據來源 [11, 22, 29])
uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}

// 設定 GPIO 輸出 (根據來源 [3, 28-30])
void GPIO_SetOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIOx->BSRR = GPIO_Pin; // 設定 BS 位元 (根據來源 [31, 32])
}

// 清除 GPIO 輸出 (根據來源 [3, 28-30])
void GPIO_ResetOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIOx->BSRR = (GPIO_Pin << 16); // 設定 BR 位元 (根據來源 [31, 32])
}

// 寫入 GPIO 輸出資料暫存器 (根據來源 [3, 28-30])
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t State) {
    if (State != 0) {
        GPIO_SetOutputPin(GPIOx, GPIO_Pin);
    } else {
        GPIO_ResetOutputPin(GPIOx, GPIO_Pin);
    }
}

// 鎖定 GPIO 配置 (根據來源 [4, 30, 32-36])
void GPIO_LockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t lckr_value = (0x1FFFF << 16) | GPIO_Pin; // 設定 LCKK 和要鎖定的 Pin
    // 鎖定序列 (必須是 32 位元寫入) (根據來源 [32, 35])
    GPIOx->LCKR = lckr_value;
    GPIOx->LCKR = GPIO_Pin;
    GPIOx->LCKR = lckr_value;
    (void)GPIOx->LCKR; // 讀取以確認鎖定 (根據來源 [34])
}

#endif /* GPIO_DRIVER_H_ */
