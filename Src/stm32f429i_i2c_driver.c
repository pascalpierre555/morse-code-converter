#include "stm32f429i_i2c_driver.h"
#include "stm32f429i_gpio_driver.h"
#include "stm32f429i.h"  // 若有共用宏定義，也可包含此檔

void I2C1_Init(void) {
    // 開啟時鐘
    GPIOB_PCLK_EN();
    I2C1_PCLK_EN();

    // 設定 PB6 = I2C1_SCL, PB7 = I2C1_SDA
    GPIOB->MODER &= ~(0xF << (6 * 2));  // 清 PB6, PB7
    GPIOB->MODER |= (0xA << (6 * 2));   // AF mode

    GPIOB->OTYPER |= (0x3 << 6);        // Open-drain
    GPIOB->OSPEEDR |= (0xF << (6 * 2)); // High speed
    GPIOB->PUPDR &= ~(0xF << (6 * 2));  // No pull-up/down

    GPIOB->AFR[0] |= (4 << (6 * 4));    // AF4 for I2C1_SCL
    GPIOB->AFR[0] |= (4 << (7 * 4));    // AF4 for I2C1_SDA

    // 關閉 I2C 模組
    I2C1->CR[0] &= ~(1 << 0);

    // 設定外部時鐘來源 = 16MHz（或依實際）
    I2C1->CR[0] = 16;

    // 設定 clock control register（100kHz, standard mode）
    I2C1->CCR = 80;  // CCR = Fpclk / (2 * I2C_speed) = 16MHz / (2 * 100kHz)

    // 設定 TRISE = Fpclk + 1 = 17
    I2C1->TRISE = 17;

    // 開啟模組
    I2C1->CR[0] |= (1 << 0);
}

void I2C1_SendByte(uint8_t slave_address, uint8_t data) {
    // 產生 START
    I2C1->CR[0] |= (1 << 8);
    while (!(I2C1->SR[0] & (1 << 0)));  // SB set

    // 發送 slave address + write (0)
    I2C1->DR = slave_address << 1;
    while (!(I2C1->SR[0] & (1 << 1)));  // ADDR set
    (void)I2C1->SR[1];  // 清 ADDR

    // 傳送 data
    I2C1->DR = data;
    while (!(I2C1->SR[0] & (1 << 7)));  // TXE set
    while (!(I2C1->SR[0] & (1 << 2)));  // BTF set

    // 發送 STOP
    I2C1->CR[0] |= (1 << 9);
}
