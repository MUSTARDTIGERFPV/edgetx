/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "spi2_bus.h"

#include "board.h"
#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

// MLX90393: datasheet says SPI mode 3, max 10 MHz. TBS's image configures the
// shared bus once as mode 0 at 10.5 MHz; whether it switches per device is not
// known yet (tasks/gimbal_raw.c can try both: mode=0|3, div=4..64).
#if !defined(MLX_SPI_MODE3)
  #define MLX_SPI_MODE3 0
#endif
#if !defined(MLX_SPI_BR)
  #define MLX_SPI_BR (2u << SPI_CR1_BR_Pos)  // PCLK1/8 = 5.25 MHz
#endif

#define CR1_COMMON (SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI)
#define CR1_MODE3  (SPI_CR1_CPOL | SPI_CR1_CPHA)

struct spi2_dev_t {
  gpio_t cs;
  uint16_t cr1;
};

static const spi2_dev_t devices[SPI2_DEV_COUNT] = {
  // SX1272: mode 0, PCLK1/4 = 10.5 MHz (as TBS)
  { SX1272_NSS_GPIO, CR1_COMMON | (1u << SPI_CR1_BR_Pos) },
  { GIMBAL0_CS_GPIO, CR1_COMMON | MLX_SPI_BR | (MLX_SPI_MODE3 ? CR1_MODE3 : 0) },
  { GIMBAL1_CS_GPIO, CR1_COMMON | MLX_SPI_BR | (MLX_SPI_MODE3 ? CR1_MODE3 : 0) },
};

volatile uint32_t spi2XferCount = 0;
static bool spi2Ready = false;

void spi2Init()
{
  if (spi2Ready) return;

  // chip-selects high first, so nothing sees a glitch on the bus
  for (const auto& d : devices) {
    gpio_init(d.cs, GPIO_OUT, GPIO_PIN_SPEED_HIGH);
    gpio_set(d.cs);
  }

  stm32_spi_enable_clock(SPI2);
  gpio_init_af(SPI2_SCK_GPIO, SPI2_GPIO_AF, GPIO_PIN_SPEED_HIGH);
  gpio_init_af(SPI2_MISO_GPIO, SPI2_GPIO_AF, GPIO_PIN_SPEED_HIGH);
  gpio_init_af(SPI2_MOSI_GPIO, SPI2_GPIO_AF, GPIO_PIN_SPEED_HIGH);

  SPI2->CR1 = 0;
  SPI2->CR2 = 0;
  SPI2->CR1 = devices[SPI2_DEV_SX1272].cr1;
  SPI2->CR1 |= SPI_CR1_SPE;

  spi2Ready = true;
}

void spi2Reinit()
{
  spi2Ready = false;
  spi2Init();
}

static inline uint8_t spi2Byte(uint8_t b)
{
  while (!(SPI2->SR & SPI_SR_TXE)) {}
  *(volatile uint8_t*)&SPI2->DR = b;
  while (!(SPI2->SR & SPI_SR_RXNE)) {}
  return *(volatile uint8_t*)&SPI2->DR;
}

void spi2_xfer(uint8_t dev, const uint8_t* tx, uint8_t* rx, uint16_t n)
{
  if (dev >= SPI2_DEV_COUNT || n == 0 || n > SPI2_XFER_MAX) return;
  const spi2_dev_t& d = devices[dev];

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // Mode/speed can only change while the peripheral is disabled
  uint16_t want = d.cr1 | SPI_CR1_SPE;
  if ((SPI2->CR1 & 0xFFFF) != want) {
    while (SPI2->SR & SPI_SR_BSY) {}
    SPI2->CR1 = d.cr1;  // SPE off
    SPI2->CR1 = want;
  }
  (void)SPI2->DR;  // drop any stale byte

  gpio_clear(d.cs);
  for (uint16_t i = 0; i < n; i++) {
    uint8_t r = spi2Byte(tx ? tx[i] : 0x00);
    if (rx) rx[i] = r;
  }
  while (SPI2->SR & SPI_SR_BSY) {}
  gpio_set(d.cs);

  spi2XferCount = spi2XferCount + 1;
  __set_PRIMASK(primask);
}
