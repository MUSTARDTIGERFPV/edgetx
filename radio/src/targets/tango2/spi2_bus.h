/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

// TBS Tango 2 SPI2 bus: SX1272 (driven later by the ELRS image) and the two
// MLX90393 gimbal sensors share SPI2. This is the only code that touches the
// SPI2 registers; the ELRS image gets spi2_xfer() as host_api->spi_xfer.

#pragma once

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

// Device numbers match ELRS_SPI_* in abi/elrs_image.h
enum {
  SPI2_DEV_SX1272 = 0,   // CS PC1, mode 0, PCLK1/4 = 10.5 MHz
  SPI2_DEV_GIMBAL0 = 1,  // CS PE8
  SPI2_DEV_GIMBAL1 = 2,  // CS PD7
  SPI2_DEV_COUNT
};

#define SPI2_XFER_MAX 64

// Pins, clock, all chip-selects high. Idempotent.
void spi2Init();

// spi2Init() again from scratch, after another owner (the Crossfire image)
// had the bus and SPI2 was reset under it.
void spi2Reinit();

// One chip-select-framed, full-duplex transfer of n bytes. tx == NULL sends
// zeros, rx == NULL discards. Atomic with respect to every other SPI2 user:
// interrupts are masked for its duration (polled, ~1 us per byte at
// 10.5 MHz; n <= SPI2_XFER_MAX).
void spi2_xfer(uint8_t dev, const uint8_t* tx, uint8_t* rx, uint16_t n);

// Statistics for the debug screens
extern volatile uint32_t spi2XferCount;

#if defined(__cplusplus)
}
#endif
