/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

// TBS Tango 2 Hall gimbals: two Melexis MLX90393 on SPI2 (see spi2_bus.h).
// Gimbal 0: CS PE8, data-ready PC5. Gimbal 1: CS PD7, data-ready PB9.

#pragma once

#include <stdint.h>

struct MlxSample {
  uint8_t status;    // MLX status byte of the last RM
  uint16_t t;        // raw temperature word; 0, not measured (kept for SWD tools)
  int16_t x, y, z;   // raw field words minus the 0x8000 TCMP offset
  uint32_t stamp;    // ms tick of the last good read
  uint16_t errors;   // consecutive bad/missing reads
};

enum { MLX_GIMBAL_LEFT = 0, MLX_GIMBAL_RIGHT = 1 };  // confirmed by capture

extern MlxSample mlxSamples[2];
extern volatile uint32_t mlxReadCount[2];

// Init both sensors with TBS's register set. Returns true when both answer.
bool mlxGimbalInit();

// Maps the newest burst-mode samples (read by the TIM7 poll) to LH/LV/RV/RH
// (0..4095) in analogs[0..3], restarting a sensor that has gone quiet.
// Called from the ADC read in the mixer task.
void mlxGimbalUpdate(uint16_t* analogs);

// Bench tuning: write a register on both sensors and restart burst mode.
// Returns false if a sensor didn't read the value back.
bool mlxSetReg(uint8_t reg, uint16_t val);

// false once a sensor has failed for long enough that its sticks are stale
bool mlxGimbalOk();

// The Crossfire image reads the sensors itself (whole SPI2 bus). Release:
// stop our poll; mlxGimbalUpdate() then maps the image's raw reads with the
// same math, holding the last values until it has some. Reclaim: set the
// sensors up again from scratch (the image reconfigured them).
void mlxGimbalRelease();
bool mlxGimbalReclaim();
