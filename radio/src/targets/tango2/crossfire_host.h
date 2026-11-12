/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

// TBS Tango 2: host for the closed TBS Crossfire TX image at
// CROSSFIRE_IMAGE_ADDR, "API 0x103".
// Called by the radio image manager only (rfhost task).

#pragma once

#include <stdint.h>

// boardInit(): route the image's IRQs to inert trampolines
void xfHostInit();

// Check the image, set up the shared struct and start its task. The image
// then owns SPI2 (SX1272 and both gimbal sensors), UART4 (WiFi), I2C1,
// TIM2 (LED), TIM5, TIM13, DMA1 streams 2/6 and EXTI10-15.
bool xfHostStart();

// Settings save + power-off handshakes, then kill the task and take all of
// that hardware back (peripherals reset, pins parked, RAM cleared).
void xfHostStop();

// From the rfhost task while the image runs, every ms: frames from the image
// to EdgeTX, its SD file service, flags (USB storage). False when the image
// has parked itself (it wants a reboot): stop it.
bool xfHostPoll();

// The four stick values the image hands back (calibrated, centre 0,
// +-0x4000 full travel), in EdgeTX's LH/LV/RV/RH order. False until the image
// has valid data.
bool xfHostSticks(int16_t out[4]);

// The image's last raw gimbal reads, as raw MLX90393 words - 0x8000 (the
// form mlx90393_gimbal.cpp uses), [sensor][X, Y, Z]. False until it has read.
bool xfHostRawGimbals(int16_t xyz[2][3]);

// SD file service trace for the CLI (`rf sdlog`), oldest first
uint32_t xfHostCopySdTrace(char* buf, uint32_t len);

const char* xfHostStatus();
