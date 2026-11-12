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

// TBS Tango 2 board definitions (modelled on targets/taranis/board.h)

#pragma once

#include <inttypes.h>
#include "hal.h"
#include "hal/serial_port.h"
#include "hal/watchdog_driver.h"

#include "definitions.h"
#include "edgetx_constants.h"
#include "board_common.h"

#if defined(ROTARY_ENCODER_NAVIGATION)
void rotaryEncoderInit();
void rotaryEncoderCheck();
void rotaryEncoderStartPolling();
#endif

// Flash: EdgeTX is not at the start of flash on this radio. The TBS
// bootloader (0x08000000) and the resident loader (0x0800C000) must never be
// written; flash_driver.cpp refuses anything below TANGO2_FLASH_WRITE_MIN.
#define FLASHSIZE                       0x100000 // 1M
#define FLASH_PAGESIZE                  256
#define BOOTLOADER_SIZE                 0x20000  // TBS bootloader + resident loader
#define FIRMWARE_ADDRESS                0x08000000
#define FIRMWARE_LEN(fsize)             (fsize - BOOTLOADER_SIZE)
#define FIRMWARE_MAX_LEN                0xA0000  // 640 KB, up to the ELRS image
#define APP_START_ADDRESS               (uint32_t)(FIRMWARE_ADDRESS + BOOTLOADER_SIZE)
#define TANGO2_FLASH_WRITE_MIN          0x080C0000  // ELRS image slot and up only

#define LUA_MEM_MAX                     (0)

#define BOOTLOADER_KEYS                 0x42

extern uint16_t sessionTimer;

// Board driver
void boardInit();
void boardOff();

// Reboot into the resident loader (RTC BKP19R = "MON1", then reset)
void boardRebootToLoader();

// Internal module: the ELRS or Crossfire image (radio_host.h). The rfhost
// task starts the one the model selects, so power-on is a no-op.
void tango2IntmoduleOn();
// SX1272, WiFi, ATSHA204A and LED pins to their idle state (radio images off)
void tango2ParkRadioHardware();
#define INTERNAL_MODULE_ON()            tango2IntmoduleOn()
#define INTERNAL_MODULE_OFF()

// Pulses driver
#define EXTERNAL_MODULE_ON()            EXTERNAL_MODULE_PWR_ON()
#define EXTERNAL_MODULE_OFF()           EXTERNAL_MODULE_PWR_OFF()

// Trainer driver
#define SLAVE_MODE()                    (g_model.trainerData.mode == TRAINER_MODE_SLAVE)

#define NUM_FUNCTIONS_SWITCHES 0

#define HAS_HARDWARE_OPTIONS

PACK(typedef struct {
  uint8_t pcbrev:2;
}) HardwareOptions;

extern HardwareOptions hardwareOptions;

// Battery driver: 1S Li-ion (confirmed). Divider measured 2026-11-10 against a
// bench supply: ADC filtered 1414 at 3.60 V on the cell -> 1414*10/3.60.
// Single point; check the slope at 4.2 V / 3.4 V when convenient.
#define BATTERY_WARN                    35 // 3.5V
#define BATTERY_MIN                     34 // 3.4V
#define BATTERY_MAX                     42 // 4.2V
#define BATTERY_DIVIDER                 3928

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

// Power driver
#define SOFT_PWR_CTRL
#define STARTUP_ANIMATION

void pwrInit();
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
bool pwrOffPressed();
void pwrResetHandler();
#define pwrForcePressed()   false

// Backlight driver: OLED, "backlight" is the panel contrast
#define BACKLIGHT_DISABLE()             lcdSetRefVolt(0)
#define BACKLIGHT_FORCED_ON             255
#define BACKLIGHT_ENABLE()              lcdSetRefVolt(currentBacklightBright)

void backlightInit();
void backlightDisable();
void backlightFullOn();
bool isBacklightEnabled();

#if defined(__cplusplus) && !defined(SIMU)
}
#endif

// Debug driver
void debugPutc(const char c);

// Audio driver
void audioInit();

// Haptic driver
void hapticInit();
void hapticOff();
void hapticOn(uint32_t pwmPercent);

#define DEBUG_BAUDRATE                  115200
#define LUA_DEFAULT_BAUDRATE            115200

const etx_serial_port_t* auxSerialGetPort(int port_nr);

// LED driver (single WS2812 on PB11)
void ledInit();
void ledOff();
void ledRed();
void ledGreen();
void ledBlue();
void ledBoot();

// LCD driver
#define IS_LCD_RESET_NEEDED()           true
#define LCD_CONTRAST_MIN                2
#define LCD_CONTRAST_MAX                254
#define LCD_CONTRAST_DEFAULT            254

void lcdInit();
void lcdInitFinish();
void lcdOff();

#if !defined(SIMU)
void lcdRefreshWait();
#else
#define lcdRefreshWait()
#endif
#if defined(SIMU) || !defined(__cplusplus)
void lcdRefresh();
#else
void lcdRefresh(bool wait=true);
#endif
void lcdSetRefVolt(unsigned char val);
void lcdSetInvert(bool invert);
#ifdef __cplusplus
void lcdSetContrast(bool useDefault = false);
#endif

#if defined(CROSSFIRE)
#define TELEMETRY_FIFO_SIZE             128
#else
#define TELEMETRY_FIFO_SIZE             64
#endif

#define INTMODULE_FIFO_SIZE             128

#define NUM_TRIMS                       4
#define NUM_TRIMS_KEYS                  (NUM_TRIMS * 2)
