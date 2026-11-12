/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

// TBS Tango 2: the internal RF is one of two separately built images in flash
// sectors 10-11, the ExpressLRS image (elrs_host.cpp) or the TBS Crossfire
// image (crossfire_host.cpp). Exactly one runs at a time. The model's internal
// module (type CRSF) picks it with its subType: TANGO2_RF_ELRS / _CROSSFIRE.
// A service task watches that and switches images without a reboot.
//
// Both images talk CRSF to EdgeTX through one virtual serial port, which is
// the internal module port.

#pragma once

#include <stdint.h>
#include "hal/serial_driver.h"

enum RadioImage : uint8_t {
  RADIO_IMAGE_NONE,
  RADIO_IMAGE_ELRS,
  RADIO_IMAGE_CROSSFIRE,
};

// Internal module subType values (model setting)
enum { TANGO2_RF_ELRS = 0, TANGO2_RF_CROSSFIRE = 1, TANGO2_RF_LAST = TANGO2_RF_CROSSFIRE };

// boardInit(): RAM vector table, service task (runs once the scheduler starts)
void radioHostInit();

// boardOff(): stop the running image cleanly (Crossfire saves its settings)
void radioHostPowerOff();

RadioImage radioHostActive();
const char* radioHostImageName(RadioImage img);
// Status and log for the CLI (`rf`)
const char* radioHostStatus();
uint32_t radioHostCopyLog(char* buf, uint32_t len);
void radioHostLog(const char* fmt, ...);

// --- for the image backends -------------------------------------------------

// Point an IRQ at a handler in the RAM vector table, or back at EdgeTX's own
// (flash) handler with nullptr.
void radioHostSetIrq(int irqn, void (*handler)());

// EdgeTX -> image. The running image installs a hook that takes the bytes
// EdgeTX's CRSF driver sends (mixer task); without one they are dropped.
// radioHostQueueFrame() + radioHostReadFrame() are a ring the hook can use:
// readFrame returns one whole CRSF frame per call, 0 when none is pending.
void radioHostSetSendHook(void (*hook)(const uint8_t* data, uint32_t size));
bool radioHostQueueFrame(const uint8_t* data, uint32_t size);
uint16_t radioHostReadFrame(uint8_t* buf, uint16_t max);
// image -> EdgeTX: one whole CRSF frame (dropped if EdgeTX isn't reading)
void radioHostWriteFrame(const uint8_t* frame, uint16_t len);

// CLI (`rf send` / `rf cap`): inject one whole CRSF frame into the running
// image as if EdgeTX sent it, and take the extended (device protocol, type
// >= 0x28) frames the image sent since the last call.
bool radioHostInject(const uint8_t* frame, uint32_t len);
uint32_t radioHostCopyCapture(uint8_t* buf, uint32_t len);

// The virtual serial port for the internal module
extern const etx_serial_driver_t radioVirtualSerialDriver;
