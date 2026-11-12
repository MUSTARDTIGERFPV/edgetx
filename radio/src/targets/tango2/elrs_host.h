/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

// TBS Tango 2: host for the ExpressLRS TX image at ELRS_IMAGE_ADDR
// (elrs_image.h). The image drives the SX1272; EdgeTX talks CRSF to it
// through the virtual serial port in radio_host.cpp. Called by the radio
// image manager only (rfhost task), never from an interrupt.

#pragma once

#include <stdint.h>

// Validate the image, call its init(), route its IRQs and start its task
// (start(), then loop() until stopped). Can be called again after
// elrsHostStop(): init() rebuilds the image's RAM. Returns false on any error.
bool elrsHostStart();

// Radio to standby and IRQs off, then the task ends and the IRQs go back to
// EdgeTX's handlers.
void elrsHostStop();

// One-line state for the CLI / version screen
const char* elrsHostStatus();
const char* elrsHostVersion();
