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

// TBS Tango 2 power: PB12 holds power, PB14 is the power button and reads
// HIGH while pressed (the opposite of the generic STM32 driver). With USB
// plugged in, the MCU is fed from VBUS, so dropping PB12 cannot switch the
// radio off: pwrOff() resets instead (Wingman resets too).

#include "board.h"
#include "hal/abnormal_reboot.h"
#include "hal/gpio.h"
#include "hal/usb_driver.h"
#include "stm32_gpio.h"

void pwrInit()
{
  // External module power off
  gpio_init(EXTMODULE_PWR_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  EXTERNAL_MODULE_PWR_OFF();

  // Power button: input with pull-up, as Wingman configures it (the line is
  // driven low by the power circuit while the button is released)
  gpio_init(PWR_SWITCH_GPIO, GPIO_IN_PU, GPIO_PIN_SPEED_LOW);
}

void pwrOn()
{
  gpio_init(PWR_ON_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_set(PWR_ON_GPIO);
}

void pwrOff()
{
  if (usbPlugged()) {
    // VBUS keeps the MCU alive, so "off" becomes a reset. Reset into the
    // resident loader rather than straight back into EdgeTX: the radio then
    // sits idle (and flashable) until the power button is held for 2 s, and
    // a boot that ends in pwrOff() (e.g. powered up by USB alone, no button
    // held) can never turn into a reset loop.
    boardRebootToLoader();
  }
  gpio_clear(PWR_ON_GPIO);
}

bool pwrPressed()
{
  return gpio_read(PWR_SWITCH_GPIO) != 0;
}

bool pwrOffPressed()
{
  return pwrPressed();
}

void pwrResetHandler()
{
  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
    pwrOn();
  }
}
