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

// Hand-off to the resident loader at 0x0800C000 (two-to-tango: loader/).
// On reset the loader stays resident instead of booting EdgeTX when
// RTC->BKP19R holds "MON1" (it clears it).

#include "board.h"
#include "stm32_hal_ll.h"

void boardRebootToLoader()
{
  __disable_irq();
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
  PWR->CR |= PWR_CR_DBP;
  RTC->BKP19R = LOADER_STAY_MAGIC;
  NVIC_SystemReset();
}
