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

// Host side of the ELRS image ABI (elrs_image.h), modelled on the resident
// loader's development host (two-to-tango: loader/src/image_host.cpp).
//
// CRSF goes through radio_host.cpp: EdgeTX's frames land in its ring (send
// hook below, mixer task) and crsf_read() hands the image one whole frame at
// a time; crsf_write() goes straight to radioHostWriteFrame().
// The image's DIO and timer IRQs run at NVIC priority 4, above
// configMAX_SYSCALL_INTERRUPT_PRIORITY; they never call into EdgeTX except
// spi_xfer/micros, which are IRQ-safe.

#include "elrs_host.h"
#include "elrs_image.h"
#include "radio_host.h"
#include "spi2_bus.h"

#include "edgetx.h"
#include "os/task.h"
#include "os/sleep.h"
#include "delays_driver.h"
#include "timers_driver.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define ELRS_CONFIG_DIR   "/ELRS"
#define ELRS_CONFIG_FILE  ELRS_CONFIG_DIR "/config.bin"
#define ELRS_STACK_SIZE   768  // words
#define ELRS_TASK_PRIO    3    // below the mixer (4), with audio
#define ELRS_STOP_TIMEOUT 500  // ms for the task to finish its loop() pass

static const elrs_image_header_t* const image =
    (const elrs_image_header_t*)ELRS_IMAGE_ADDR;

static_assert(ELRS_TASK_PRIO < configMAX_PRIORITIES, "FreeRTOS priority out of range");

static char statusText[48] = "not started";
static bool imageValid;

static task_handle_t elrsTaskId;
TASK_DEFINE_STACK(elrsStack, ELRS_STACK_SIZE);
static volatile bool stopRequested;
static volatile bool taskRunning;

static void setStatus(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(statusText, sizeof(statusText), fmt, ap);
  va_end(ap);
  radioHostLog("[elrs host] %s", statusText);
}

// --- elrs_host_api_t ----------------------------------------------------------

static void h_spi_xfer(uint8_t dev, const uint8_t* tx, uint8_t* rx, uint16_t n)
{
  // ABI device numbers are the spi2_bus ones
  static_assert(ELRS_SPI_SX1272 == SPI2_DEV_SX1272 &&
                ELRS_SPI_GIMBAL0 == SPI2_DEV_GIMBAL0 &&
                ELRS_SPI_GIMBAL1 == SPI2_DEV_GIMBAL1, "SPI2 device ids");
  spi2_xfer(dev, tx, rx, n);
}

static uint16_t h_crsf_read(uint8_t* buf, uint16_t max)
{
  return radioHostReadFrame(buf, max);
}

static void h_crsf_write(const uint8_t* frame, uint16_t len)
{
  radioHostWriteFrame(frame, len);
}

static uint32_t h_micros(void) { return timersGetUsTick(); }
static uint32_t h_millis(void) { return timersGetMsTick(); }
static void h_delay_us(uint32_t us) { delay_us(us); }

static int h_config_load(void* buf, uint32_t len)
{
  FIL f;
  UINT got = 0;
  if (f_open(&f, ELRS_CONFIG_FILE, FA_OPEN_EXISTING | FA_READ) != FR_OK) return -1;
  FRESULT res = f_read(&f, buf, len, &got);
  f_close(&f);
  return res == FR_OK ? (int)got : -1;
}

static int h_config_save(const void* buf, uint32_t len)
{
  FIL f;
  UINT put = 0;
  f_mkdir(ELRS_CONFIG_DIR);
  if (f_open(&f, ELRS_CONFIG_FILE, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return -1;
  FRESULT res = f_write(&f, buf, len, &put);
  f_close(&f);
  return (res == FR_OK && put == len) ? 0 : -1;
}

static void h_log(const char* fmt, ...)
{
  char b[96];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  radioHostLog("[ELRS] %s", b);
}

// Measured on the bench unit 2026-11-10.
// TODO: per-unit value from radio settings
static int16_t h_crystal_ppm_x10(void) { return -159; }

// TBS hwId in the bootloader area, e.g. 0x00040005 -> 5
static uint8_t h_hw_rev(void) { return *(volatile uint32_t*)0x08007F00u & 0xFF; }

static const elrs_host_api_t hostApi = {
  ELRS_ABI_MAJOR, ELRS_ABI_MINOR,
  h_spi_xfer, h_crsf_read, h_crsf_write,
  h_micros, h_millis, h_delay_us,
  h_config_load, h_config_save, h_log,
  h_crystal_ppm_x10, h_hw_rev,
};

// --- image lifecycle ------------------------------------------------------------

static uint32_t crc32(const uint8_t* p, uint32_t n)
{
  uint32_t crc = 0xFFFFFFFFu;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  }
  return crc ^ 0xFFFFFFFFu;
}

static bool validate()
{
  if (image->magic != ELRS_IMAGE_MAGIC) {
    setStatus("no ELRS image");
    return false;
  }
  if (image->abi_major != ELRS_ABI_MAJOR) {
    setStatus("image ABI %u, host %u", image->abi_major, ELRS_ABI_MAJOR);
    return false;
  }
  if (image->image_size < ELRS_IMAGE_HEADER_SIZE || image->image_size > ELRS_IMAGE_MAX) {
    setStatus("bad image size");
    return false;
  }
  uint32_t crc = crc32((const uint8_t*)ELRS_IMAGE_ADDR + ELRS_IMAGE_HEADER_SIZE,
                       image->image_size - ELRS_IMAGE_HEADER_SIZE);
  if (crc != image->image_crc32) {
    setStatus("image CRC %08lx", (unsigned long)crc);
    return false;
  }
  if (image->ram_start < ELRS_IMAGE_RAM_START || image->ram_end > ELRS_IMAGE_RAM_END) {
    setStatus("image RAM window");
    return false;
  }
  return true;
}

static void sendHook(const uint8_t* data, uint32_t size)
{
  if (!radioHostQueueFrame(data, size)) return;  // image not reading: drop
  if (taskRunning && elrsTaskId._rtos_handle)
    xTaskNotifyGive(elrsTaskId._rtos_handle);
}

static void elrsTask()
{
  image->start();
  while (!stopRequested) {
    image->loop();
    // woken early by the send hook; 1 ms otherwise
    ulTaskNotifyTake(pdTRUE, 1);
  }
  // stop() from here, between loop() passes: the image may block in
  // config_save() (SD), so another task must not call it mid-pass.
  image->stop();
  taskRunning = false;
  vTaskDelete(nullptr);
}

bool elrsHostStart()
{
  if (!imageValid && !(imageValid = validate())) return false;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  int rc = image->init(&hostApi);
  if (rc) {
    __set_PRIMASK(primask);
    setStatus("image init rc=%d", rc);
    return false;
  }
  if (image->irq_exti15_10) radioHostSetIrq(EXTI15_10_IRQn, image->irq_exti15_10);
  if (image->irq_timer) radioHostSetIrq(image->timer_irqn, image->irq_timer);
  __set_PRIMASK(primask);

  stopRequested = false;
  taskRunning = true;
  radioHostSetSendHook(sendHook);
  task_create(&elrsTaskId, elrsTask, "elrs", elrsStack, ELRS_STACK_SIZE,
              ELRS_TASK_PRIO);
  setStatus("running");
  return true;
}

void elrsHostStop()
{
  if (!taskRunning) return;
  radioHostSetSendHook(nullptr);
  stopRequested = true;
  xTaskNotifyGive(elrsTaskId._rtos_handle);
  uint32_t t0 = timersGetMsTick();
  while (taskRunning && timersGetMsTick() - t0 < ELRS_STOP_TIMEOUT) sleep_ms(1);
  if (taskRunning) {
    // Stuck in loop(): take it down anyway, the IRQs below go with it
    vTaskDelete(elrsTaskId._rtos_handle);
    taskRunning = false;
    image->stop();
    setStatus("task did not stop, deleted");
  }

  // stop() turned the image's IRQs off; give the vectors back
  NVIC_DisableIRQ(EXTI15_10_IRQn);
  radioHostSetIrq(EXTI15_10_IRQn, nullptr);
  if (image->irq_timer) {
    NVIC_DisableIRQ((IRQn_Type)image->timer_irqn);
    radioHostSetIrq(image->timer_irqn, nullptr);
  }
  setStatus("stopped");
}

const char* elrsHostStatus() { return statusText; }

const char* elrsHostVersion()
{
  if (!imageValid || !image->version) return nullptr;
  return image->version;
}
