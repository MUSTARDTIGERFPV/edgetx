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

// Internal RF image manager (see radio_host.h).
//
// CRSF between EdgeTX and the images:
//   EdgeTX -> image: sendBuffer() (mixer task) -> the image's send hook
//   image -> EdgeTX: radioHostWriteFrame() -> fromImage -> copyRxBuffer()
//                    (telemetry), with the idle callback standing in for UART
//                    idle
// The "rfhost" task (priority 1, like Wingman's service task) switches
// images when the model asks for another one and runs the Crossfire image's
// services. Switching happens in task context, never from the UI.

#include "radio_host.h"
#include "elrs_host.h"
#include "crossfire_host.h"
#include "mlx90393_gimbal.h"

#include "edgetx.h"
#include "os/task.h"
#include "os/sleep.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RFHOST_STACK_SIZE  1024  // words: FatFS calls for the Crossfire SD service
#define RFHOST_TASK_PRIO   1

template <unsigned N>
struct ByteRing {
  static_assert((N & (N - 1)) == 0, "power of two");
  uint8_t buf[N];
  volatile uint32_t head = 0;  // written by the producer only
  volatile uint32_t tail = 0;  // written by the consumer only

  uint32_t count() const { return head - tail; }
  uint32_t space() const { return N - count(); }
  uint8_t peek(uint32_t i) const { return buf[(tail + i) & (N - 1)]; }
  void drop(uint32_t n) { tail = tail + n; }
  bool push(const uint8_t* d, uint32_t n)
  {
    if (space() < n) return false;
    uint32_t h = head;
    for (uint32_t i = 0; i < n; i++) buf[(h + i) & (N - 1)] = d[i];
    __DMB();
    head = h + n;
    return true;
  }
};

static ByteRing<512> toImage;    // CRSF from EdgeTX
static ByteRing<1024> fromImage; // CRSF from the image

static volatile RadioImage activeImage = RADIO_IMAGE_NONE;
static volatile bool poweringOff;
static_assert(RFHOST_TASK_PRIO < configMAX_PRIORITIES, "FreeRTOS priority out of range");

static char statusText[48] = "idle";

static task_handle_t rfhostTaskId;
// In SRAM, not CCM (CCM is full); FatFS + the SD service run on it
static StackType_t rfhostStack[RFHOST_STACK_SIZE] __attribute__((section(".ram"), aligned(8)));

// Vector table copy in SRAM (VTOR can't point at CCM). 98 vectors on F407.
static uint32_t ramVectors[128] __attribute__((section(".ram"), aligned(512)));
static const uint32_t* flashVectors;

// --- log ring for the CLI (`rf`) ----------------------------------------------

static char logBuf[1024];
static uint32_t logHead;  // total bytes ever written

static void logAppend(const char* s, int n)
{
  for (int i = 0; i < n; i++) logBuf[(logHead++) % sizeof(logBuf)] = s[i];
}

void radioHostLog(const char* fmt, ...)
{
  char b[96];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(b) - 2) n = sizeof(b) - 2;
  if (n == 0 || b[n - 1] != '\n') b[n++] = '\n';
  logAppend(b, n);
  TRACE_NOCRLF("[RF] %.*s", n, b);
}

uint32_t radioHostCopyLog(char* buf, uint32_t len)
{
  uint32_t avail = logHead < sizeof(logBuf) ? logHead : sizeof(logBuf);
  if (avail > len) avail = len;
  uint32_t start = logHead - avail;
  for (uint32_t i = 0; i < avail; i++) buf[i] = logBuf[(start + i) % sizeof(logBuf)];
  return avail;
}

const char* radioHostStatus() { return statusText; }
RadioImage radioHostActive() { return activeImage; }

const char* radioHostImageName(RadioImage img)
{
  switch (img) {
    case RADIO_IMAGE_ELRS: return "ELRS";
    case RADIO_IMAGE_CROSSFIRE: return "Crossfire";
    default: return "none";
  }
}

// --- vectors --------------------------------------------------------------------

void radioHostSetIrq(int irqn, void (*handler)())
{
  ramVectors[16 + irqn] = handler ? (uint32_t)handler : flashVectors[16 + irqn];
  __DSB();
}

// --- frames -----------------------------------------------------------------------

static void (*volatile sendHook)(const uint8_t*, uint32_t);

void radioHostSetSendHook(void (*hook)(const uint8_t*, uint32_t))
{
  sendHook = hook;
  toImage.tail = toImage.head;
}

bool radioHostQueueFrame(const uint8_t* data, uint32_t size)
{
  return toImage.push(data, size);
}

static bool isCrsfSync(uint8_t b)
{
  return b == 0xC8 || b == 0xEA || b == 0xEE || b == 0xEC;
}

uint16_t radioHostReadFrame(uint8_t* buf, uint16_t max)
{
  for (;;) {
    uint32_t n = toImage.count();
    if (n < 2) return 0;
    if (!isCrsfSync(toImage.peek(0))) { toImage.drop(1); continue; }
    uint32_t len = toImage.peek(1);
    if (len < 2 || len > 62) { toImage.drop(1); continue; }
    if (n < len + 2) return 0;
    if (len + 2 > max) { toImage.drop(len + 2); continue; }
    for (uint32_t i = 0; i < len + 2; i++) buf[i] = toImage.peek(i);
    toImage.drop(len + 2);
    return len + 2;
  }
}

static void (*rxIdleCb)(void*);
static void* rxIdleParam;

// Extended (device protocol) frames from the image, kept for the CLI
// (`rf cap`) so menus/bind can be driven from a PC
static ByteRing<1024> capture;

uint32_t radioHostCopyCapture(uint8_t* buf, uint32_t len)
{
  uint32_t n = capture.count();
  if (n > len) n = len;
  for (uint32_t i = 0; i < n; i++) buf[i] = capture.peek(i);
  capture.drop(n);
  return n;
}

bool radioHostInject(const uint8_t* frame, uint32_t len)
{
  auto hook = sendHook;
  if (!hook) return false;
  hook(frame, len);
  return true;
}

void radioHostWriteFrame(const uint8_t* frame, uint16_t len)
{
  if (len > 3 && frame[2] >= 0x28 && capture.space() >= len) capture.push(frame, len);
  fromImage.push(frame, len);  // full: this frame is dropped
  // Kick EdgeTX's reader whenever anything is waiting, even if this frame
  // didn't fit: it only drains on this callback, so skipping it when the
  // ring was full left telemetry dead for good (2026-11-11, after an RX
  // update burst filled the ring).
  // The callback is EdgeTX's ISR-side trigger (async_call_isr): it pends the
  // reader on the timer task, yields, and only then marks it queued. Called
  // from a task below the timer task (rfhost, prio 1) the yield ran the
  // reader before the mark, which then stuck at "queued" and blocked every
  // later trigger. In a critical section the yield waits, as in an ISR.
  auto cb = rxIdleCb;
  if (cb && fromImage.count()) {
    taskENTER_CRITICAL();
    cb(rxIdleParam);
    taskEXIT_CRITICAL();
  }
}

// --- switching ----------------------------------------------------------------------

static RadioImage wantedImage()
{
  if (poweringOff) return RADIO_IMAGE_NONE;
  const ModuleData& md = g_model.moduleData[INTERNAL_MODULE];
  if (md.type != MODULE_TYPE_CROSSFIRE) return RADIO_IMAGE_NONE;
  return md.subType == TANGO2_RF_CROSSFIRE ? RADIO_IMAGE_CROSSFIRE : RADIO_IMAGE_ELRS;
}

static void setStatus(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(statusText, sizeof(statusText), fmt, ap);
  va_end(ap);
  radioHostLog("%s", statusText);
}

static void stopActive()
{
  RadioImage old = activeImage;
  if (old == RADIO_IMAGE_NONE) return;
  uint32_t t0 = timersGetMsTick();
  radioHostSetSendHook(nullptr);
  if (old == RADIO_IMAGE_ELRS) elrsHostStop();
  else xfHostStop();
  activeImage = RADIO_IMAGE_NONE;
  if (old == RADIO_IMAGE_CROSSFIRE) mlxGimbalReclaim();  // the image had SPI2
  setStatus("%s stopped (%lu ms)", radioHostImageName(old),
            (unsigned long)(timersGetMsTick() - t0));
}

static void startImage(RadioImage img)
{
  uint32_t t0 = timersGetMsTick();
  bool ok;
  if (img == RADIO_IMAGE_ELRS) {
    ok = elrsHostStart();
  } else {
    mlxGimbalRelease();  // the Crossfire image reads the gimbals itself
    ok = xfHostStart();
    if (!ok) mlxGimbalReclaim();
  }
  if (ok) activeImage = img;
  setStatus("%s %s (%lu ms)", radioHostImageName(img),
            ok ? "running" : (img == RADIO_IMAGE_ELRS ? elrsHostStatus() : xfHostStatus()),
            (unsigned long)(timersGetMsTick() - t0));
}

static void rfhostTask()
{
  RadioImage failed = RADIO_IMAGE_NONE;  // don't retry a bad image in a loop
  for (;;) {
    RadioImage want = wantedImage();
    if (want != activeImage && want != failed) {
      stopActive();
      failed = RADIO_IMAGE_NONE;
      if (want != RADIO_IMAGE_NONE) {
        startImage(want);
        if (activeImage != want) failed = want;
      }
    }
    if (want != failed) failed = RADIO_IMAGE_NONE;
    if (activeImage == RADIO_IMAGE_CROSSFIRE && !xfHostPoll()) {
      stopActive();
      failed = RADIO_IMAGE_CROSSFIRE;
    }
    sleep_ms(1);
  }
}

void radioHostInit()
{
  flashVectors = (const uint32_t*)SCB->VTOR;
  memcpy(ramVectors, flashVectors, sizeof(ramVectors));
  SCB->VTOR = (uint32_t)ramVectors;
  __DSB();
  xfHostInit();
  task_create(&rfhostTaskId, rfhostTask, "rfhost", rfhostStack,
              RFHOST_STACK_SIZE, RFHOST_TASK_PRIO);
}

// boardOff: the rfhost task stops the image (it may be mid-switch, and it
// runs the Crossfire SD service the settings save needs); wait for it.
void radioHostPowerOff()
{
  poweringOff = true;
  if (!scheduler_is_running()) return;
  uint32_t t0 = timersGetMsTick();
  while (activeImage != RADIO_IMAGE_NONE && timersGetMsTick() - t0 < 4000) {
    WDG_RESET();
    sleep_ms(5);
  }
}

// --- virtual serial port (internal module) ------------------------------------------

static uint32_t virtualBaudrate;

static void* vs_init(void*, const etx_serial_init* params)
{
  virtualBaudrate = params->baudrate;
  toImage.tail = toImage.head;  // consumer side is the image; just resync
  fromImage.tail = fromImage.head;
  return (void*)&virtualBaudrate;  // any non-null context
}

static void vs_deinit(void*) { rxIdleCb = nullptr; }

static void vs_sendBuffer(void*, const uint8_t* data, uint32_t size)
{
  auto hook = sendHook;
  if (hook) hook(data, size);
}

static void vs_sendByte(void* ctx, uint8_t byte) { vs_sendBuffer(ctx, &byte, 1); }
static bool vs_txCompleted(void*) { return true; }
static void vs_waitForTxCompleted(void*) {}
static void vs_enableRx(void*) {}

static int vs_getByte(void*, uint8_t* data)
{
  if (!fromImage.count()) return 0;
  *data = fromImage.peek(0);
  fromImage.drop(1);
  return 1;
}

static int vs_getLastByte(void*, uint32_t idx, uint8_t* data)
{
  uint32_t n = fromImage.count();
  if (idx == 0 || idx > n) return 0;
  *data = fromImage.peek(n - idx);
  return 1;
}

static int vs_getBufferedBytes(void*) { return fromImage.count(); }

static int vs_copyRxBuffer(void*, uint8_t* buf, uint32_t len)
{
  uint32_t n = fromImage.count();
  if (n > len) n = len;
  for (uint32_t i = 0; i < n; i++) buf[i] = fromImage.peek(i);
  fromImage.drop(n);
  return n;
}

static void vs_clearRxBuffer(void*) { fromImage.tail = fromImage.head; }
static uint32_t vs_getBaudrate(void*) { return virtualBaudrate; }
static void vs_setBaudrate(void*, uint32_t baudrate) { virtualBaudrate = baudrate; }
static void vs_setPolarity(void*, uint8_t) {}
static void vs_setHWOption(void*, uint32_t) {}
static void vs_setReceiveCb(void*, void (*)(uint8_t*, uint32_t)) {}

static void vs_setIdleCb(void*, void (*on_idle)(void*), void* param)
{
  rxIdleParam = param;
  rxIdleCb = on_idle;
}

static void vs_setBaudrateCb(void*, void (*)(uint32_t)) {}

const etx_serial_driver_t radioVirtualSerialDriver = {
  .init = vs_init,
  .deinit = vs_deinit,
  .sendByte = vs_sendByte,
  .sendBuffer = vs_sendBuffer,
  .txCompleted = vs_txCompleted,
  .waitForTxCompleted = vs_waitForTxCompleted,
  .enableRx = vs_enableRx,
  .getByte = vs_getByte,
  .getLastByte = vs_getLastByte,
  .getBufferedBytes = vs_getBufferedBytes,
  .copyRxBuffer = vs_copyRxBuffer,
  .clearRxBuffer = vs_clearRxBuffer,
  .getBaudrate = vs_getBaudrate,
  .setBaudrate = vs_setBaudrate,
  .setPolarity = vs_setPolarity,
  .setHWOption = vs_setHWOption,
  .setReceiveCb = vs_setReceiveCb,
  .setIdleCb = vs_setIdleCb,
  .setBaudrateCb = vs_setBaudrateCb,
};
