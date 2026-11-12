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

// Host for the TBS Crossfire TX image ("API 0x103"), reconstructed from how
// Wingman's EdgeTX port hosts it.
//
// - The image is started by calling its reset handler from a task (prio 4,
//   the highest we have; Wingman uses 5). The handler moves SP to 0x20020000, so the image runs on
//   its own stack in 0x20015000-0x2001FFFF under our scheduler, and blocks
//   only through the wait()/clear() callbacks below.
// - Shared struct at CCM 0x10000000 (the linker keeps EdgeTX out of it):
//   flags, two 256-byte CRSF rings, the four calibrated sticks and the 16
//   channel outputs.
// - Its three IRQs (EXTI15_10, TIM13 sharing IRQ 44 with our TIM8, DMA1
//   Stream2) go through trampolines in the RAM vector table to the handlers
//   it registers in the callback table.
// - It never returns and has no stop entry: stopping is a save + power-off
//   handshake, then deleting its task and resetting everything it drives.
// Only Crossfire 6.30 is accepted: 6.13 reprograms the PLL and SysTick.

#include "crossfire_host.h"
#include "radio_host.h"
#include "elrs_image.h"   // CROSSFIRE_IMAGE_ADDR

#include "edgetx.h"
#include "os/task.h"
#include "os/sleep.h"
#include "hal/usb_driver.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Wingman runs it at 5, above its mixer; our FreeRTOS has priorities 0..4
// (configMAX_PRIORITIES 5, and 5 trips configASSERT), so the top one, shared
// with the mixer. The image blocks in wait() on every loop pass.
#define XF_TASK_PRIO       4
#define XF_STACK_SIZE      64    // words: the image moves SP off it at once
#define XF_API_VERSION     0x103
#define XF_RAM_START       0x20015000u
#define XF_RAM_END         0x20020000u
#define XF_SAVE_TIMEOUT    1000  // ms, EEPROM_SAVE handshake (Wingman: 1 s)
#define XF_POWEROFF_TIMEOUT 2000 // ms, POWER_OFF handshake (Wingman: 2 s)

// The 6.30 builds we can host. Each one's .data holds the struct pointer,
// initialised from `structPtr`; that word is how we tell them apart.
// rawGimbals: the six s16 the image's MLX90393 reader (0x080C2CE8) keeps,
// at its context struct + 0x40.
struct xf_build_t {
  uint32_t structPtr;
  volatile const int16_t* rawGimbals;
};
static const xf_build_t xfBuilds[] = {
  {0x080DDF90u, (volatile const int16_t*)0x200188D0u},  // 0x000141xx 6.30: PA_BOOST
  {0x080DDEB0u, (volatile const int16_t*)0x200188E0u},  // 0x000140xx 6.30: RFO
};
static const xf_build_t* xfBuild;  // set by imageOk()

// Flags word (+0x14)
#define XF_FLAG_REBOOT       (1u << 0)  // image: shut down and parked
#define XF_FLAG_EEPROM_SAVE  (1u << 1)  // host sets, image clears when saved
#define XF_FLAG_POWER_OFF    (1u << 3)  // host sets, image clears when RF is off
#define XF_FLAG_RF_OFF       (1u << 4)  // host: USB storage mode
#define XF_FLAG_API_MISMATCH (1u << 5)  // image

struct xf_shared_t {
  uint32_t api;                 // 0x000  must be 0x103
  volatile uint32_t* table;     // 0x004  callback table (6 words)
  uint8_t eventMap[10];         // 0x008  image event i -> host event map[i]
  uint8_t pad0[2];
  volatile uint32_t flags;      // 0x014
  uint8_t reserved0[0x40];      // 0x018
  uint8_t ringA[256];           // 0x058  image -> host
  volatile uint32_t ringAWrite; // 0x158  image
  volatile uint32_t ringARead;  // 0x15C  host
  uint8_t ringB[256];           // 0x160  host -> image
  volatile uint32_t ringBWrite; // 0x260  host
  volatile uint32_t ringBRead;  // 0x264  image
  uint8_t reserved1[0x100];     // 0x268
  volatile int16_t sticks[4];   // 0x368  calibrated, centre 0
  volatile uint8_t stickState;  // 0x370  bits 0-4 state, 5-6 calibration mode
  uint8_t pad1[3];
  volatile int32_t channels[16];// 0x374  EdgeTX units, -1024..+1024
};
static_assert(sizeof(xf_shared_t) == 0x3B4, "API 0x103 struct size");
static_assert(offsetof(xf_shared_t, flags) == 0x14, "flags");
static_assert(offsetof(xf_shared_t, ringAWrite) == 0x158, "ring A");
static_assert(offsetof(xf_shared_t, ringBRead) == 0x264, "ring B");
static_assert(offsetof(xf_shared_t, sticks) == 0x368, "sticks");
static_assert(offsetof(xf_shared_t, channels) == 0x374, "channels");

static xf_shared_t* const S = (xf_shared_t*)0x10000000u;

// Callback table: [0] EXTI15_10, [1] TIM13, [5] DMA1 Stream2 (image),
// [3] wait, [4] clear (host)
static volatile uint32_t xfTable[6];

static task_handle_t xfTaskId;
TASK_DEFINE_STACK(xfStack, XF_STACK_SIZE);
static volatile TaskHandle_t xfTask;  // non-null while IRQs may notify it

static volatile uint32_t eventBits;   // host events 1..9
static StaticSemaphore_t eventSemBuf;
static SemaphoreHandle_t eventSem;    // wakes xf_wait() when a bit is set
static uint32_t savedTim8Prio;
static_assert(XF_TASK_PRIO < configMAX_PRIORITIES, "FreeRTOS priority out of range");

static char statusText[48] = "not started";

static void setStatus(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(statusText, sizeof(statusText), fmt, ap);
  va_end(ap);
  radioHostLog("[xf host] %s", statusText);
}

const char* xfHostStatus() { return statusText; }

static bool usbStorageModeActive()
{
  return usbPlugged() && getSelectedUsbMode() == USB_MASS_STORAGE_MODE;
}

// --- callbacks the image calls ------------------------------------------------
// Return 0 when the event happened, 5 on timeout (Wingman's values).

static int xf_wait(uint8_t ev, uint32_t ticks)
{
  if (ticks == 0) ticks = portMAX_DELAY;
  if (ev == 0)  // IRQ wake-up: the trampolines notify the task
    return ulTaskNotifyTake(pdTRUE, ticks) ? 0 : 5;
  // Woken by setEvent() through eventSem, so a reply is picked up at once
  // (a 1-tick poll here made the bind-time RX update too slow for its window)
  uint32_t bit = 1u << ev;
  TickType_t t0 = xTaskGetTickCount();
  while (!(eventBits & bit)) {
    TickType_t gone = xTaskGetTickCount() - t0;
    if (gone >= ticks) return 5;
    xSemaphoreTake(eventSem, ticks - gone);
  }
  return 0;
}

static void xf_clear(uint8_t ev)
{
  if (ev == 0) {
    ulTaskNotifyValueClear(nullptr, 0xFFFFFFFFu);
  } else {
    taskENTER_CRITICAL();
    eventBits &= ~(1u << ev);
    taskEXIT_CRITICAL();
  }
}

static void setEvent(uint8_t ev)
{
  taskENTER_CRITICAL();
  eventBits |= 1u << ev;
  taskEXIT_CRITICAL();
  xSemaphoreGive(eventSem);
}

// --- IRQ trampolines --------------------------------------------------------------

static void notifyFromIsr()
{
  TaskHandle_t t = xfTask;
  if (!t) return;
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(t, &woken);
  portYIELD_FROM_ISR(woken);
}

static void xf_exti15_10_irq()
{
  auto f = (void (*)())xfTable[0];
  if (f) {
    f();
    notifyFromIsr();
  } else {
    EXTI->PR = 0xFC00;
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler();  // EdgeTX: external module pulses

static void xf_tim8_tim13_irq()
{
  TIM8_UP_TIM13_IRQHandler();
  if (TIM13->SR & TIM_SR_UIF) {
    TIM13->SR = ~TIM_SR_UIF;
    auto f = (void (*)())xfTable[1];
    if (f) f();
    notifyFromIsr();
  }
}

// NVIC priority 0 (the image sets it): no RTOS calls in here
static void xf_dma1_stream2_irq()
{
  if (DMA1->LISR & DMA_LISR_TCIF2) {
    DMA1->LIFCR = DMA_LIFCR_CTCIF2;
    auto f = (void (*)())xfTable[5];
    if (f) f();
  }
}

// --- rings -------------------------------------------------------------------------

// host -> image, whole frames or nothing, first byte forced to 0xC8 (Wingman).
// Two producers (mixer task via the send hook, rfhost task): critical section.
static bool ringBPush(const uint8_t* f, uint32_t n)
{
  bool ok = false;
  taskENTER_CRITICAL();
  uint32_t w = S->ringBWrite & 0xFF;
  uint32_t r = S->ringBRead & 0xFF;
  if (n <= ((r - w - 1) & 0xFF)) {
    for (uint32_t i = 0; i < n; i++) S->ringB[(w + i) & 0xFF] = i ? f[i] : 0xC8;
    __DMB();
    S->ringBWrite = (w + n) & 0xFF;
    ok = true;
  }
  taskEXIT_CRITICAL();
  return ok;
}

static void sdService(const uint8_t* frame, uint32_t len);

// image -> host: whole CRSF frames to EdgeTX, 0x3A (SD service) to us
static void pumpRingA()
{
  uint8_t frame[64];
  for (;;) {
    uint32_t r = S->ringARead & 0xFF;
    uint32_t avail = (S->ringAWrite - r) & 0xFF;
    if (avail < 2) return;
    uint32_t len = S->ringA[(r + 1) & 0xFF];
    if (len < 2 || len > 62) {  // not a frame start: resync
      S->ringARead = (r + 1) & 0xFF;
      continue;
    }
    if (avail < len + 2) return;
    // 0x3A: the SD file service, except sub 0x10 to the handset, which is
    // the CRSF timing frame EdgeTX's mixer scheduler syncs to
    uint8_t type = S->ringA[(r + 2) & 0xFF];
    bool sd = type == 0x3A && !(S->ringA[(r + 3) & 0xFF] == 0xEA &&
                                S->ringA[(r + 5) & 0xFF] == 0x10);
    // One SD request at a time, and only once the image has taken our last
    // reply, as Wingman's pump does. Answering a burst of pipelined reads
    // lets a reply and its "reply ready" event land just as the image clears
    // the event after draining ring B: the reply then sits unread, the image
    // times out after 100 ms and restarts the block, and can lose the same
    // race on every retry (bind-time RX update stalled on one block).
    if (sd && S->ringBWrite != S->ringBRead) return;
    for (uint32_t i = 0; i < len + 2; i++) frame[i] = S->ringA[(r + i) & 0xFF];
    __DMB();
    S->ringARead = (r + len + 2) & 0xFF;
    if (sd) {
      // the image (higher priority) takes the reply as soon as its event is
      // set, so the next request is usually waiting already
      sdService(frame, len + 2);
      continue;
    }
    radioHostWriteFrame(frame, len + 2);
  }
}

// One frame from EdgeTX. RC frames don't go through the ring: the image
// reads the channels from the struct (as Wingman feeds them).
static void sendFrame(const uint8_t* data, uint32_t size)
{
  // 26 bytes, or 27 with the arming status byte this EdgeTX appends
  if (data[2] == 0x16 && size >= 26) {
    // 16 x 11 bits, LSB first; EdgeTX packed v*4/5 + 992
    uint32_t acc = 0;
    int bits = 0, ch = 0;
    for (uint32_t i = 3; i < 25 && ch < 16; i++) {
      acc |= (uint32_t)data[i] << bits;
      bits += 8;
      while (bits >= 11 && ch < 16) {
        int32_t v = (int32_t)(acc & 0x7FF) - 992;
        S->channels[ch++] = v * 5 / 4;
        acc >>= 11;
        bits -= 11;
      }
    }
    return;
  }
  ringBPush(data, size);
}

// EdgeTX's CRSF driver (mixer task): one buffer can hold several frames
// (RC channels plus whatever it queued for the module)
static void sendHook(const uint8_t* data, uint32_t size)
{
  while (size >= 4) {
    uint32_t n = data[1] + 2u;
    if (data[1] < 2 || n > size) return;
    sendFrame(data, n);
    data += n;
    size -= n;
  }
}

// --- SD file service -------------------------------------------------------------
// CRSF 0x3A frames from the image: [sync len 0x3A dest origin sub fields.. crc],
// integers big-endian, crc8 poly 0xD5 over type..payload. Files under
// /CROSSFIRE/ only (EEPROM.bin = its settings, OTA/ = receiver firmware).
// Replies, return codes and the "reply ready" event (host event 1) mirror
// Wingman's handler.

#define XF_EVENT_REPLY  1
#define XF_SD_DATA      57   // data bytes in every Wingman data reply
#define XF_SD_CHUNK     44   // file bytes per read/write frame
#define XF_SD_PATH      53

static FIL sdFile;
static bool sdFileOpen;
static char sdPath[XF_SD_PATH + 1];
static uint32_t sdPos;

static uint8_t crc8d5(const uint8_t* p, uint32_t n)
{
  uint8_t crc = 0;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
  }
  return crc;
}

static uint32_t be32(const uint8_t* p) { return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static uint16_t be16(const uint8_t* p) { return (p[0] << 8) | p[1]; }
static void putBe32(uint8_t* p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void putBe16(uint8_t* p, uint16_t v) { p[0] = v >> 8; p[1] = v; }

// Retried for up to 301 ms while ring B is full, as Wingman does
static void sdTraceLine(const char* fmt, ...);

static void sdReply(uint8_t dest, uint8_t sub, const uint8_t* d, uint32_t n)
{
  uint8_t f[64];
  f[0] = 0xC8;
  f[1] = n + 5;
  f[2] = 0x3A;
  f[3] = dest;
  f[4] = 0xEA;
  f[5] = sub;
  memcpy(f + 6, d, n);
  f[6 + n] = crc8d5(f + 2, n + 4);
  uint32_t t0 = timersGetMsTick();
  while (!ringBPush(f, n + 7)) {
    if (timersGetMsTick() - t0 > 301) {
      sdTraceLine("    reply sub %u dropped (ring B full 301 ms)", sub);
      return;
    }
    sleep_ms(1);
  }
  uint32_t waited = timersGetMsTick() - t0;
  if (waited > 5) sdTraceLine("    reply sub %u waited %lu ms for ring B", sub, (unsigned long)waited);
}

static bool sdPathOk(const uint8_t* p)
{
  char s[XF_SD_PATH + 1];
  memcpy(s, p, XF_SD_PATH);
  s[XF_SD_PATH] = 0;
  return !strncmp(s, "/CROSSFIRE/", 11) && !strstr(s, "..");
}

static void sdClose()
{
  if (sdFileOpen) f_close(&sdFile);
  sdFileOpen = false;
}

static bool sdEnsureOpen()
{
  if (sdFileOpen) return true;
  if (f_open(&sdFile, sdPath, FA_READ | FA_WRITE | FA_OPEN_ALWAYS) != FR_OK) return false;
  sdFileOpen = true;
  sdPos = 0;
  return true;
}

static bool sdSeek(uint32_t pos)
{
  if (sdPos == pos) return true;
  if (f_lseek(&sdFile, pos) != FR_OK) return false;
  sdPos = pos;
  return true;
}

// f_mkdir at every '/' after the first component
static void sdMkdirs(const char* path)
{
  char d[XF_SD_PATH + 1];
  for (int i = 1; path[i]; i++) {
    if (path[i] != '/') continue;
    memcpy(d, path, i);
    d[i] = 0;
    f_mkdir(d);
  }
}

// Returns true when the image's "reply ready" event should be set
static bool sdHandle(const uint8_t* f, uint32_t total)
{
  if (total <= 6) return false;
  if (crc8d5(f + 2, total - 3) != f[total - 1]) {
    radioHostLog("[xf sd] bad CRC");
    return false;
  }
  uint8_t sub = f[5], org = f[4];
  const uint8_t* p = f + 6;

  if (sub == 7) {
    uint8_t m = sdMounted() ? 1 : 0;
    sdReply(org, 7, &m, 1);
    return true;
  }
  if (!sdMounted() || usbStorageModeActive()) {
    sdFileOpen = false;  // the file object is abandoned with the card
    return sub != 3 && sub != 4;
  }

  switch (sub) {
    case 1: {  // open
      if (total < 64) return true;
      if (!sdPathOk(p)) {
        sdPath[0] = 0;
        return true;
      }
      memcpy(sdPath, p, XF_SD_PATH);
      sdPath[XF_SD_PATH] = 0;
      sdMkdirs(sdPath);
      sdClose();
      if (!sdEnsureOpen()) return true;
      uint8_t d[XF_SD_DATA];
      memcpy(d, p, XF_SD_PATH);
      putBe32(d + XF_SD_PATH, f_size(&sdFile));
      sdReply(org, 1, d, sizeof(d));
      return true;
    }
    case 2:  // close, no reply
      sdClose();
      return true;
    case 3:
    case 4: {  // read / write one chunk of the transfer [addr, addr+size)
      if (total < (sub == 3 ? 20u : 64u) || !sdPath[0]) return true;
      uint32_t addr = be32(p), size = be32(p + 4), chunk = be32(p + 8);
      if (addr > chunk || size <= chunk - addr) return true;
      if (!sdEnsureOpen()) return false;
      uint32_t n = addr + size - chunk;
      if (n > XF_SD_CHUNK) n = XF_SD_CHUNK;
      if (sub == 3) {
        uint8_t d[XF_SD_DATA] = {};
        UINT br = 0;
        bool good = f_size(&sdFile) >= addr + size && sdSeek(chunk) &&
                    f_read(&sdFile, d + 13, n, &br) == FR_OK;
        if (!good) {
          sdClose();
          return false;
        }
        sdPos = chunk + br;
        putBe32(d, addr);
        putBe32(d + 4, size);
        putBe32(d + 8, chunk);
        d[12] = 1;
        sdReply(org, 3, d, sizeof(d));
      } else {
        UINT bw = 0;
        bool good = sdSeek(chunk) && f_write(&sdFile, p + 13, n, &bw) == FR_OK &&
                    bw == n;
        if (!good) {
          sdClose();
          return false;
        }
        sdPos = chunk + bw;
        uint8_t d[4];
        putBe32(d, chunk);
        sdReply(org, 5, d, sizeof(d));
      }
      return true;
    }
    case 6: {  // erase, no reply
      if (total < 60 || !sdPathOk(p)) return true;
      char s[XF_SD_PATH + 1];
      memcpy(s, p, XF_SD_PATH);
      s[XF_SD_PATH] = 0;
      if (sdFileOpen && !strcmp(s, sdPath)) sdClose();
      f_unlink(s);
      return true;
    }
    case 8: {  // list: index 0 = entry count, entries counted from 1
      // The image NUL-terminates the path, so accept short frames too
      // (Wingman needs 64 bytes and ignores shorter ones).
      if (total < 10) return true;
      uint16_t want = be16(p);
      char dir[52] = {};
      uint32_t plen = total - 1 - 12;  // path bytes up to the CRC
      memcpy(dir, f + 12, plen < 51 ? plen : 51);
      char name[51] = {};
      uint32_t fsz = 0;
      uint16_t cnt = 0;
      DIR dj;
      if (!strncmp(dir, "/CROSSFIRE", 10) && (dir[10] == 0 || dir[10] == '/') &&
          !strstr(dir, "..") && f_opendir(&dj, dir) == FR_OK) {
        FILINFO fno;
        while (f_readdir(&dj, &fno) == FR_OK && fno.fname[0]) {
          cnt++;
          if (want && cnt == want) {
            strncpy(name, fno.fname, 50);
            fsz = fno.fsize;
            break;
          }
        }
        f_closedir(&dj);
        if (want && cnt != want) cnt = 0;
      }
      uint8_t d[XF_SD_DATA] = {};
      putBe16(d, cnt);
      putBe32(d + 2, fsz);
      memcpy(d + 6, name, 51);
      sdReply(org, 8, d, sizeof(d));
      return true;
    }
    default:
      return true;
  }
}

// --- SD service trace (CLI `rf sdlog`) ---------------------------------------
// One line per request, except that consecutive read/write chunks of one
// transfer are merged into a single "read/write <file> [first..end)" line.

static char sdTrace[6144] __attribute__((section(".ram")));  // SRAM, not CCM
static uint32_t sdTraceHead;  // total bytes ever written
static struct {
  uint8_t sub;     // 3 or 4 while a run is open, else 0
  uint32_t first, end, count, t0;
  bool failed;
} sdRun;

static void sdTraceLine(const char* fmt, ...)
{
  char b[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof(b) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(b) - 2) n = sizeof(b) - 2;
  b[n++] = 10;  // newline
  for (int i = 0; i < n; i++) sdTrace[(sdTraceHead++) % sizeof(sdTrace)] = b[i];
}

static void sdTraceFlushRun()
{
  if (!sdRun.sub) return;
  sdTraceLine("%lu %s %s [%lu..%lu) %lu chunks%s", (unsigned long)sdRun.t0,
              sdRun.sub == 3 ? "read " : "write", sdPath, (unsigned long)sdRun.first,
              (unsigned long)sdRun.end, (unsigned long)sdRun.count,
              sdRun.failed ? " FAILED" : "");
  sdRun.sub = 0;
}

uint32_t xfHostCopySdTrace(char* buf, uint32_t len)
{
  sdTraceFlushRun();
  uint32_t avail = sdTraceHead < sizeof(sdTrace) ? sdTraceHead : sizeof(sdTrace);
  if (avail > len) avail = len;
  uint32_t start = sdTraceHead - avail;
  for (uint32_t i = 0; i < avail; i++) buf[i] = sdTrace[(start + i) % sizeof(sdTrace)];
  return avail;
}

static void sdService(const uint8_t* frame, uint32_t len)
{
  bool ok = sdHandle(frame, len);
  if (ok) setEvent(XF_EVENT_REPLY);

  uint8_t sub = frame[5];
  uint32_t now = timersGetMsTick();
  if ((sub == 3 || sub == 4) && len >= 18) {
    uint32_t addr = be32(frame + 6), size = be32(frame + 10), chunk = be32(frame + 14);
    uint32_t n = addr + size - chunk;
    if (n > XF_SD_CHUNK) n = XF_SD_CHUNK;
    if (sdRun.sub != sub || chunk != sdRun.end) {
      sdTraceFlushRun();
      sdRun = {sub, chunk, chunk, 0, now, false};
      sdTraceLine("%lu %s transfer addr %lu size %lu", (unsigned long)now,
                  sub == 3 ? "read " : "write", (unsigned long)addr, (unsigned long)size);
    }
    sdRun.end = chunk + n;
    sdRun.count++;
    if (!ok) sdRun.failed = true;
    return;
  }
  sdTraceFlushRun();
  char path[XF_SD_PATH + 1] = "";
  if ((sub == 1 || sub == 6) && len > 6) {
    uint32_t n = len - 7 < XF_SD_PATH ? len - 7 : XF_SD_PATH;
    memcpy(path, frame + 6, n);
  } else if (sub == 8 && len > 13) {
    uint32_t n = len - 13 < XF_SD_PATH ? len - 13 : XF_SD_PATH;
    memcpy(path, frame + 12, n);
  }
  static const char* const names[] = {"?", "open", "close", "read", "write", "?5", "erase", "query", "list"};
  sdTraceLine("%lu %s %s%s%s", (unsigned long)now, sub < 9 ? names[sub] : "?", path,
              sub == 1 && sdFileOpen ? " size " : "", "");
  if (sub == 1 && sdFileOpen)
    sdTraceLine("    size %lu", (unsigned long)f_size(&sdFile));
  if (sub == 1 || sub == 6) radioHostLog("[xf sd] %s %s", names[sub], path);
  if (!ok) sdTraceLine("    -> no reply");
}

// --- lifecycle -----------------------------------------------------------------------

static void xfTaskBody()
{
  uint32_t entry = *(const uint32_t*)(CROSSFIRE_IMAGE_ADDR + 4);
  ((void (*)())entry)();
  for (;;) sleep_ms(1000);  // the image never returns
}

static bool imageOk()
{
  const uint32_t* v = (const uint32_t*)CROSSFIRE_IMAGE_ADDR;
  if (v[0] != XF_RAM_END || v[1] != CROSSFIRE_IMAGE_ADDR + 0x21 ||
      v[8] == 0xFFFFFFFFu) {
    setStatus("no Crossfire image");
    return false;
  }
  for (const auto& b : xfBuilds) {
    if (*(const uint32_t*)b.structPtr == (uint32_t)S) {
      xfBuild = &b;
      return true;
    }
  }
  setStatus("Crossfire build not 6.30");
  return false;
}

void xfHostInit()
{
  eventSem = xSemaphoreCreateBinaryStatic(&eventSemBuf);
  // Until an image registers, the trampolines only clear their flags
  radioHostSetIrq(DMA1_Stream2_IRQn, xf_dma1_stream2_irq);
}

bool xfHostStart()
{
  if (!imageOk()) return false;

  memset((void*)XF_RAM_START, 0, XF_RAM_END - XF_RAM_START);
  memset((void*)S, 0, sizeof(*S));
  for (auto& t : xfTable) t = 0;
  eventBits = 0;
  S->api = XF_API_VERSION;
  for (int i = 0; i < 10; i++) S->eventMap[i] = i;
  xfTable[3] = (uint32_t)xf_wait;
  xfTable[4] = (uint32_t)xf_clear;
  S->table = xfTable;

  savedTim8Prio = NVIC_GetPriority(TIM8_UP_TIM13_IRQn);
  radioHostSetIrq(EXTI15_10_IRQn, xf_exti15_10_irq);
  radioHostSetIrq(TIM8_UP_TIM13_IRQn, xf_tim8_tim13_irq);
  radioHostSetIrq(DMA1_Stream2_IRQn, xf_dma1_stream2_irq);

  radioHostSetSendHook(sendHook);
  task_create(&xfTaskId, xfTaskBody, "crsf", xfStack, XF_STACK_SIZE,
              XF_TASK_PRIO);
  xfTask = xfTaskId._rtos_handle;

  // It registers its IRQ handlers early in main()
  uint32_t t0 = timersGetMsTick();
  while (!xfTable[0] && !(S->flags & XF_FLAG_REBOOT)) {
    if (timersGetMsTick() - t0 > 500) break;
    sleep_ms(1);
  }
  if (!xfTable[0] || (S->flags & XF_FLAG_REBOOT)) {
    setStatus("image did not start (flags %02lx)", (unsigned long)S->flags);
    xfHostStop();
    return false;
  }
  setStatus("running");
  return true;
}

static void setFlag(uint32_t bit, bool on)
{
  taskENTER_CRITICAL();  // the image task (prio 5) can't run in here
  if (on) S->flags |= bit;
  else S->flags &= ~bit;
  taskEXIT_CRITICAL();
}

// Host sets the bit, the image clears it when done. Keeps the SD service
// running, as the settings save needs it.
static bool handshake(uint32_t bit, uint32_t timeoutMs)
{
  setFlag(bit, true);
  uint32_t t0 = timersGetMsTick();
  while (S->flags & bit) {
    if (timersGetMsTick() - t0 > timeoutMs) {
      setFlag(bit, false);
      return false;
    }
    pumpRingA();
    WDG_RESET();
    sleep_ms(1);
  }
  return true;
}

bool xfHostPoll()
{
  uint32_t flags = S->flags;
  if (flags & XF_FLAG_REBOOT) {
    setStatus(flags & XF_FLAG_API_MISMATCH ? "image: API mismatch"
                                           : "image parked (asked for reboot)");
    return false;
  }
  bool msc = usbStorageModeActive();
  if (msc != !!(flags & XF_FLAG_RF_OFF)) setFlag(XF_FLAG_RF_OFF, msc);
  pumpRingA();
  return true;
}

bool xfHostSticks(int16_t out[4])
{
  if (!xfTask || (S->stickState & 0x1F) <= 10) return false;
  for (int i = 0; i < 4; i++) out[i] = S->sticks[i];
  return true;
}

// 6.30 keeps the last MLX90393 reads as (raw - 0x8000) * k, k = the sensor's
// uT/LSB. Six s16, sensor 0 X Y Z, sensor 1 X Y Z.

bool xfHostRawGimbals(int16_t xyz[2][3])
{
  if (!xfTask) return false;
  static const float k[3] = {1.573f, 1.573f, 2.534f};
  bool any = false;
  for (int s = 0; s < 2; s++) {
    for (int a = 0; a < 3; a++) {
      int16_t v = xfBuild->rawGimbals[s * 3 + a];
      any = any || v;
      xyz[s][a] = (int16_t)(v / k[a]);
    }
  }
  return any;  // all zero until the image's first read
}

static void rccReset(volatile uint32_t* rst, volatile uint32_t* en, uint32_t bits)
{
  *rst |= bits;
  (void)*rst;
  *rst &= ~bits;
  *en &= ~bits;
}

void xfHostStop()
{
  radioHostSetSendHook(nullptr);
  if (xfTask && xfTable[0] && !(S->flags & XF_FLAG_REBOOT)) {
    bool saved = sdMounted() && !usbStorageModeActive() &&
                 handshake(XF_FLAG_EEPROM_SAVE, XF_SAVE_TIMEOUT);
    bool off = handshake(XF_FLAG_POWER_OFF, XF_POWEROFF_TIMEOUT);
    radioHostLog("[xf host] save %s, power off %s", saved ? "ok" : "skipped",
                 off ? "ok" : "timed out");
  }

  // Interrupts first, so nothing calls into the image or notifies its task
  NVIC_DisableIRQ(EXTI15_10_IRQn);
  NVIC_DisableIRQ(DMA1_Stream2_IRQn);
  NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  NVIC_DisableIRQ(TIM5_IRQn);
  NVIC_DisableIRQ(UART4_IRQn);
  NVIC_DisableIRQ(I2C1_EV_IRQn);
  NVIC_DisableIRQ(I2C1_ER_IRQn);
  TIM13->DIER = 0;
  TIM13->CR1 = 0;
  for (auto& t : xfTable) t = 0;
  TaskHandle_t t = xfTask;
  xfTask = nullptr;
  if (t) vTaskDelete(t);

  // Peripherals: DMA streams off, then reset everything else it drives
  DMA1_Stream2->CR = 0;
  DMA1_Stream6->CR = 0;
  while (DMA1_Stream2->CR & DMA_SxCR_EN) {}
  while (DMA1_Stream6->CR & DMA_SxCR_EN) {}
  DMA1->LIFCR = 0x3Du << 16;  // stream 2 flags
  DMA1->HIFCR = 0x3Du << 16;  // stream 6 flags
  EXTI->IMR &= ~0xFC00u;
  EXTI->EMR &= ~0xFC00u;
  EXTI->RTSR &= ~0xFC00u;
  EXTI->FTSR &= ~0xFC00u;
  EXTI->PR = 0xFC00u;
  rccReset(&RCC->APB1RSTR, &RCC->APB1ENR,
           RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM5RST | RCC_APB1RSTR_TIM13RST |
           RCC_APB1RSTR_UART4RST | RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_SPI2RST);
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
  NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);

  radioHostSetIrq(EXTI15_10_IRQn, nullptr);
  radioHostSetIrq(TIM8_UP_TIM13_IRQn, nullptr);
  NVIC_SetPriority(TIM8_UP_TIM13_IRQn, savedTim8Prio);

  // Pins back to the parked state (SX1272 deselected, PA off, WiFi off...)
  tango2ParkRadioHardware();

  memset((void*)XF_RAM_START, 0, XF_RAM_END - XF_RAM_START);
  memset((void*)S, 0, sizeof(*S));
  setStatus("stopped");
}
