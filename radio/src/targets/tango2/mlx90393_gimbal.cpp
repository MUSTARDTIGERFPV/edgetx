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

// MLX90393 gimbal driver skeleton for the TBS Tango 2.
//
// Sensor handling follows TBS's Crossfire image (0x080c4aa0..0x080c4f5c):
//   init:  EX (0x80), RT (0xF0), ~84 ms, WR reg0=0x0000 reg1=0x8400
//          reg2=0x02A4 reg4/5/6=0x8000
//   loop:  SM 0x3F (ZYXT) to both, wait for data-ready, RM 0x4F -> status,
//          T, X, Y, Z (big endian, 0x8000 offset), ~300 Hz.
// Here both sensors run in burst mode instead (SB 0x1E), X, Y, Z only: the
// angles don't use temperature. A sensor starts its next conversion only once
// the last one is read, so an 8 kHz timer (TIM7) reads each result as it
// lands: 753 samples/s per sensor, at most ~1.45 ms old when the mixer maps it
// (was 200/s and ~5 ms with single measurements started from the mixer).
// Bench, sticks still (tools/gimbal_noise.py): HALLCONF 0 vs 0xC changes the
// gain (+29%) but not the rate; DIG_FILT 0 gives 885/s at 1.6x the angle
// noise (0.026 vs 0.016 deg sd). TBS's HALLCONF 0 / DIG_FILT 1 are kept.
//
// The stick mapping (mlxMapToSticks) comes from SWD captures of mlxSamples.

#include <math.h>

#include "mlx90393_gimbal.h"
#include "spi2_bus.h"
#include "crossfire_host.h"

#include "board.h"
#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "delays_driver.h"
#include "timers_driver.h"

#define MLX_CMD_SB_ZYX    0x1E
#define MLX_CMD_RM_ZYX    0x4E
#define MLX_CMD_EX        0x80
#define MLX_CMD_RT        0xF0
#define MLX_CMD_WR        0x60
#define MLX_CMD_RR        0x50

#define MLX_STATUS_BURST  0x80   // status byte bit 7: burst mode active
#define MLX_STATUS_ERROR  0x10   // status byte bit 4
#define MLX_TIMEOUT_MS    20     // no new sample this long: count an error, restart burst

// Data-ready poll: both sensors run in burst mode (BURST_DATA_RATE 0, ~1.2 ms
// per X/Y/Z conversion); TIM7 checks their INT pins every 125 us and reads
// whichever has a result. Polled rather than EXTI because PB9 (gimbal 1)
// shares EXTI line 9 with the telemetry soft-serial RX on PD9.
#define MLX_POLL_TIMER    TIM7
#define MLX_POLL_IRQn     TIM7_IRQn
#define MLX_POLL_US       125
#define MLX_POLL_IRQ_PRIO 6      // below the ELRS image (4) and RTOS syscalls (5)
#define MLX_FAIL_LIMIT    10     // consecutive errors before sticks are invalid

#define ADC_CENTER        2048
#define ADC_MAX           4095

MlxSample mlxSamples[2];
volatile uint32_t mlxReadCount[2];  // good reads since boot (rate checks over SWD)

static const gpio_t drdyPin[2] = { GIMBAL0_DRDY_GPIO, GIMBAL1_DRDY_GPIO };
static const uint8_t spiDev[2] = { SPI2_DEV_GIMBAL0, SPI2_DEV_GIMBAL1 };

static bool sensorUp[2];
// Set while the CLI or the mixer talks to the sensors outside the poll. The
// TIM7 ISR then skips; a flag, not NVIC masking, so neither path can unmask
// the poll in the middle of the other's command sequence.
static volatile bool pollPaused;

// The Crossfire image owns SPI2 and reads the sensors itself
static volatile bool released;

// Which of the Crossfire image's two sensors is our sensor 0 (CS PE8).
// [?] not traced in the image; check on the radio.
#define XF_SENSOR_FOR_OURS_0  0

static const struct { uint8_t reg; uint16_t val; } tbsRegs[] = {
  { 0, 0x0000 },  // GAIN_SEL 0, HALLCONF 0
  { 1, 0x8400 },  // TRIG_INT_SEL: INT pin = data ready; TCMP_EN
  { 2, 0x02A4 },  // OSR 0, DIG_FILT 1, RES_X/Y/Z 1
  { 4, 0x8000 },  // offset X (needed with TCMP)
  { 5, 0x8000 },  // offset Y
  { 6, 0x8000 },  // offset Z
};

static uint8_t mlxCmd(int s, uint8_t cmd)
{
  uint8_t tx[2] = { cmd, 0 }, rx[2];
  spi2_xfer(spiDev[s], tx, rx, 2);
  return rx[1];
}

static uint8_t mlxWriteReg(int s, uint8_t reg, uint16_t val)
{
  uint8_t tx[5] = { MLX_CMD_WR, (uint8_t)(val >> 8), (uint8_t)val, (uint8_t)(reg << 2), 0 };
  uint8_t rx[5];
  spi2_xfer(spiDev[s], tx, rx, 5);
  return rx[4];
}

static uint16_t mlxReadReg(int s, uint8_t reg)
{
  uint8_t tx[5] = { MLX_CMD_RR, (uint8_t)(reg << 2), 0, 0, 0 };
  uint8_t rx[5];
  spi2_xfer(spiDev[s], tx, rx, 5);
  return (uint16_t)(rx[3] << 8 | rx[4]);
}

static bool mlxReadMeasurement(int s)
{
  uint8_t tx[8] = { MLX_CMD_RM_ZYX };
  uint8_t rx[8];
  spi2_xfer(spiDev[s], tx, rx, 8);
  MlxSample& m = mlxSamples[s];
  m.status = rx[1];
  if (m.status & MLX_STATUS_ERROR) return false;
  m.x = (int16_t)((rx[2] << 8 | rx[3]) - 0x8000);
  m.y = (int16_t)((rx[4] << 8 | rx[5]) - 0x8000);
  m.z = (int16_t)((rx[6] << 8 | rx[7]) - 0x8000);
  m.stamp = timersGetMsTick();
  mlxReadCount[s]++;
  return true;
}

// (Re)start continuous X/Y/Z conversions. Returns false if the sensor didn't
// report burst mode.
static bool mlxStartBurst(int s)
{
  mlxCmd(s, MLX_CMD_EX);
  delay_ms(3);  // a conversion in flight finishes before EX takes effect
  return (mlxCmd(s, MLX_CMD_SB_ZYX) & MLX_STATUS_BURST) != 0;
}

extern "C" void TIM7_IRQHandler()
{
  MLX_POLL_TIMER->SR = ~TIM_SR_UIF;
  if (pollPaused) return;
  for (int s = 0; s < 2; s++) {
    if (sensorUp[s] && gpio_read(drdyPin[s])) mlxReadMeasurement(s);
  }
}

static void mlxPollTimerInit()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
  (void)RCC->APB1ENR;
  MLX_POLL_TIMER->CR1 = 0;
  MLX_POLL_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 1000000 - 1;  // 1 us
  MLX_POLL_TIMER->ARR = MLX_POLL_US - 1;
  MLX_POLL_TIMER->EGR = TIM_EGR_UG;
  MLX_POLL_TIMER->SR = 0;
  MLX_POLL_TIMER->DIER = TIM_DIER_UIE;
  NVIC_SetPriority(MLX_POLL_IRQn, MLX_POLL_IRQ_PRIO);
  NVIC_EnableIRQ(MLX_POLL_IRQn);
  MLX_POLL_TIMER->CR1 = TIM_CR1_CEN;
}

// Rewrite one register on both sensors (bench tuning, CLI `gimbal reg`):
// stop the poll, leave burst mode, write and read back, restart the burst.
bool mlxSetReg(uint8_t reg, uint16_t val)
{
  bool ok = true;
  pollPaused = true;
  // keep the mixer from restarting a sensor while we work on it
  mlxSamples[0].stamp = mlxSamples[1].stamp = timersGetMsTick();
  for (int s = 0; s < 2; s++) {
    if (!sensorUp[s]) continue;
    mlxCmd(s, MLX_CMD_EX);
    delay_ms(3);
    mlxWriteReg(s, reg, val);
    if (mlxReadReg(s, reg) != val) ok = false;
    if (!mlxStartBurst(s)) ok = false;
    mlxSamples[s].stamp = timersGetMsTick();
  }
  pollPaused = false;
  return ok;
}

void mlxGimbalRelease()
{
  pollPaused = true;
  released = true;
  NVIC_DisableIRQ(MLX_POLL_IRQn);
  MLX_POLL_TIMER->CR1 = 0;
  MLX_POLL_TIMER->SR = 0;
  NVIC_ClearPendingIRQ(MLX_POLL_IRQn);
}

bool mlxGimbalReclaim()
{
  spi2Reinit();
  bool ok = mlxGimbalInit();
  pollPaused = false;
  released = false;
  return ok;
}

bool mlxGimbalInit()
{
  spi2Init();
  gpio_init(GIMBAL0_DRDY_GPIO, GPIO_IN, GPIO_PIN_SPEED_LOW);
  gpio_init(GIMBAL1_DRDY_GPIO, GPIO_IN, GPIO_PIN_SPEED_LOW);

  for (int s = 0; s < 2; s++) {
    mlxCmd(s, MLX_CMD_EX);
    delay_ms(2);
    mlxCmd(s, MLX_CMD_RT);
  }
  delay_ms(84);  // TBS waits this long after RT

  bool ok = true;
  for (int s = 0; s < 2; s++) {
    sensorUp[s] = true;
    for (const auto& r : tbsRegs) {
      mlxWriteReg(s, r.reg, r.val);
      if (mlxReadReg(s, r.reg) != r.val) sensorUp[s] = false;
    }
    mlxSamples[s] = {};
    if (sensorUp[s] && !mlxStartBurst(s)) sensorUp[s] = false;
    ok = ok && sensorUp[s];
  }
  mlxPollTimerInit();
  return ok;
}

// ===========================================================================
// AXIS MAPPING (from gimbal/capture1.csv, 2026-11-10)
//
// Each gimbal's magnet tilts in two planes over its MLX90393:
//   vertical   = atan2(y, sqrt(x*x + z*z))    -40 deg (down) .. +40 deg (up)
//   horizontal = atan2(x, sqrt(y*y + z*z))    -38 deg (left) .. +41 deg (right)
// i.e. asin of each component over |B|. With this pair the stick corners
// read within ~2 deg of the pure-axis ends. atan2(y, z) for vertical read
// ~5 deg past them in the corners, so calibration (which records the corners)
// left straight up/down at only 90%. |B| is 9.5k..13.3k counts, so the angle
// doesn't care about magnet strength or temperature drift.
//
// Output: 12-bit values in hal order, out[0] = LH, out[1] = LV, out[2] = RV,
// out[3] = RH, larger = up/right. EdgeTX's stick calibration does the rest.
// ===========================================================================
#define MLX_COUNTS_PER_DEG  40.0f  // +-45 deg -> +-1800 around 2048

static uint16_t mlxAngleToAdc(float deg)
{
  int32_t v = ADC_CENTER + (int32_t)(deg * MLX_COUNTS_PER_DEG);
  if (v < 0) v = 0;
  if (v > ADC_MAX) v = ADC_MAX;
  return (uint16_t)v;
}

static void mlxMapToSticks(const MlxSample sample[2], uint16_t out[4])
{
  const float RAD2DEG = 57.29578f;
  float h[2], v[2];
  for (int s = 0; s < 2; s++) {
    float x = sample[s].x, y = sample[s].y, z = sample[s].z;
    v[s] = atan2f(y, sqrtf(x * x + z * z)) * RAD2DEG;
    h[s] = atan2f(x, sqrtf(y * y + z * z)) * RAD2DEG;
  }
  out[0] = mlxAngleToAdc(h[MLX_GIMBAL_LEFT]);   // LH
  out[1] = mlxAngleToAdc(v[MLX_GIMBAL_LEFT]);   // LV
  out[2] = mlxAngleToAdc(v[MLX_GIMBAL_RIGHT]);  // RV
  out[3] = mlxAngleToAdc(h[MLX_GIMBAL_RIGHT]);  // RH
}

bool mlxGimbalOk()
{
  return sensorUp[0] && sensorUp[1] &&
         mlxSamples[0].errors < MLX_FAIL_LIMIT &&
         mlxSamples[1].errors < MLX_FAIL_LIMIT;
}

void mlxGimbalUpdate(uint16_t* analogs)
{
  if (released) {
    // Same mapping on the image's reads. Until it has some, the analogs keep
    // the last values we read ourselves (no throttle jump on a switch).
    int16_t xyz[2][3];
    if (!xfHostRawGimbals(xyz)) return;
    MlxSample sample[2] = {};
    for (int s = 0; s < 2; s++) {
      const int16_t* v = xyz[s ^ XF_SENSOR_FOR_OURS_0];
      sample[s].x = v[0];
      sample[s].y = v[1];
      sample[s].z = v[2];
    }
    uint16_t sticks[4];
    mlxMapToSticks(sample, sticks);
    for (int i = 0; i < 4; i++) analogs[i] = sticks[i];
    return;
  }

  uint32_t now = timersGetMsTick();

  // The TIM7 poll fills mlxSamples; take a consistent copy of both.
  MlxSample sample[2];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  sample[0] = mlxSamples[0];
  sample[1] = mlxSamples[1];
  __set_PRIMASK(primask);

  for (int s = 0; s < 2; s++) {
    if (!sensorUp[s]) continue;
    if (now - sample[s].stamp <= MLX_TIMEOUT_MS) {
      mlxSamples[s].errors = 0;
    } else {
      // Stalled (or never started): count it and restart the burst. One
      // restart per MLX_TIMEOUT_MS, as the restart refreshes the stamp.
      if (mlxSamples[s].errors < 0xFFFF) mlxSamples[s].errors++;
      if (pollPaused) continue;  // the CLI is reconfiguring
      pollPaused = true;
      mlxStartBurst(s);
      mlxSamples[s].stamp = now;
      pollPaused = false;
    }
  }

  uint16_t sticks[4];
  if (mlxGimbalOk()) {
    mlxMapToSticks(sample, sticks);
  } else {
    // Never feed stale values: centre everything.
    // TODO: throttle to its low end and raise an alarm once the mapping and
    // the throttle stick are known.
    for (auto& v : sticks) v = ADC_CENTER;
  }
  for (int i = 0; i < 4; i++) analogs[i] = sticks[i];
}
