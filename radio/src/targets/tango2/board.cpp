/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
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

// TBS Tango 2 board init (modelled on targets/taranis/board.cpp)

#include "stm32_hal_ll.h"
#include "stm32_gpio.h"

#include "hal/adc_driver.h"
#include "hal/switch_driver.h"
#include "hal/module_port.h"
#include "hal/abnormal_reboot.h"
#include "hal/rotary_encoder.h"
#include "hal/usb_driver.h"
#include "hal/gpio.h"

#include "board.h"
#include "boards/generic_stm32/module_ports.h"
#include "boards/generic_stm32/analog_inputs.h"
#include "boards/generic_stm32/rgb_leds.h"

#include "debug.h"
#include "rtc.h"
#include "timers_driver.h"
#include "delays_driver.h"
#include "dataconstants.h"

#include "spi2_bus.h"
#include "radio_host.h"
#if defined(TANGO2_MLX_GIMBALS)
  #include "mlx90393_gimbal.h"
#endif

#if !defined(BOOT)
  #include "edgetx.h"
#endif

HardwareOptions hardwareOptions;

// ---------------------------------------------------------------------------
// Resident loader contract (two-to-tango: loader/src/main.cpp):
//  - the loader starts the IWDG (~4 s) before jumping here: kick it until
//    EdgeTX's own WDG_ENABLE() takes over
//  - RTC BKP18R is its crash-loop counter: clear it once we run properly
//  - RTC BKP17R holds boot breadcrumbs, shown by the loader's 'I' command
//  - RTC BKP19R = "MON1" + reset: stay in the loader (boardRebootToLoader)
// ---------------------------------------------------------------------------
#define TANGO2_CRUMB_TAG      0xED6E0000u  // "EdgE" + stage
#define TANGO2_HEALTHY_MS     3000

static void tango2Crumb(uint32_t stage)
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;
  PWR->CR |= PWR_CR_DBP;
  RTC->BKP17R = TANGO2_CRUMB_TAG | stage;
}

#if !defined(BOOT)
static void tango2MarkHealthy()
{
  static bool healthy = false;
  if (healthy || timersGetMsTick() < TANGO2_HEALTHY_MS) return;
  healthy = true;
  tango2Crumb(4);
  RTC->BKP18R = 0;  // tell the loader EdgeTX started fine
}
#endif

// ---------------------------------------------------------------------------
// Clocks. The EdgeTX firmware normally inherits them from the EdgeTX
// bootloader. Here the resident loader jumps to us before its own clock init
// (on HSI, 16 MHz), so bring up HSE 8 MHz -> 168 MHz (48 MHz USB) ourselves,
// as early as possible (init hook: after .data, before .bss and main()).
// ---------------------------------------------------------------------------
extern "C" void SystemClock_Config(void);

static void tango2EarlyInit()
{
  // FPU on (normally done by SystemInit in the EdgeTX bootloader)
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
  __DSB();
  __ISB();

  // If something left SYSCLK on the PLL, step back to HSI before
  // reprogramming it.
  if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
  }
  RCC->CR &= ~RCC_CR_PLLON;
  while (RCC->CR & RCC_CR_PLLRDY) {}

  SystemClock_Config();  // f4/system_clock.c, PLL_M follows HSE_VALUE

  IWDG->KR = 0xAAAA;     // loader's watchdog
  tango2Crumb(1);
}

void* _tango2_early_init_hook[] __INIT_HOOK = {
  (void*)tango2EarlyInit,
};

#if !defined(BOOT)

// ---------------------------------------------------------------------------
// Sticks: the stock ADC driver reads VBAT; the MLX90393 gimbals fill in
// LH/LV/RV/RH after each conversion. The same hook (mixer task) tells the
// loader that EdgeTX is healthy.
// ---------------------------------------------------------------------------
static bool tango2AdcStart()
{
  return _adc_driver.start_conversion ? _adc_driver.start_conversion() : true;
}

static void tango2AdcWait()
{
  if (_adc_driver.wait_completion) _adc_driver.wait_completion();
#if defined(TANGO2_MLX_GIMBALS)
  mlxGimbalUpdate(getAnalogValues());
#endif
  tango2MarkHealthy();  // runs from the mixer task once EdgeTX is up
}

static const etx_hal_adc_driver_t _tango2_adc_driver = {
  .inputs = _adc_driver.inputs,
  .default_pots_cfg = _adc_driver.default_pots_cfg,
  .init = _adc_driver.init,
  .deinit = _adc_driver.deinit,
  .start_conversion = tango2AdcStart,
  .wait_completion = tango2AdcWait,
  .set_input_mask = _adc_driver.set_input_mask,
  .get_input_mask = _adc_driver.get_input_mask,
};

// ---------------------------------------------------------------------------
// Parts only the radio images drive: park them in a safe state. Also used
// after the Crossfire image is stopped.
// ---------------------------------------------------------------------------
void tango2ParkRadioHardware()
{
  // SX1272 (ELRS image will own these): deselected, not in reset,
  // RF switch to RX, PA2 high (TBS: external PA off)
  gpio_init(SX1272_NSS_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_set(SX1272_NSS_GPIO);
  gpio_init(SX1272_RESET_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_clear(SX1272_RESET_GPIO);
  gpio_init(SX1272_RFSWITCH_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_set(SX1272_RFSWITCH_GPIO);
  gpio_init(SX1272_PA2_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_set(SX1272_PA2_GPIO);

  // Internal WiFi module off
  gpio_init(WIFI_EN_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_clear(WIFI_EN_GPIO);

  // The Crossfire image's other pins: WiFi UART4 (PA0/PA1) and the ATSHA204A
  // on I2C1 (PB6/PB7) as inputs, status LED data (PB11) low
  gpio_init(GPIO_PIN(GPIOA, 0), GPIO_IN, GPIO_PIN_SPEED_LOW);
  gpio_init(GPIO_PIN(GPIOA, 1), GPIO_IN, GPIO_PIN_SPEED_LOW);
  gpio_init(GPIO_PIN(GPIOB, 6), GPIO_IN, GPIO_PIN_SPEED_LOW);
  gpio_init(GPIO_PIN(GPIOB, 7), GPIO_IN, GPIO_PIN_SPEED_LOW);
  gpio_init(GPIO_PIN(GPIOB, 11), GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_clear(GPIO_PIN(GPIOB, 11));
}

// Status LED: one WS2812 on PB11
void ledInit() {}
void ledOff() { rgbSetLedColor(0, 0, 0, 0); rgbLedColorApply(); }
void ledRed() { rgbSetLedColor(0, 40, 0, 0); rgbLedColorApply(); }
void ledGreen() { rgbSetLedColor(0, 0, 40, 0); rgbLedColorApply(); }
void ledBlue() { rgbSetLedColor(0, 0, 0, 40); rgbLedColorApply(); }
void ledBoot() { ledBlue(); }

void boardInit()
{
  tango2Crumb(2);
  IWDG->KR = 0xAAAA;
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  // Audio amp muted before anything can pop
  gpio_init(AUDIO_MUTE_GPIO, GPIO_OUT, GPIO_PIN_SPEED_LOW);
  gpio_set(AUDIO_MUTE_GPIO);

  delaysInit();
  timersInit();
  __enable_irq();

  usbInit();

  pwrInit();
  boardInitModulePorts();

  tango2ParkRadioHardware();
  spi2Init();

  keysInit();
  switchInit();
  rotaryEncoderInit();
  rotaryEncoderStartPolling();  // see ROTARY_ENCODER_POLLING in hal.h

#if defined(TANGO2_MLX_GIMBALS)
  if (!mlxGimbalInit())
    TRACE("MLX90393 gimbal init failed");
#endif

  // Internal RF images (ELRS / Crossfire): RAM vector table and the rfhost
  // task, which starts the image the model asks for once the scheduler runs
  radioHostInit();

  if (!adcInit(&_tango2_adc_driver))
    TRACE("adcInit failed");

  lcdInit();  // delaysInit() must be called before
  audioInit();

  rgbLedInit();
  ledBoot();

  hapticInit();

#if defined(DEBUG)
  // Freeze timers & watchdog when core is halted
  DBGMCU->APB1FZ = 0x00E009FF;
  DBGMCU->APB2FZ = 0x00070003;
#endif

  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
    pwrOn();
  }

#if defined(RTCLOCK)
  rtcInit();  // RTC must be initialized before rambackupRestore() is called
#endif

  backlightInit();

#if defined(GUI)
  lcdSetContrast(true);
#endif

  IWDG->KR = 0xAAAA;
  tango2Crumb(3);
}
#endif  // !BOOT

// The rfhost task follows the model's internal module setting
void tango2IntmoduleOn() {}

void boardOff()
{
#if !defined(BOOT)
  radioHostPowerOff();
#endif
#if !defined(BOOT)
  rgbLedStop();
  rgbLedClearAll();
#endif

  BACKLIGHT_DISABLE();

  while (pwrPressed()) {
    WDG_RESET();
  }

  lcdOff();
  SysTick->CTRL = 0;  // turn off systick
  pwrOff();           // resets instead when USB powers the radio

  __disable_irq();

  while (1) {
    WDG_RESET();
    // CPU still runs while the power key is held
    if (!pwrPressed()) {
      PWR->CR |= PWR_CR_CWUF;
      PWR->CR |= PWR_CR_PDDS;
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      __WFE();
    }
  }
  // this function must not return!
}
