/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * TBS Tango 2 hardware definitions.
 *
 * Pins were worked out from the unofficial "Wingman" build, TBS's Crossfire
 * image and the hardware itself.
 * Confidence there: [sure] read from code/tables, [likely] inferred, [?] role
 * guessed. Keys, switches, ADC, ports and timers that live in the JSON
 * description are in radio/src/boards/hw_defs/tango2.json.
 */

#pragma once

#include "hal_settings.h"

#define TELEMETRY_EXTI_PRIO             0 // required for soft serial

// --- Power -------------------------------------------------------------------
#define PWR_ON_GPIO                     GPIO_PIN(GPIOB, 12) // [sure] hold power
#define PWR_SWITCH_GPIO                 GPIO_PIN(GPIOB, 14) // [sure] button, reads HIGH when pressed
#define PWR_SWITCH_ACTIVE_HIGH                              // see pwr_driver.cpp

// --- External module bay (Wingman's 5-port table) ------------------------------
#define EXTMODULE_PWR_GPIO              GPIO_PIN(GPIOE, 5)  // [sure]
#define EXTERNAL_MODULE_PWR_ON()        gpio_set(EXTMODULE_PWR_GPIO)
#define EXTERNAL_MODULE_PWR_OFF()       gpio_clear(EXTMODULE_PWR_GPIO)
#define IS_EXTERNAL_MODULE_ON()         gpio_read(EXTMODULE_PWR_GPIO)
#define EXTMODULE_TX_GPIO               GPIO_PIN(GPIOC, 6)  // [sure]
#define EXTMODULE_RX_GPIO               GPIO_PIN(GPIOC, 7)  // [sure]
// port TIMER: TIM8_CH1 on PC6
#define EXTMODULE_TIMER                 TIM8
#define EXTMODULE_TIMER_Channel         LL_TIM_CHANNEL_CH1
#define EXTMODULE_TIMER_FREQ            (PERI2_FREQUENCY * TIMER_MULT_APB2)
#define EXTMODULE_TIMER_IRQn            TIM8_UP_TIM13_IRQn
#define EXTMODULE_TIMER_IRQHandler      TIM8_UP_TIM13_IRQHandler
#define EXTMODULE_TIMER_TX_GPIO_AF      LL_GPIO_AF_3
#define EXTMODULE_TIMER_DMA             DMA2
#define EXTMODULE_TIMER_DMA_CHANNEL     LL_DMA_CHANNEL_7
#define EXTMODULE_TIMER_DMA_STREAM      LL_DMA_STREAM_1
#define EXTMODULE_TIMER_DMA_STREAM_IRQn DMA2_Stream1_IRQn
#define EXTMODULE_TIMER_DMA_IRQHandler  DMA2_Stream1_IRQHandler
// port UART: USART6 PC6/PC7, DMA2 S6 ch5 (TX) / S1 ch5 (RX)   [sure]
#define EXTMODULE_USART                 USART6
#define EXTMODULE_USART_IRQn            USART6_IRQn
#define EXTMODULE_USART_IRQHandler      USART6_IRQHandler
#define EXTMODULE_USART_TX_DMA          DMA2
#define EXTMODULE_USART_TX_DMA_CHANNEL  LL_DMA_CHANNEL_5
#define EXTMODULE_USART_TX_DMA_STREAM   LL_DMA_STREAM_6
#define EXTMODULE_USART_RX_DMA_CHANNEL  LL_DMA_CHANNEL_5
#define EXTMODULE_USART_RX_DMA_STREAM   LL_DMA_STREAM_1
// Wingman's set_inverted callback writes PE4 = !enable. The generic F4
// callback writes TX and RX inverter pins, so both are PE4.     [likely]
#define EXTMODULE_TX_INVERT_GPIO        GPIO_PIN(GPIOE, 4)
#define EXTMODULE_RX_INVERT_GPIO        GPIO_PIN(GPIOE, 4)

// --- S.Port on the module bay: USART3 PD8 (TX) / PD9 (RX), DMA1 S3 ch4 [sure]
#define TELEMETRY_SET_INPUT             0
#define TELEMETRY_TX_GPIO               GPIO_PIN(GPIOD, 8)
#define TELEMETRY_RX_GPIO               GPIO_PIN(GPIOD, 9)
#define TELEMETRY_USART                 USART3
#define TELEMETRY_DMA                   DMA1
#define TELEMETRY_DMA_Stream_TX         LL_DMA_STREAM_3
#define TELEMETRY_DMA_Channel_TX        LL_DMA_CHANNEL_4
#define TELEMETRY_DMA_TX_Stream_IRQ     DMA1_Stream3_IRQn
#define TELEMETRY_DMA_TX_IRQHandler     DMA1_Stream3_IRQHandler
#define TELEMETRY_DMA_TX_FLAG_TC        DMA_IT_TCIF3
#define TELEMETRY_USART_IRQHandler      USART3_IRQHandler
#define TELEMETRY_USART_IRQn            USART3_IRQn
// soft-serial RX on PD9, TIM11 (Wingman port 4)                 [sure]
#define TELEMETRY_EXTI_PORT             LL_SYSCFG_EXTI_PORTD
#define TELEMETRY_EXTI_SYS_LINE         LL_SYSCFG_EXTI_LINE9
#define TELEMETRY_EXTI_LINE             LL_EXTI_LINE_9
#define TELEMETRY_EXTI_TRIGGER          LL_EXTI_TRIGGER_RISING
#if !defined(USE_EXTI9_5_IRQ)
  #define USE_EXTI9_5_IRQ
#endif
#undef EXTI9_5_IRQ_Priority
#define EXTI9_5_IRQ_Priority            TELEMETRY_EXTI_PRIO
#define TELEMETRY_TIMER                 TIM11
#define TELEMETRY_TIMER_IRQn            TIM1_TRG_COM_TIM11_IRQn
#define TELEMETRY_TIMER_IRQHandler      TIM1_TRG_COM_TIM11_IRQHandler

// Software IRQ (Prio 5 -> FreeRTOS compatible)
#define TELEMETRY_RX_FRAME_EXTI_LINE    LL_EXTI_LINE_4
#define USE_EXTI4_IRQ
#define EXTI4_IRQ_Priority              5

// --- ADC: VBAT on PB1 / ADC1 ch9 (JSON). 3.3 V reference.
#define ADC_VREF_PREC2                  330

// --- LCD: SPI3 + DMA1 Stream7 ch0 -------------------------------------------
// The controller is SSD1327-class (4-bit grey, 128 columns, 96 rows used),
// not SSD1309: Wingman's init is FD 12, AE, A8 7F, A1 00, A2 00, A0 51,
// AB 01, 81 80, B1 51, B3 01, B9, BC 08, BE 07, B6 01, D5 62, A4, and it
// refreshes 0x1800 bytes (128x96 @ 4 bpp) rotated by 180 degrees.
#define LCD_SPI                         SPI3                 // [sure]
#define LCD_GPIO_AF                     GPIO_AF6
#define LCD_CLK_GPIO                    GPIO_PIN(GPIOB, 3)   // [sure]
#define LCD_MOSI_GPIO                   GPIO_PIN(GPIOB, 5)   // [sure]
#define LCD_NCS_GPIO                    GPIO_PIN(GPIOD, 1)   // [sure]
#define LCD_RST_GPIO                    GPIO_PIN(GPIOD, 3)   // [sure]
#define LCD_A0_GPIO                     GPIO_PIN(GPIOD, 6)   // [sure]
#define LCD_DMA                         DMA1
#define LCD_DMA_Stream                  DMA1_Stream7
#define LCD_DMA_Stream_IRQn             DMA1_Stream7_IRQn
#define LCD_DMA_Stream_IRQHandler       DMA1_Stream7_IRQHandler
#define LCD_DMA_FLAGS                   (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)
#define LCD_DMA_FLAG_INT                DMA_HIFCR_CTCIF7
#define LCD_PANEL_ROWS                  96   // rows the controller window covers
#define LCD_ROW_OFFSET                  16   // the 64-row UI is centred in them

// --- Audio: DAC1 on PA4, TIM6, DMA1 Stream5 -----------------------------------
#define AUDIO_OUTPUT_GPIO               GPIO_PIN(GPIOA, 4)   // [sure]
#define AUDIO_DMA                       DMA1
#define AUDIO_DMA_Stream                DMA1_Stream5
#define AUDIO_DMA_Stream_IRQn           DMA1_Stream5_IRQn
#define AUDIO_DMA_Stream_IRQHandler     DMA1_Stream5_IRQHandler
#define AUDIO_TIMER                     TIM6
#define AUDIO_MUTE_GPIO                 GPIO_PIN(GPIOD, 5)   // [likely] polarity not checked
#define AUDIO_MUTE_DELAY                500  // ms
#define AUDIO_UNMUTE_DELAY              150  // ms

// --- SD card on SDIO (PC8-PC12, PD2) ----------------------------------------
// No card-detect pin known. DMA2 Stream3 ch4 (Stream6 is USART6 TX).
#define SD_SDIO_DMA                     DMA2
#define SD_SDIO_DMA_STREAM              DMA2_Stream3
#define SD_SDIO_DMA_CHANNEL             LL_DMA_CHANNEL_4
#define SD_SDIO_DMA_IRQn                DMA2_Stream3_IRQn
#define SD_SDIO_DMA_IRQHANDLER          DMA2_Stream3_IRQHandler
#define SD_SDIO_CLK_DIV(fq)             ((48000000 / (fq)) - 2)
#define SD_SDIO_TRANSFER_CLK_DIV        SD_SDIO_CLK_DIV(24000000)
#define STORAGE_USE_SDIO

// --- Rotary encoder (PA8/PA10, from the JSON) ------------------------------------
// EXTI line 10 belongs to the ELRS image (DIO0 on PE10), so PA10 gets no
// interrupts once the image is up: sample both phases from the encoder timer
// instead (rotaryEncoderStartPolling() in boardInit).
#define ROTARY_ENCODER_POLLING

// --- SPI2: SX1272 (ELRS image, later) + two MLX90393 gimbals -----------------
#define SPI2_SCK_GPIO                   GPIO_PIN(GPIOB, 10)  // [sure] AF5
#define SPI2_MISO_GPIO                  GPIO_PIN(GPIOC, 2)   // [sure] AF5
#define SPI2_MOSI_GPIO                  GPIO_PIN(GPIOC, 3)   // [sure] AF5
#define SPI2_GPIO_AF                    GPIO_AF5
#define SX1272_NSS_GPIO                 GPIO_PIN(GPIOC, 1)   // [sure]
#define SX1272_RESET_GPIO               GPIO_PIN(GPIOC, 13)  // [sure] active high
#define SX1272_RFSWITCH_GPIO            GPIO_PIN(GPIOC, 0)   // [sure] low = TX
#define SX1272_PA2_GPIO                 GPIO_PIN(GPIOA, 2)   // [likely] high = PA off
#define GIMBAL0_CS_GPIO                 GPIO_PIN(GPIOE, 8)   // [sure]
#define GIMBAL1_CS_GPIO                 GPIO_PIN(GPIOD, 7)   // [sure]
#define GIMBAL0_DRDY_GPIO               GPIO_PIN(GPIOC, 5)   // [sure] pin, pairing with CS [likely]
#define GIMBAL1_DRDY_GPIO               GPIO_PIN(GPIOB, 9)   // [sure] pin, pairing with CS [likely]

// --- Internal "Tango WiFi" ESP: UART4 PA0/PA1 @500k, PA3 enable. Kept off. ---
#define WIFI_EN_GPIO                    GPIO_PIN(GPIOA, 3)

// --- Unknown pins ------------------------------------------------------------
// PD10: input, pull-up, active low, only checked together with usbPlugged() [?]

// Resident loader handshake (two-to-tango: loader/src/main.cpp): RTC->BKP19R = "MON1" + reset
#define LOADER_STAY_MAGIC               0x4D4F4E31u

