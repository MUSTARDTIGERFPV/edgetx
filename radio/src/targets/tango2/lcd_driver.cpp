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

// TBS Tango 2 OLED: SSD1327-class controller (4-bit grey), 128 x 96 window,
// on SPI3 (mode 3, PCLK1/4) with DMA1 Stream7. The EdgeTX UI is the
// 128x64 monochrome one; each refresh expands it to 4 bpp and writes it into
// rows LCD_ROW_OFFSET..+63 of the panel.
//
// Init sequence and SPI setup are Wingman's (see hal.h). Unverified on the
// hardware: which way up the picture comes out (TANGO2_LCD_ROTATE180) and the
// nibble order (TANGO2_LCD_LEFT_PIXEL_LOW). Both are one-line switches.

#include "hal/gpio.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_dma.h"

#include "board.h"
#include "debug.h"
#include "lcd.h"

#include "hal/abnormal_reboot.h"
#include "timers_driver.h"
#include "delays_driver.h"

#if !defined(BOOT)
  #include "edgetx.h"
#endif

#if !defined(TANGO2_LCD_ROTATE180)
  #define TANGO2_LCD_ROTATE180 1   // Wingman sends its frame buffer reversed
#endif
#if !defined(TANGO2_LCD_LEFT_PIXEL_LOW)
  #define TANGO2_LCD_LEFT_PIXEL_LOW 0
#endif

#define OLED_COLS_BYTES   (LCD_W / 2)            // 64 bytes per row at 4 bpp
#define OLED_FRAME_BYTES  (OLED_COLS_BYTES * LCD_H)
#define RESET_WAIT_DELAY_MS 100

#define LCD_NCS_HIGH()  gpio_set(LCD_NCS_GPIO)
#define LCD_NCS_LOW()   gpio_clear(LCD_NCS_GPIO)
#define LCD_A0_HIGH()   gpio_set(LCD_A0_GPIO)
#define LCD_A0_LOW()    gpio_clear(LCD_A0_GPIO)
#define LCD_RST_HIGH()  gpio_set(LCD_RST_GPIO)
#define LCD_RST_LOW()   gpio_clear(LCD_RST_GPIO)

static uint8_t oledFrame[OLED_FRAME_BYTES] __DMA;
static volatile bool lcd_busy;
bool lcdInitFinished = false;
static uint8_t lcdGrey = 0x0F;   // "on" pixel level

static void spiWaitIdle()
{
  while ((LCD_SPI->SR & SPI_SR_TXE) == 0) {}
  while (LCD_SPI->SR & SPI_SR_BSY) {}
  (void)LCD_SPI->DR;
  (void)LCD_SPI->SR;
}

static void spiSend(uint8_t b)
{
  while ((LCD_SPI->SR & SPI_SR_TXE) == 0) {}
  LCD_SPI->DR = b;
}

// SSD1327 takes command arguments with D/C low too
static void lcdWriteCommand(uint8_t byte)
{
  LCD_A0_LOW();
  LCD_NCS_LOW();
  spiSend(byte);
  spiWaitIdle();
  LCD_NCS_HIGH();
}

static void lcdWriteCommand2(uint8_t cmd, uint8_t arg)
{
  lcdWriteCommand(cmd);
  lcdWriteCommand(arg);
}

static void lcdSetWindow(uint8_t row0, uint8_t row1, uint8_t col0, uint8_t col1)
{
  lcdWriteCommand(0x75);  // row address
  lcdWriteCommand(row0);
  lcdWriteCommand(row1);
  lcdWriteCommand(0x15);  // column address (2 pixels per column)
  lcdWriteCommand(col0);
  lcdWriteCommand(col1);
}

static void lcdHardwareInit()
{
  stm32_spi_enable_clock(LCD_SPI);
  gpio_init_af(LCD_MOSI_GPIO, LCD_GPIO_AF, GPIO_PIN_SPEED_HIGH);
  gpio_init_af(LCD_CLK_GPIO, LCD_GPIO_AF, GPIO_PIN_SPEED_HIGH);

  // Wingman: CR1 = SSM|SSI|BR=/4|CPOL|CPHA, then MSTR, then SPE
  LCD_SPI->CR1 = 0;
  LCD_SPI->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_0;
  LCD_SPI->CR2 = 0;
  LCD_SPI->CR1 |= SPI_CR1_MSTR;
  LCD_SPI->CR1 |= SPI_CR1_SPE;

  gpio_init(LCD_NCS_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);
  LCD_NCS_HIGH();
  gpio_init(LCD_RST_GPIO, GPIO_OUT, GPIO_PIN_SPEED_MEDIUM);
  gpio_init(LCD_A0_GPIO, GPIO_OUT, GPIO_PIN_SPEED_HIGH);

  stm32_dma_enable_clock(LCD_DMA);
  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN;
  LCD_DMA->HIFCR = LCD_DMA_FLAGS;
  // channel 0 (SPI3_TX), memory to peripheral, memory increment
  LCD_DMA_Stream->CR = DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
  LCD_DMA_Stream->PAR = (uint32_t)&LCD_SPI->DR;
  LCD_DMA_Stream->M0AR = (uint32_t)oledFrame;
  LCD_DMA_Stream->FCR = 0x05;  // DMDIS | FTH_0

  NVIC_SetPriority(LCD_DMA_Stream_IRQn, 7);
  NVIC_EnableIRQ(LCD_DMA_Stream_IRQn);
}

static void lcdStart()
{
  lcdWriteCommand2(0xFD, 0x12);  // unlock commands
  lcdWriteCommand(0xAE);         // display off
  lcdWriteCommand2(0xA8, 0x7F);  // multiplex ratio
  lcdWriteCommand2(0xA1, 0x00);  // start line
  lcdWriteCommand2(0xA2, 0x00);  // display offset
  lcdWriteCommand2(0xA0, 0x51);  // remap
  lcdWriteCommand2(0xAB, 0x01);  // function select A: internal VDD
  lcdWriteCommand2(0x81, 0x80);  // contrast
  lcdWriteCommand2(0xB1, 0x51);  // phase length
  lcdWriteCommand2(0xB3, 0x01);  // clock divider / oscillator
  lcdWriteCommand(0xB9);         // default linear grey table
  lcdWriteCommand2(0xBC, 0x08);  // pre-charge voltage
  lcdWriteCommand2(0xBE, 0x07);  // VCOMH
  lcdWriteCommand2(0xB6, 0x01);  // second pre-charge period
  lcdWriteCommand2(0xD5, 0x62);  // function select B
  lcdWriteCommand(0xA4);         // normal display
}

// GDDRAM powers up random: blank all of it (128 rows x 64 column bytes)
static void lcdClearRam()
{
  lcdSetWindow(0, 0x7F, 0, 0x3F);
  LCD_A0_HIGH();
  LCD_NCS_LOW();
  for (uint32_t i = 0; i < 0x2000; i++) spiSend(0);
  spiWaitIdle();
  LCD_NCS_HIGH();
}

void lcdRefreshWait()
{
  while (lcd_busy) {}
}

// displayBuf: LCD_H/8 pages of LCD_W bytes, bit n of a byte = row 8*page+n
static void lcdConvert()
{
  const uint8_t on = lcdGrey;
  for (uint32_t y = 0; y < LCD_H; y++) {
    const uint8_t* page = displayBuf + (y >> 3) * LCD_W;
    const uint8_t bit = 1 << (y & 7);
#if TANGO2_LCD_ROTATE180
    uint8_t* out = oledFrame + (LCD_H - 1 - y) * OLED_COLS_BYTES + OLED_COLS_BYTES - 1;
#else
    uint8_t* out = oledFrame + y * OLED_COLS_BYTES;
#endif
    for (uint32_t x = 0; x < LCD_W; x += 2) {
      uint8_t left = (page[x] & bit) ? on : 0;
      uint8_t right = (page[x + 1] & bit) ? on : 0;
#if TANGO2_LCD_ROTATE180
      // rotated: right pixel becomes the left one of the output pair
      uint8_t a = right, b = left;
      *out-- =
#else
      uint8_t a = left, b = right;
      *out++ =
#endif
#if TANGO2_LCD_LEFT_PIXEL_LOW
          (uint8_t)(a | (b << 4));
#else
          (uint8_t)((a << 4) | b);
#endif
    }
  }
}

void lcdRefresh(bool wait)
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }

  lcdRefreshWait();
  lcdConvert();

  lcdSetWindow(LCD_ROW_OFFSET, LCD_ROW_OFFSET + LCD_H - 1, 0, OLED_COLS_BYTES - 1);

  LCD_A0_HIGH();
  LCD_NCS_LOW();

  lcd_busy = true;
  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN;
  LCD_DMA->HIFCR = LCD_DMA_FLAGS;
  LCD_DMA_Stream->M0AR = (uint32_t)oledFrame;
  LCD_DMA_Stream->NDTR = OLED_FRAME_BYTES;
  LCD_DMA_Stream->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE;
  LCD_SPI->CR2 |= SPI_CR2_TXDMAEN;

  if (wait) lcdRefreshWait();
}

extern "C" void LCD_DMA_Stream_IRQHandler()
{
  DEBUG_INTERRUPT(INT_LCD);

  LCD_DMA_Stream->CR &= ~DMA_SxCR_TCIE;
  LCD_DMA->HIFCR = LCD_DMA_FLAG_INT;
  LCD_SPI->CR2 &= ~SPI_CR2_TXDMAEN;
  LCD_DMA_Stream->CR &= ~DMA_SxCR_EN;

  // TC fires before the last bytes leave the shifter
  while (LCD_SPI->SR & SPI_SR_BSY) {}
  (void)LCD_SPI->DR;
  (void)LCD_SPI->SR;
  LCD_NCS_HIGH();
  lcd_busy = false;
}

void lcdOff()
{
  lcdRefreshWait();
  lcdWriteCommand(0xAE);  // display off
  delay_ms(3);
}

static void lcdReset()
{
  LCD_NCS_HIGH();
  LCD_RST_LOW();
  delay_ms(50);           // Wingman: 50 ms low pulse
  LCD_RST_HIGH();
  delay_ms(50);
}

void lcdInit()
{
  lcdHardwareInit();
  if (IS_LCD_RESET_NEEDED()) {
    lcdReset();
  }
}

void lcdInitFinish()
{
  lcdInitFinished = true;

  if (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
    uint32_t end = timersGetMsTick() + RESET_WAIT_DELAY_MS;
    while (timersGetMsTick() < end);
  }

  lcdStart();
  lcdClearRam();
  lcdWriteCommand(0xAF);  // display on
  delay_ms(20);
}

// "Backlight"/contrast: 0 turns the panel dark, otherwise SSD1327 contrast
void lcdSetRefVolt(uint8_t val)
{
  if (!lcdInitFinished) {
    lcdInitFinish();
  }
  lcdRefreshWait();
  lcdWriteCommand2(0x81, val);
}

void lcdSetInvert(bool invert)
{
  lcdRefreshWait();
  lcdWriteCommand(invert ? 0xA7 : 0xA4);  // inverse / normal (A6 is "all off")
}
