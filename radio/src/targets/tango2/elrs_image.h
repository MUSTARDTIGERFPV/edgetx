// Interface between a host (EdgeTX, or the resident loader/monitor for
// development) and the ExpressLRS TX image in flash at 0x080E8000.
// Both sides include this file. Change it only by agreement between the two
// sides; bump ELRS_ABI_MAJOR for incompatible changes.
#pragma once
#include <stdint.h>

#define ELRS_ABI_MAJOR 1
#define ELRS_ABI_MINOR 1

// Sectors 10-11 (0x080C0000-0x080FFFFF) hold both radio images, written
// together as one 256 KB slot file. The TBS Crossfire image is linked to run
// at 0x080C0000 and needs 0x20F70 bytes (v6.30); it gets 160 KB so later
// builds fit, and ELRS sits after it. Only one image runs at a time and they
// share the RAM window below.
#define CROSSFIRE_IMAGE_ADDR 0x080C0000u
#define CROSSFIRE_IMAGE_MAX  0x00028000u
#define ELRS_IMAGE_ADDR      0x080E8000u
#define ELRS_IMAGE_MAX       0x00018000u
#define ELRS_IMAGE_MAGIC     0x3252454Cu   // "LER2"

// RAM the image owns for .data/.bss (the host never touches it while the
// image is loaded). Same addresses under EdgeTX and under the monitor.
#define ELRS_IMAGE_RAM_START 0x20018000u
#define ELRS_IMAGE_RAM_END   0x20020000u   // 32 KB

// SPI2 devices on the Tango 2 (CS pin, SPI mode and speed chosen by the host).
enum { ELRS_SPI_SX1272 = 0, ELRS_SPI_GIMBAL0 = 1, ELRS_SPI_GIMBAL1 = 2 };

typedef struct {
  uint16_t abi_major, abi_minor;

  // SPI2, shared with the gimbals. One call is one chip-select-framed transfer,
  // atomic w.r.t. other SPI2 users (the host masks IRQs for its duration).
  // tx or rx may be NULL. n <= 64.
  void (*spi_xfer)(uint8_t dev, const uint8_t *tx, uint8_t *rx, uint16_t n);

  // CRSF with the handset side (EdgeTX's internal-module stack, or the PC
  // through the monitor). Whole frames only: [addr][len][type][payload][crc].
  // crsf_read returns 0 when nothing is pending. Safe from loop() only.
  uint16_t (*crsf_read)(uint8_t *buf, uint16_t max);
  void (*crsf_write)(const uint8_t *frame, uint16_t len);

  // Time. micros() wraps at 2^32 and is safe from interrupts.
  uint32_t (*micros)(void);
  uint32_t (*millis)(void);
  void (*delay_us)(uint32_t us);

  // Persistent config blob (ELRS TX config). The host stores it (SD card under
  // EdgeTX, RAM-only under the monitor) and may do so slowly; never call from
  // an interrupt. Returns bytes read / 0 on success, negative on error.
  int (*config_load)(void *buf, uint32_t len);
  int (*config_save)(const void *buf, uint32_t len);

  // Diagnostics: printf-style text, newline-terminated lines. Never from IRQ.
  void (*log)(const char *fmt, ...);

  // Per-unit crystal error in 0.1 ppm units (-159 = -15.9 ppm), 0 if unknown.
  int16_t (*crystal_ppm_x10)(void);
  // Hardware revision index from the TBS hwId (0x0004xx05 -> 5).
  uint8_t (*hw_rev)(void);
} elrs_host_api_t;

typedef struct {
  uint32_t magic;            // ELRS_IMAGE_MAGIC
  uint16_t abi_major, abi_minor;
  uint32_t image_size;       // bytes from ELRS_IMAGE_ADDR, header included
  uint32_t image_crc32;      // zlib CRC-32 over [sizeof(header), image_size)
  const char *version;       // e.g. "4.1.0-153 tango2 <git>"
  uint32_t ram_start, ram_end;
  uint32_t data_load;        // flash address of the .data initialiser
  // Called with interrupts disabled, after the host has set up clocks, GPIO
  // for everything it owns and SPI2. The image copies .data, zeroes .bss,
  // configures its own pins/timer/EXTI (pins the host doesn't use) and
  // returns 0 on success.
  int  (*init)(const elrs_host_api_t *host);
  void (*start)(void);       // interrupts on; begin transmitting
  void (*loop)(void);        // called repeatedly from a normal-priority context
  void (*stop)(void);        // radio to standby, RF switch RX, PA2 high, IRQs off
  // Interrupt entry points; the host patches its (RAM) vector table with them
  // after init() succeeds. NULL = unused.
  void (*irq_exti15_10)(void);   // DIO0..5 on PE10..15
  void (*irq_timer)(void);       // the image's hwTimer
  uint8_t timer_irqn;            // NVIC number of that timer's interrupt
  uint8_t reserved[3];
} elrs_image_header_t;

// The CRC covers [ELRS_IMAGE_HEADER_SIZE, image_size); abi/finish_image.py fills
// image_size and image_crc32 after linking.
#define ELRS_IMAGE_HEADER_SIZE 60
#ifdef __cplusplus
static_assert(sizeof(elrs_image_header_t) == ELRS_IMAGE_HEADER_SIZE, "elrs_image_header_t layout changed");
#else
_Static_assert(sizeof(elrs_image_header_t) == ELRS_IMAGE_HEADER_SIZE, "elrs_image_header_t layout changed");
#endif
