/*
   TBS Tango 2 memory layout (EdgeTX booted by the resident loader).

   Flash (1 MB part):
     0x08000000-0x0800BFFF  TBS bootloader          (never touched)
     0x0800C000-0x0801FFFF  resident loader/monitor (never touched)
     0x08020000-0x080BFFFF  EdgeTX, vector table at 0x08020000 (640 KB max)
     0x080C0000-0x080E7FFF  TBS Crossfire image (abi/elrs_image.h)
     0x080E8000-0x080FFFFF  ELRS image
   RAM:
     0x20000000-0x20014FFF  EdgeTX (heap, DMA buffers)
     0x20015000-0x2001FFFF  radio image RAM, one image at a time: Crossfire
                            .data/.bss/stack from 0x20015000, ELRS from
                            0x20018000. EdgeTX must not use it.
   CCM 0x10000000-0x100003FF: the Crossfire image's shared struct (API 0x103,
     0x3B4 bytes), found by the image at that fixed address.
   CCM 0x10000400-0x1000FFFF: EdgeTX .data/.bss and main stack.
   RTC BKP19R belongs to the loader's "stay in loader" handshake.

   The generic scripts put the EdgeTX bootloader in front of the firmware;
   there is none here (BOOTLOADER_SIZE = 0), so .isr_vector is the first
   thing in FLASH.
*/

/* Required amount of stack for interrupt stack (Main stack) */
MAIN_STACK_SIZE = 1024;

/* No EdgeTX bootloader in this image */
BOOTLOADER_SIZE = 0;

/* Minimum Heap Size (indicative) */
MIN_HEAP_SIZE = 0;

/* Highest heap address */
HEAP_ADDRESS = ORIGIN(RAM) + LENGTH(RAM);

/* Highest address of the user mode stack */
STACK_ADDRESS = ORIGIN(CCM) + LENGTH(CCM); /* end of CCM */

MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08020000, LENGTH = 640K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 84K   /* 0x20015000+ is the radio images' */
  CCM (xrw)       : ORIGIN = 0x10000400, LENGTH = 63K   /* 0x10000000+ is the API 0x103 struct */
}

REGION_ALIAS("REGION_BOOTLOADER",   FLASH);
REGION_ALIAS("REGION_ISR_VECT",     FLASH);
REGION_ALIAS("REGION_TEXT",         FLASH);
REGION_ALIAS("REGION_TEXT_STORAGE", FLASH);

REGION_ALIAS("REGION_DATA", CCM);
REGION_ALIAS("REGION_BSS", CCM);
REGION_ALIAS("REGION_RAM", RAM);

/* Linker-time guards */
ASSERT(ORIGIN(RAM) + LENGTH(RAM) <= 0x20015000, "EdgeTX RAM overlaps the radio image window")
ASSERT(ORIGIN(CCM) >= 0x100003B4, "EdgeTX CCM overlaps the Crossfire API 0x103 struct")
