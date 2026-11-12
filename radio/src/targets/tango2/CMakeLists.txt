# TBS Tango 2 (STM32F407VG class, 128x64 monochrome UI on a 128x96 SSD1327-class OLED)
#
# Booted by the resident loader at 0x0800C000 (TBS bootloader at 0x08000000
# stays untouched). EdgeTX lives at 0x08020000, max 640 KB. No EdgeTX
# bootloader. No internal RF module yet: the SX1272 will be driven by the
# ELRS image at 0x080C0000 (abi/elrs_image.h) through spi2_bus.
#
# Build: cmake -DPCB=TANGO2 ...  (two-to-tango: build.sh)

option(PXX1 "PXX1 protocol support" ON)
option(PXX2 "PXX2 protocol support" OFF)
option(AFHDS3 "AFHDS3 TX Module" OFF)
option(GHOST "Ghost TX Module" ON)
option(TANGO2_MLX_GIMBALS "MLX90393 gimbal driver (sticks)" ON)

set(USE_RTC_CLOCK YES)
# Throttle on the left (non-centring) stick: fresh radio.yml starts in Mode 2
set(DEFAULT_MODE 2)
set(HARDWARE_EXTERNAL_MODULE YES)
set(TARGET_DIR tango2)
set(TARGET_SRC_DIR targets/${TARGET_DIR})
set(TARANIS_SRC_DIR targets/taranis)

set(HSE_VALUE 8000000)
set(PPM_LIMITS_SYMETRICAL YES)

# Behaves as an X7-class radio for the GUI, data structures and YAML
add_definitions(-DPCBTARANIS -DPCBX7 -DSOFTWARE_VOLUME)

set(PWR_BUTTON "PRESS" CACHE STRING "Pwr button type (PRESS/SWITCH)")
add_definitions(-DPWR_BUTTON_${PWR_BUTTON})

set(FLAVOUR tango2)
set(CPU_TYPE_FULL STM32F407xG)
set(CPU_TYPE STM32F4)
add_definitions(-DSTM32F40_41xxx -DSTM32F407xx -DSTM32F407xG)
set(TARGET_LINKER_DIR .)
# Own linker scripts: origin 0x08020000, no EdgeTX bootloader, RAM ends at
# 0x20015000 (radio image window above it), CCM from 0x10000400.
set(TARGET_LINKER_ROOT ${RADIO_SRC_DIR}/${TARGET_SRC_DIR}/linker)
# RTC backup RAM (BKPSRAM) would need libclang at build time for datacopy.inc;
# off for now.
set(RTC_BACKUP_RAM NO)

set(HAPTIC YES)
set(GUI_DIR 128x64)
set(BITMAPS_DIR 128x64)
set(BITMAPS_TARGET 9x_bitmaps)
set(FONTS_TARGET 9x_fonts_1bit)
set(MODULE_SIZE_SML YES)
set(ROTARY_ENCODER YES)
# Internal RF: the ELRS image at 0x080E8000 behind a virtual CRSF port
# (radio_host.cpp). FORCE: older build dirs cached NONE.
set(INTERNAL_MODULES CRSF CACHE STRING "Internal modules" FORCE)
set(DEFAULT_INTERNAL_MODULE CROSSFIRE CACHE STRING "Default internal module" FORCE)

# The USB serial CLI is how the PC reboots EdgeTX into the loader
set(CLI ON)
set(USB_SERIAL ON)

# No EdgeTX bootloader: the loader at 0x0800C000 boots and flashes EdgeTX.
set(BOOTLOADER OFF CACHE BOOL "Include Bootloader" FORCE)

if(USE_RTC_CLOCK)
  add_definitions(-DRTCLOCK)
endif()

add_definitions(-DLED_STRIP_LENGTH=1)
# CLI fput/fread/rm/mkdir: SD files from the PC (tools/sd_put.py)
add_definitions(-DCLI_FILE_ACCESS)

set(BOARD_COMMON_SRC
  ${TARGET_SRC_DIR}/board.cpp
  ${TARGET_SRC_DIR}/pwr_driver.cpp
  ${TARGET_SRC_DIR}/loader.cpp
  ${TARGET_SRC_DIR}/lcd_driver.cpp
  ${TARANIS_SRC_DIR}/haptic_driver.cpp
  ${TARANIS_SRC_DIR}/backlight_driver.cpp
  targets/common/arm/stm32/abnormal_reboot.cpp
  targets/common/arm/stm32/delays_driver.cpp
  targets/common/arm/stm32/flash_driver.cpp
  targets/common/arm/stm32/rtc_driver.cpp
  targets/common/arm/stm32/watchdog_driver.cpp
  targets/common/arm/stm32/rotary_encoder_driver.cpp
)

# Bootloader board library (not built: no EdgeTX bootloader on this radio,
# but the common STM32 cmake expects the target to exist)
add_library(board_bl OBJECT EXCLUDE_FROM_ALL
  ${BOARD_COMMON_SRC}
  ${RADIO_SRC_DIR}/gui/common/stdlcd/boot_menu.cpp
)
set(BOOTLOADER_SRC ${BOOTLOADER_SRC} $<TARGET_OBJECTS:board_bl>)

# Firmware board library
add_library(board OBJECT EXCLUDE_FROM_ALL
  ${BOARD_COMMON_SRC}
  ${TARGET_SRC_DIR}/spi2_bus.cpp
  ${TARGET_SRC_DIR}/radio_host.cpp
  ${TARGET_SRC_DIR}/elrs_host.cpp
  ${TARGET_SRC_DIR}/crossfire_host.cpp
  targets/common/arm/stm32/audio_dac_driver.cpp
  targets/common/arm/stm32/heartbeat_driver.cpp
  targets/common/arm/stm32/mixer_scheduler_driver.cpp
  targets/common/arm/stm32/module_timer_driver.cpp
  targets/common/arm/stm32/stm32_pulse_driver.cpp
  targets/common/arm/stm32/stm32_softserial_driver.cpp
  targets/common/arm/stm32/stm32_switch_driver.cpp
  targets/common/arm/stm32/trainer_driver.cpp
  targets/common/arm/stm32/stm32_rgbleds.cpp
)
set(FIRMWARE_SRC ${FIRMWARE_SRC} $<TARGET_OBJECTS:board>)

if(TANGO2_MLX_GIMBALS)
  add_definitions(-DTANGO2_MLX_GIMBALS)
  target_sources(board PRIVATE ${TARGET_SRC_DIR}/mlx90393_gimbal.cpp)
endif()

set(SRC ${SRC}
  io/frsky_firmware_update.cpp
  io/multi_firmware_update.cpp
)

set(GUI_SRC ${GUI_SRC}
  model_input_edit.cpp
  model_mix_edit.cpp
  model_display.cpp
  radio_diagkeys.cpp
  radio_diaganas.cpp
  view_channels.cpp
  view_telemetry.cpp
  view_about.cpp
  bmp.cpp
)

set(RADIO_DEPENDENCIES ${RADIO_DEPENDENCIES} ${BITMAPS_TARGET})
