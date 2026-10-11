// tft_setup.h – ILI9341 display pin definitions for ARC-Play (ESP32-S3 N16R8)
//
// Driver : Adafruit_ILI9341 (hardware SPI via Arduino SPI bus)
// SPI bus: shared with the SD-MMC module (different CS pins ensure no conflict)
//
// GPIO assignments:
//   GPIO 11  TFT_MOSI   SPI data out
//   GPIO 12  TFT_SCLK   SPI clock
//   GPIO 13  TFT_MISO   SPI data in  (read-back / touch, currently unused)
//   GPIO 10  TFT_CS     Chip-select  (active LOW)
//   GPIO  9  TFT_DC     Data / command select
//   GPIO  8  TFT_RST    Hardware reset (active LOW)
//   GPIO  7  TFT_BL     Backlight enable (HIGH = on)

#pragma once

// SPI data lines
#define TFT_MOSI  11
#define TFT_SCLK  12
#define TFT_MISO  13

// Control lines
#define TFT_CS    10   // chip-select, pulled HIGH before SPI.begin()
#define TFT_DC     9   // HIGH = pixel data, LOW = command byte
#define TFT_RST    8   // pull LOW to hard-reset the panel

// Backlight
#define TFT_BL             7
#define TFT_BACKLIGHT_ON   HIGH