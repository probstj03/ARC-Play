#pragma once
// ARC-Play pin map on the ARC_AP board (ESP32-S3-DevKitC-1 N16R8).
// Wiring diagram: hardware/wiring/ARC-Play_Verdrahtung.svg
// GPIO35–37 are used by the octal PSRAM and must not be used.

// On-board: PCM5102A (left = song -> J3, right = LTC -> J2)
constexpr int PIN_DAC_BCK  = 18;
constexpr int PIN_DAC_LRCK = 17;
constexpr int PIN_DAC_DIN  = 16;
constexpr int PIN_PCM_XSMT = 3;   // HIGH = output, LOW = soft mute

// On-board: LTC input stage (LM358 + 74HC14)
constexpr int PIN_LTC_IN = 4;

// U2: DS3231 RTC
constexpr int PIN_RTC_SDA = 42;
constexpr int PIN_RTC_SCL = 41;
constexpr int PIN_RTC_SQW = 5;

// J1: micro SD (SD_MMC 1-bit)
constexpr int PIN_SD_CLK = 14;
constexpr int PIN_SD_CMD = 15;
constexpr int PIN_SD_D0  = 2;

// U4: 2.4" ILI9341 TFT (SPI)
constexpr int PIN_TFT_MOSI = 11;
constexpr int PIN_TFT_SCLK = 12;
constexpr int PIN_TFT_MISO = 13;
constexpr int PIN_TFT_CS   = 10;
constexpr int PIN_TFT_DC   = 9;
constexpr int PIN_TFT_RST  = 8;
constexpr int PIN_TFT_BL   = 7;

// SW1: rotary encoder KY-040
constexpr int PIN_ENC_SW  = 40;
constexpr int PIN_ENC_DT  = 39;
constexpr int PIN_ENC_CLK = 38;
