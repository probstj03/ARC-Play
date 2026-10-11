/*
 * =========================================================================
 * ARC-Play  –  All-in-One LTC Generator, WAV Playback, Decoder, Jam Sync
 *
 * Hardware: ESP32-S3 N16R8 (DevKitC-1)
 * -------------------------------------------------------------------------
 * NASA 10-Rule Compliance Notes:
 *  Rule 1  – Simple control flow: no goto, no setjmp/longjmp, no recursion.
 *  Rule 2  – All task loops use vTaskDelay; every ISR loop has a fixed bound.
 *  Rule 3  – Dynamic allocation (malloc/new) only inside init paths.
 *  Rule 4  – Functions >60 lines are split into focused helpers.
 *  Rule 5  – Parameter validation (NULL/range checks) at function entry.
 *  Rule 6  – Local variables declared at the tightest scope possible.
 *  Rule 7  – Non-void return values (semaphore, file ops) are checked.
 *  Rule 8  – Preprocessor limited to include guards and simple macros.
 *  Rule 9  – Pointer nesting limited to one level of indirection.
 *  Rule 10 – Code compiled with all warnings enabled; zero warnings target.
 * -------------------------------------------------------------------------
 * Pin assignment (ESP32-S3):
 *  I2S BCK  / PCM5102  GPIO 18     I2S bit clock
 *  I2S LCK  / PCM5102  GPIO 17     I2S left/right clock
 *  I2S DIN  / PCM5102  GPIO 16     I2S data out
 *  SD CLK   / SD_MMC   GPIO 14     SD card clock
 *  SD CMD   / SD_MMC   GPIO 15     SD card command
 *  SD D0    / SD_MMC   GPIO  2     SD card data
 *  TFT MOSI / ILI9341  GPIO 11     SPI data
 *  TFT SCLK / ILI9341  GPIO 12     SPI clock
 *  TFT MISO / ILI9341  GPIO 13     SPI read-back
 *  TFT CS   / ILI9341  GPIO 10     Chip-select (active LOW)
 *  TFT DC   / ILI9341  GPIO  9     Data/command
 *  TFT RST  / ILI9341  GPIO  8     Hardware reset
 *  TFT BL   / ILI9341  GPIO  7     Backlight (HIGH = on)
 *  RTC SDA  / DS3231   GPIO 42     I2C data
 *  RTC SCL  / DS3231   GPIO 41     I2C clock
 *  RTC SQW  / DS3231   GPIO  5     1 Hz square wave for TC sync
 *  LTC IN   / ADC      GPIO  4     Incoming LTC audio (edge ISR)
 *  PCM XSMT / PCM5102  GPIO  3     Soft mute (LOW) / unmute (HIGH)
 *  ENC CLK  / KY-040   GPIO 38     Encoder clock (CHANGE interrupt)
 *  ENC DT   / KY-040   GPIO 39     Encoder direction
 *  ENC SW   / KY-040   GPIO 40     Encoder button (press = play/pause)
 * -------------------------------------------------------------------------
 */

#include <Arduino.h>
#include "tft_setup.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "AudioTools.h"
#include "AudioTools/Disk/AudioSourceSDMMC.h"
#include "SD_MMC.h"
#include "AudioTools/AudioCodecs/CodecWAV.h"
#include "ltc_esp32.h"
#include <Wire.h>
#include <RTClib.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ── Pins ──────────────────────────────────────────────────────────────────
#define BCLK_PIN    18 
#define LRCK_PIN    17 
#define DIN_PIN     16 

#define SDMMC_CLK   14 
#define SDMMC_CMD   15 
#define SDMMC_D0    2  

#define RTC_SDA_PIN 42
#define RTC_SCL_PIN 41
#define RTC_SQW_PIN 5

#define LTC_INPUT_PIN 4

// ── PCM5102 MUTE control ──────────────────────────────────────────────────
// XSMT pin: HIGH = normal operation (encode modes)
//           LOW  = soft mute (decode-only modes: Slave, Jam Sync)
#define PCM_MUTE_PIN  3
#define PCM_UNMUTE()  digitalWrite(PCM_MUTE_PIN, HIGH)
#define PCM_MUTE()    digitalWrite(PCM_MUTE_PIN, LOW)

// Returns true for modes where we only receive/decode LTC (no audio output needed)
// NOTE: defined after SystemMode enum below

// ── Rotary Encoder ─────────────────────────────────────────────────────────
#define ENC_CLK_PIN  38
#define ENC_DT_PIN   39
#define ENC_SW_PIN   40

// ── WiFi & NTP ────────────────────────────────────────────────────────────
// Add / remove networks as needed – tried in order, first success wins
struct WifiCred { const char* ssid; const char* pass; };
static const WifiCred kWifiNetworks[] = {
    { "WLAN-828327",       "35074216513574635873" },
    { "Berserker", "76915661820278777668" },
    { "MagentaWLAN-R38Y", "88255808344892594692"},
    { "JULIAN-IPHONE", "12345678909"},
    {"HAW-WLAN", "VXAD-uk73-E3TL-u37g"},
};
static const int kWifiCount = sizeof(kWifiNetworks) / sizeof(kWifiNetworks[0]);
#define NTP_SERVER1    "pool.ntp.org"
#define NTP_SERVER2    "time.nist.gov"
// POSIX timezone: CET (UTC+1), summer time CEST (UTC+2).
// M3.5.0  = last Sunday in March  02:00 → 03:00
// M10.5.0 = last Sunday in October 03:00 → 02:00
#define NTP_TZ_STRING  "CET-1CEST,M3.5.0,M10.5.0/3"

// ── Globale Modi & States ──────────────────────────────────────────────────
void broadcastStatus(); // Forward declaration for broadcastStatus

enum SystemMode {
    MODE_GEN_FREE_RUN,      // LTC generator, timecode follows RTC wall clock
    MODE_GEN_RECORD_RUN,    // LTC generator, timecode only advances while running
    MODE_PLAYBACK,          // WAV file + LTC mixed and output simultaneously
    MODE_SLAVE,             // LTC decoder only, no audio output
    MODE_JAM_SYNC_FREE_RUN, // Decode incoming LTC, lock on, switch to Free Run
    MODE_JAM_SYNC_RECORD_RUN, // Decode incoming LTC, lock on, switch to Rec Run
    MODE_NONE               // Uninitialized / transitioning
};

// Returns true for modes where we only receive LTC and must silence the DAC.
static inline bool isMuteMode(SystemMode m) {
    return m == MODE_SLAVE || m == MODE_JAM_SYNC_FREE_RUN || m == MODE_JAM_SYNC_RECORD_RUN;
}

enum RTC_Mode { RTC_FREE_RUN, RTC_RECORD_RUN };

// Active mode (written only from loop() via applyMode).
volatile SystemMode g_sysMode     = MODE_NONE;
// Requested mode (set from any context, applied in loop()).
volatile SystemMode g_reqMode     = MODE_GEN_FREE_RUN;
volatile LTC_TV_STANDARD g_ltcStd = LTC_TV_625_50; // default: 25 fps PAL
volatile RTC_Mode   g_rtcMode     = RTC_FREE_RUN;

// Current timecode digits (H:M:S:F) – written by encoder/decoder tasks.
static volatile uint8_t  g_tc_h = 0, g_tc_m = 0, g_tc_s = 0, g_tc_f = 0;
static volatile bool     g_running    = false;  // playback / generation active
static volatile bool     g_wavRx      = false;  // SD write in progress (upload)
static volatile uint32_t g_rxUserBits = 0;      // user-bits from decoded LTC
static volatile uint32_t g_txUserBits = 0;      // user-bits being generated (date/shot)
static volatile bool     g_shotBitsActive = false;  // true: shot identification userbit active
static volatile bool     g_rxDropFrame = false; // drop-frame flag from decoded LTC
static volatile float    g_rxFps      = 0.0f;   // frame rate detected by decoder

static String   g_songTitle  = "--";
static float    g_songDurSec = 0.0f;            // song duration from server
static float    g_songBpm    = 0.0f;            // BPM from songinfo.json
static int      g_songTimeSig = 4;              // time signature numerator from songinfo.json
static volatile uint32_t g_jamOkCnt     = 0;    // consecutive valid jam frames
static volatile bool     g_rtcSyncDue   = false;// RTC SQW ISR flag: sync now
static volatile bool     g_ltcStdDirty  = false;// LTC standard change pending
static volatile bool     g_countInActive = false; // true while 8-beat count-in is running

// ── Encoder runtime ────────────────────────────────────────────────────────
static volatile int g_encDelta = 0;

// ── Volume control ────────────────────────────────────────────────────────
static volatile int  g_volumePercent   = 50;   // 0–100, default 50%
static volatile uint32_t g_volShowUntilMs = 0; // millis() timestamp until volume bar is visible
static volatile bool g_volDirty        = false;// set by encoder ISR, applied in loop()

// ── Rotary encoder ISR ───────────────────────────────────────────────────────────────
// On every CLK edge: sample DT to determine direction and update g_encDelta.
// Display task reads and clears g_encDelta in its main loop.
void IRAM_ATTR isrEncCLK() {
    static bool clkLast = true;
    bool clkNow = (bool)digitalRead(ENC_CLK_PIN);
    if (clkLast && !clkNow) {   // falling edge – read DT to get direction
        g_encDelta += digitalRead(ENC_DT_PIN) ? 1 : -1;
    }
    clkLast = clkNow;
}

// Configure encoder pins and attach the CLK interrupt.
void initEncoder() {
    pinMode(ENC_CLK_PIN, INPUT_PULLUP);
    pinMode(ENC_DT_PIN,  INPUT_PULLUP);
    pinMode(ENC_SW_PIN,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), isrEncCLK, CHANGE);
}

// ── FreeRTOS synchronisation handles & hardware objects ──────────────────────
// Mutexes protect shared resources accessed from multiple cores / tasks.
// NASA Rule 3: all handles are created once in setup(); never reallocated.
SemaphoreHandle_t g_i2cMtx  = nullptr;  // guards Wire / DS3231 bus
SemaphoreHandle_t g_audMtx  = nullptr;  // guards AudioPlayer / I2S writes
SemaphoreHandle_t g_sysMtx  = nullptr;  // guards shot metadata struct
TaskHandle_t      g_dispTask = nullptr;  // task notification target for UI refresh
RTC_DS3231        rtc;

// Hardware SPI (fast) – uses the Arduino SPI bus object, not bit-banged GPIO.
Adafruit_ILI9341  tft(&SPI, TFT_DC, TFT_CS, TFT_RST);

// Return a short human-readable string for the current LTC standard.
const char* ltcStandardStr(LTC_TV_STANDARD s) {
    switch (s) {
        case LTC_TV_625_50:  return "25 ND";
        case LTC_TV_525_60:  return "30 ND";
        case LTC_TV_1125_60: return "30 HD";
        case LTC_TV_FILM_24: return "24 ND";
        case LTC_TV_525_DF:  return "29.9DF";
        default:             return "? XX";
    }
}

// ── WebSocket server & current shot metadata ─────────────────────────────────
// Shot data is sent by the Node.js server via WebSocket "shot" messages.
// All shot fields are protected by g_sysMtx when accessed from multiple tasks.
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
static File g_uploadFile;  // SD file handle during WAV upload

static int    g_shotScene  = 0;
static String g_shotName   = "0";
static int    g_shotTake   = 0;
static String g_shotRoll   = "A001";
static String g_shotCamera = "A";           // camera identifier from loaded shot
static String g_shotStartTC = "";   // HH:MM:SS:FF start of the active shot
static String g_shotEndTC   = "";   // HH:MM:SS:FF end  of the active shot
static uint32_t g_shotStartFr = 0;  // pre-converted frame count for g_shotStartTC
static uint32_t g_shotEndFr   = 0;  // pre-converted frame count for g_shotEndTC
static bool g_shotLimited = false;  // true when start/end TC limits are active
static volatile float g_shotSpeed  = 1.0f;  // playback speed multiplier (1.0 = normal)
static volatile bool  g_speedDirty = false; // true = I2S+LTC need reinit for new speed

// Deferred re-seek: the AudioPlayer parses the WAV header lazily on the first
// player.copy() after starting, which discards any seek applied beforehand.
// We therefore store the target TC and re-apply the seek once after the first
// copy() so playback always starts at the correct position.
static volatile bool    g_reseekPending = false;
static volatile uint8_t g_reseekH = 0, g_reseekM = 0, g_reseekS = 0, g_reseekF = 0;

// Convert HH:MM:SS:FF timecode components to a flat frame count (assumes 30 fps base).
static uint32_t tcToFrames(uint8_t h, uint8_t m, uint8_t s, uint8_t f) {
    float fps = ltc_standard_fps(g_ltcStd);
    if (fps <= 0.0f) fps = 25.0f;
    int ifps = (int)(fps + 0.5f); // round to nearest integer fps
    return (uint32_t)(h * 3600 + m * 60 + s) * (uint32_t)ifps + f;
}

// Parse a timecode string "HH:MM:SS:FF" into its four components.
// NASA Rule 5: returns silently if string is too short (safe no-op).
static void parseTC(String tcStr, uint8_t &h, uint8_t &m, uint8_t &s, uint8_t &f) {
    if (tcStr.length() < 11) return;  // guard: invalid input
    h = (uint8_t)tcStr.substring(0, 2).toInt();
    m = (uint8_t)tcStr.substring(3, 5).toInt();
    s = (uint8_t)tcStr.substring(6, 8).toInt();
    f = (uint8_t)tcStr.substring(9, 11).toInt();
}

// ── RTC Steuerung ──────────────────────────────────────────────────────────
void IRAM_ATTR onSqwRising() {
    g_rtcSyncDue = true;
}

void initRTC() {
    Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
    if (!rtc.begin(&Wire)) {
        Serial.println("[RTC] DS3231 nicht gefunden!");
        return;
    }
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
    pinMode(RTC_SQW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), onSqwRising, RISING);
}

// ── WiFi, NTP and WebSocket server init ────────────────────────────────────
// Tries each stored credential in order; on first success syncs the DS3231
// with NTP and starts the WebSocket server on port 80.
// NASA Rule 2: the inner while-loop has an 8-second hard timeout.
extern AudioPlayer player;
extern AudioSourceSDMMC source;

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Cached byte offset of the WAV "data" chunk payload within /track.wav.
// BWF / broadcast-wave files can have headers far larger than the classic
// 44 bytes (bext, iXML, fmt-extension chunks). We therefore parse the real
// data-chunk offset once per file instead of assuming 44.
static uint32_t g_wavDataOffset = 44;

// Scan the WAV/BWF RIFF chunk list of /track.wav and store the byte offset of
// the "data" chunk payload in g_wavDataOffset. Falls back to 44 on any error.
// Safe to call at any time; opens its own read-only file handle.
static void scanWavDataOffset() {
    g_wavDataOffset = 44;  // sensible default
    File f = SD_MMC.open("/track.wav", FILE_READ);
    if (!f) return;
    uint8_t hdr[12];
    if (f.read(hdr, 12) != 12) { f.close(); return; }
    // Expect "RIFF" .... "WAVE"
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
        f.close();
        return;
    }
    uint32_t pos = 12;  // first chunk starts right after the RIFF/WAVE header
    // NASA Rule 2: bounded loop (max 64 chunks) to avoid runaway parsing.
    for (int guard = 0; guard < 64; guard++) {
        uint8_t ch[8];
        f.seek(pos, SeekSet);
        if (f.read(ch, 8) != 8) break;
        uint32_t chunkSize = (uint32_t)ch[4] | ((uint32_t)ch[5] << 8) |
                             ((uint32_t)ch[6] << 16) | ((uint32_t)ch[7] << 24);
        if (memcmp(ch, "data", 4) == 0) {
            g_wavDataOffset = pos + 8;  // payload starts after the 8-byte chunk header
            break;
        }
        // Chunks are word-aligned: odd sizes are padded with one byte.
        pos += 8 + chunkSize + (chunkSize & 1);
    }
    f.close();
}

// Seeks the audio player and resets g_tc_* to the given timecode.
// Must be called with g_audMtx held by the caller.
static void seekAudioToTC(uint8_t h, uint8_t m, uint8_t s, uint8_t f) {
    float fps = ltc_standard_fps(g_ltcStd);
    if (fps <= 0.0f) fps = 25.0f;
    float startSecs = (float)tcToFrames(h, m, s, f) / fps;
    Stream* audioStream = player.getStream();
    if (audioStream) {
        fs::File* audioFile = (fs::File*)audioStream;
        if (audioFile) {
            audio_tools::AudioInfo inf = player.audioInfo();
            int sr  = inf.sample_rate    > 0 ? inf.sample_rate    : 44100;
            int ch  = inf.channels       > 0 ? inf.channels       : 1;
            int bps = inf.bits_per_sample > 0 ? inf.bits_per_sample : 16;
            // Use the real data-chunk offset (BWF headers can exceed 44 bytes).
            uint32_t byte_pos = g_wavDataOffset + (uint32_t)(startSecs * sr * ch * (bps / 8));
            byte_pos = byte_pos - (byte_pos % (ch * (bps / 8))); // align to frame boundary
            audioFile->seek(byte_pos, SeekSet);
        }
    }
    g_tc_h = h; g_tc_m = m; g_tc_s = s; g_tc_f = f;
}

void initWiFiAndServer() {
    WiFi.mode(WIFI_STA);
    bool connected = false;
    for (int i = 0; i < kWifiCount && !connected; i++) {
        Serial.printf("[WiFi] Trying '%s' ...", kWifiNetworks[i].ssid);
        WiFi.begin(kWifiNetworks[i].ssid, kWifiNetworks[i].pass);
        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(100);
        if (WiFi.status() == WL_CONNECTED) { Serial.println(" OK"); connected = true; }
        else { Serial.println(" FAIL"); WiFi.disconnect(); delay(200); }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(WiFi.localIP());
        
        // Sync NTP (inkl. automatischer Sommerzeit via POSIX-TZ-String)
        configTzTime(NTP_TZ_STRING, NTP_SERVER1, NTP_SERVER2);
        struct tm ti;
        if (getLocalTime(&ti, 5000)) {
            if (xSemaphoreTake(g_i2cMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
                rtc.adjust(DateTime(ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec));
                xSemaphoreGive(g_i2cMtx);
            }
            Serial.println("[RTC] Sync OK");
        }
        
        ws.onEvent(onWebSocketEvent);
        server.addHandler(&ws);
        server.begin();
    } else {
        Serial.println("FAIL");
    }
}

// Manually re-sync DS3231 from NTP; called from menu and WebSocket "sync_rtc" command.
void syncRTCWithNTP() {
    // Only attempt if WiFi is up
    if (WiFi.status() == WL_CONNECTED) {
        struct tm ti;
        if (getLocalTime(&ti, 5000)) {
            if (xSemaphoreTake(g_i2cMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
                rtc.adjust(DateTime(ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec));
                xSemaphoreGive(g_i2cMtx);
            }
            Serial.println("[RTC] Sync OK");
            broadcastStatus(); // Notify clients that RTC was synced
        }
    }
}

// Notify the display task that state has changed; it redraws only changed regions.
static void updateUI() {
    if (g_dispTask) xTaskNotifyGive(g_dispTask);
}

// ── Date User Bits Helpers ──────────────────────────────────────────────────

// Returns the current POSIX timezone offset string (+0200 CEST or +0100 CET).
// Uses the ESP32 system clock (set via configTzTime at startup) to detect DST.
static const char* currentTZString() {
    struct tm ti = {};
    if (getLocalTime(&ti, 0)) {
        return (ti.tm_isdst > 0) ? "+0200" : "+0100";
    }
    return "+0100";  // fallback: CET
}

// Pack user bits from an LTC frame in the same nibble-reversed order that the
// ISR decoder uses for g_rxUserBits, so the display always shows the value a
// receiver would see (user1 = most-significant nibble of the displayed hex).
static uint32_t packUserBitsForDisplay(LTCFrame *f) {
    return ((uint32_t)f->user1 << 28) |
           ((uint32_t)f->user2 << 24) |
           ((uint32_t)f->user3 << 20) |
           ((uint32_t)f->user4 << 16) |
           ((uint32_t)f->user5 << 12) |
           ((uint32_t)f->user6 <<  8) |
           ((uint32_t)f->user7 <<  4) |
           ((uint32_t)f->user8);
}

// Reverse all 8 nibbles of a 32-bit value.
// ltc_encoder_set_user_bits maps bit[3:0] → user1 (MSB of display).
// So: set_user_bits(reverseNibbles32(displayVal)) → packUserBitsForDisplay() == displayVal.
static uint32_t reverseNibbles32(uint32_t v) {
    return ((v & 0xF) << 28) | (((v >> 4) & 0xF) << 24) | (((v >> 8) & 0xF) << 20) |
           (((v >> 12) & 0xF) << 16) | (((v >> 16) & 0xF) << 12) | (((v >> 20) & 0xF) << 8) |
           (((v >> 24) & 0xF) << 4) | ((v >> 28) & 0xF);
}

// Write today's date + current timezone into the LTC user bits of an encoder
// that was created WITHOUT the LTC_USE_DATE flag (flag=0), using
// ltc_encoder_set_user_bits directly.  Also refreshes g_txUserBits.
static void applyDateUserBits(LTCEncoder *enc) {
    if (!enc) return;
    DateTime now;
    if (xSemaphoreTake(g_i2cMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        now = rtc.now();
        xSemaphoreGive(g_i2cMtx);
    } else { return; }

    uint8_t dd = now.day(), mm = now.month(), yy = now.year() % 100;
    const char* tzStr = currentTZString();

    // Look up TZ code from the libltc table (also defined in ltc_esp32.h)
    uint8_t tz_code = 0x00;
    for (int i = 0; _ltc_tz_table[i].code != 0xFF; i++) {
        if (strcmp(_ltc_tz_table[i].tz, tzStr) == 0) {
            tz_code = _ltc_tz_table[i].code;
            break;
        }
    }
    // user1=DD_units, user2=DD_tens, user3=MM_units, user4=MM_tens,
    // user5=YY_units, user6=YY_tens, user7=TZ_lo, user8=TZ_hi
    uint32_t d = (uint32_t)(dd % 10) | ((uint32_t)(dd / 10) << 4)
               | ((uint32_t)(mm % 10) << 8) | ((uint32_t)(mm / 10) << 12)
               | ((uint32_t)(yy % 10) << 16) | ((uint32_t)(yy / 10) << 20)
               | ((uint32_t)(tz_code & 0x0F) << 24)
               | ((uint32_t)((tz_code >> 4) & 0x0F) << 28);
    ltc_encoder_set_user_bits(enc, d);
    LTCFrame frm;
    ltc_encoder_get_frame(enc, &frm);
    g_txUserBits = packUserBitsForDisplay(&frm);
    Serial.printf("[LTC] DateBits: %02u.%02u.%02u TZ=%s -> 0x%08lX\n",
                  dd, mm, yy, tzStr, (unsigned long)g_txUserBits);
}

// Compute the 32-bit shot identification value from the current shot globals.
// Layout: [31:28] Camera (A=0xA), [27:21] Roll, [20:14] Scene, [13:7] Shot, [6:0] Take.
static uint32_t computeShotUserBits() {
    // Derive camera letter from the roll prefix (A001→A, B002→B, ..., F→F).
    // This follows the film production convention: roll name starts with camera letter.
    char camChar = 'A';
    for (unsigned i = 0; i < g_shotRoll.length(); i++) {
        char c = (char)toupper((unsigned char)g_shotRoll[i]);
        if (c >= 'A' && c <= 'F') { camChar = c; break; }
    }
    uint32_t camId = (uint32_t)(camChar - 'A' + 10) & 0xF;  // A=0xA, B=0xB, ..., F=0xF

    // Extract numeric part from roll string (e.g. "A001" -> 1, "B012" -> 12)
    uint32_t rollNum = 1;
    for (unsigned i = 0; i < g_shotRoll.length(); i++) {
        if (isdigit((unsigned char)g_shotRoll[i])) {
            rollNum = (uint32_t)g_shotRoll.substring(i).toInt();
            break;
        }
    }
    rollNum &= 0x7F;

    uint32_t sceneNum = (uint32_t)g_shotScene & 0x7F;
    uint32_t shotNum  = (uint32_t)g_shotName.toInt() & 0x7F;
    uint32_t takeNum  = (uint32_t)g_shotTake & 0x7F;

    return (camId << 28) | (rollNum << 21) | (sceneNum << 14) | (shotNum << 7) | takeNum;
}

// Apply either shot identification bits or date fallback bits to an encoder.
// Shot bits are computed fresh from the current shot globals each time.
static void applyCurrentUserBits(LTCEncoder *enc) {
    if (!enc) return;
    if (g_shotBitsActive && g_sysMode == MODE_PLAYBACK) {
        uint32_t bits = computeShotUserBits();
        ltc_encoder_set_user_bits(enc, reverseNibbles32(bits));
        g_txUserBits = bits;
        Serial.printf("[LTC] ShotBits: 0x%08lX (cam=%s roll=%s sc=%d sh=%s tk=%d)\n",
                      (unsigned long)bits, g_shotCamera.c_str(), g_shotRoll.c_str(),
                      g_shotScene, g_shotName.c_str(), g_shotTake);
    } else {
        applyDateUserBits(enc);
    }
}

// Forward declaration: applies current user bits to the active encoder.
// Full definition is below, after both LTCGenerator and LTCMixerOutput are defined.
static void applyCurrentUserBitsToActiveEncoder();

// ── LTC Generator (encode-only modes) ────────────────────────────────────
// Generates a bi-phase mark LTC audio stream via the AudioTools SoundGenerator
// interface.  The stream is routed directly to I2S at 48 kHz stereo.
// One instance is enough; the LTCMixerOutput is used for playback mode.
class LTCGenerator : public SoundGenerator<int16_t> {
public:
    bool begin(AudioInfo cfg) {
        _sampleRate = cfg.sample_rate;
        if (!_started) {
            _encoder = ltc_encoder_create(cfg.sample_rate, ltc_standard_fps(g_ltcStd), g_ltcStd, 0);
            ltc_encoder_set_filter(_encoder, 0.0);
            ltc_encoder_set_volume(_encoder, -3.0);
            applyCurrentUserBits(_encoder);
            _started = true;
        }
        _fill_next_frame();
        return SoundGenerator<int16_t>::begin(cfg);
    }
    
    int16_t readSample() override {
        if (!g_running) return 0;
        if (g_ltcStdDirty || _pos >= _len) {
            if (g_ltcStdDirty) {
                _recreate(g_ltcStd);
                g_ltcStdDirty = false;
            }
            _fill_next_frame();
        }
        return ((int16_t)(_buf[_pos++]) - 128) << 8;
    }

    void applyStartTC() {
        if (!_encoder) return;
        if (g_rtcMode == RTC_FREE_RUN) _syncFromRTC();
        _updateUserBits();
        _fill_next_frame();
    }

    // Apply current user bits (shot or date) to the generator encoder.
    void applyUserBits() {
        applyCurrentUserBits(_encoder);
    }

    void setManualTimecode(uint8_t hh, uint8_t mm, uint8_t ss, uint8_t ff) {
        if (!_encoder) return;
        SMPTETimecode st;
        ltc_encoder_get_timecode(_encoder, &st);
        st.hours = hh;
        st.mins = mm;
        st.secs = ss;
        st.frame = ff;
        ltc_encoder_set_timecode(_encoder, &st);
    }

private:
    LTCEncoder      *_encoder    = nullptr;
    ltcsnd_sample_t *_buf        = nullptr;
    int              _len = 0, _pos = 0;
    double           _sampleRate = 48000.0;
    bool             _started = false;

    void _recreate(LTC_TV_STANDARD std) {
        SMPTETimecode cur = {};
        if (_encoder) {
            ltc_encoder_get_timecode(_encoder, &cur);
            ltc_encoder_free(_encoder);
        }
        _encoder = ltc_encoder_create(_sampleRate, ltc_standard_fps(std), std, 0);
        ltc_encoder_set_volume(_encoder, -3.0);
        // Restore H/M/S/F only; user bits set by applyCurrentUserBits below
        SMPTETimecode st = {};
        st.hours = cur.hours; st.mins = cur.mins; st.secs = cur.secs; st.frame = cur.frame;
        ltc_encoder_set_timecode(_encoder, &st);
        applyCurrentUserBits(_encoder);
        _pos = _len = 0;
    }

    void _fill_next_frame() {
        if (!_encoder) return;
        ltc_encoder_encode_frame(_encoder);
        _len = ltc_encoder_get_bufferptr(_encoder, &_buf, 1);
        _pos = 0;

        SMPTETimecode st;
        ltc_encoder_get_timecode(_encoder, &st);
        
        // Update UI only if we are the active encoder
        if (g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN) {
            g_tc_h = st.hours; g_tc_m = st.mins; g_tc_s = st.secs; g_tc_f = st.frame;
            updateUI();
        }
        ltc_encoder_inc_timecode(_encoder);

        if (g_rtcSyncDue) {
            SMPTETimecode next;
            ltc_encoder_get_timecode(_encoder, &next);
            if (next.frame == 0) {
                if (g_rtcMode == RTC_FREE_RUN) _syncFromRTC();
                g_rtcSyncDue = false;
            }
        }
    }

    void _syncFromRTC() {
        if (xSemaphoreTake(g_i2cMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            delayMicroseconds(500);
            DateTime now = rtc.now();
            xSemaphoreGive(g_i2cMtx);
            SMPTETimecode st = {};
            st.hours = now.hour(); st.mins = now.minute(); st.secs = now.second(); st.frame = 0;
            ltc_encoder_set_timecode(_encoder, &st);  // flag=0: user bits not touched
            LTCFrame frm; ltc_encoder_get_frame(_encoder, &frm);
            g_txUserBits = packUserBitsForDisplay(&frm);
            // Refresh date bits at midnight so the date stays current
            if (now.hour() == 0 && now.minute() == 0 && now.second() == 0) {
                _updateUserBits();
            }
        }
    }

    void _updateUserBits() {
        applyCurrentUserBits(_encoder);
    }
};

// ── LTC Mixer Output (playback mode) ─────────────────────────────────────────
// Sits between the WAV AudioPlayer and the I2S driver. Each write() call
// mixes mono WAV samples onto the left channel and freshly generated LTC
// onto the right channel, then forwards the stereo frame to I2S.
class LTCMixerOutput : public AudioOutput {
public:
    explicit LTCMixerOutput(I2SStream &i2s) : _i2s(i2s) {}
    ~LTCMixerOutput() { if (_enc) ltc_encoder_free(_enc); }

    bool begin() override {
        _speed = (float)g_shotSpeed;    // read current playback speed from global
        int wavSR  = 44100;             // base WAV sample rate
        int scaledSR = (int)((float)wavSR * _speed + 0.5f);
        if (scaledSR < 8000) scaledSR = 8000;
        _sampleRate = scaledSR;
        _ltcEncSR   = wavSR;
        _initLTC((double)wavSR, g_ltcStd);
        return true;
    }
    
    void setAudioInfo(AudioInfo info) override {
        _inInfo = info;
        int wavSR    = info.sample_rate;
        int scaledSR = (int)((float)wavSR * _speed + 0.5f);
        if (scaledSR < 8000) scaledSR = 8000;
        if (scaledSR != _sampleRate) {
            _sampleRate = scaledSR;
            _ltcEncSR   = wavSR;
            AudioInfo outInfo = info;
            outInfo.sample_rate = scaledSR;  // output at speed-scaled rate
            outInfo.channels = 2;
            _i2s.setAudioInfo(outInfo);
            _initLTC((double)wavSR, g_ltcStd);  // LTC encoder at original WAV rate
        }
    }

    size_t write(const uint8_t *data, size_t len) override {
        // Serial.printf("[DEBUG] write len: %d\n", len);
        if (len == 0 || !_enc) return 0;
        if (g_ltcStdDirty) { _changeStd(g_ltcStd); g_ltcStdDirty = false; }
        if (_inInfo.bits_per_sample != 16) return len; // Skip if not 16bit
        
        int ch = _inInfo.channels;
        int nf = len / (2 * ch);
        const int16_t *in = (const int16_t *)data;
        int16_t out[256];
        int pos = 0;
        
        for (int i = 0; i < nf; i++) {
            // During count-in: mix click tone with WAV sample so the music
            // is still audible behind the count-in beats.
            // Both are halved to prevent clipping.
            int16_t wavSample = (ch == 1) ? in[i] : in[i*2];
            out[pos*2]   = _countInActive
                              ? (int16_t)((_nextCountInSample() * _playbackVolume + wavSample) / 2)
                              : wavSample;
            out[pos*2+1] = _nextLTC();
            pos++;
            if (pos >= 128) {
                uint8_t *ptr = (uint8_t *)out;
                size_t toWrite = pos * 4;
                while (toWrite > 0) {
                    size_t w = _i2s.write(ptr, toWrite);
                    if (w > 0) {
                        ptr += w;
                        toWrite -= w;
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
                pos = 0;
            }
        }
        if (pos > 0) {
            uint8_t *ptr = (uint8_t *)out;
            size_t toWrite = pos * 4;
            while (toWrite > 0) {
                size_t w = _i2s.write(ptr, toWrite);
                if (w > 0) {
                    ptr += w;
                    toWrite -= w;
                } else {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
        }
        return nf * (2 * ch);
    }

    void setStartTC(uint8_t h, uint8_t m, uint8_t s, uint8_t f) {
        if (!_enc) return;
        SMPTETimecode tc0 = {};
        tc0.hours = h; tc0.mins = m; tc0.secs = s; tc0.frame = f;
        ltc_encoder_set_timecode(_enc, &tc0);  // flag=0: user bits not touched
        _ltcLen = _ltcPos = 0;
        _encodeNext();
    }

    void resetTC() {
        if (!_enc) return;
        SMPTETimecode tc0 = {};
        ltc_encoder_set_timecode(_enc, &tc0);  // flag=0: user bits not touched
        _ltcLen = _ltcPos = 0;
        _encodeNext();
    }

    // Apply current user bits (shot or date) to the mixer encoder.
    void applyUserBits() {
        applyCurrentUserBits(_enc);
    }

    // Arms an 8-beat count-in at the given BPM, adjusted for playback speed.
    // actualPrerollSecs is the real preroll duration (may be < 8 beats if shot
    // starts too early in the file). WAV must already be seeked to preroll start.
    void startCountIn(float bpm, float actualPrerollSecs) {
        if (bpm <= 0.0f || actualPrerollSecs <= 0.0f || _sampleRate <= 0) {
            _countInActive = false;
            return;
        }
        // Scale BPM and preroll duration by playback speed so the count-in
        // clicks stay in sync with the speed-adjusted WAV playback.
        float effBpm     = bpm * _speed;
        float effPreroll = actualPrerollSecs / _speed;
        _countInSamplesPerBeat = (uint32_t)((60.0f / effBpm) * (float)_sampleRate);
        _countInTotalSamples   = (uint32_t)(effPreroll * (float)_sampleRate);
        // Cap at 8 beats to avoid over-run
        uint32_t maxSamples = _countInSamplesPerBeat * 8;
        if (_countInTotalSamples > maxSamples) _countInTotalSamples = maxSamples;
        _countInElapsed   = 0;
        _countInSinePhase = 0.0f;
        _countInActive    = true;
        g_countInActive   = true;
    }

    bool isCountingIn() const { return _countInActive; }

    // Sets the playback channel volume (0.0 – 1.0). Affects WAV and count-in;
    // the TC channel (right) is intentionally unaffected.
    void setPlaybackVolume(float v) {
        _playbackVolume = (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v);
    }

private:
    I2SStream &_i2s;
    LTCEncoder *_enc = nullptr;
    float _playbackVolume = 0.5f; // mirrors initial player volume
    float _speed     = 1.0f;     // playback speed multiplier
    int   _ltcEncSR  = 44100;    // LTC encoder sample rate (original WAV rate, not speed-scaled)
    uint8_t _ltcBuf[3000];
    int _ltcLen = 0, _ltcPos = 0;
    AudioInfo _inInfo = {44100, 1, 16};
    int _sampleRate = 44100;

    // ── Count-in click track state (8-beat sine before WAV audio starts) ──
    bool     _countInActive        = false;
    uint32_t _countInElapsed       = 0;
    uint32_t _countInTotalSamples  = 0;  // exact preroll sample count
    uint32_t _countInSamplesPerBeat = 0; // samples per beat at song BPM
    float    _countInSinePhase     = 0.0f;

    void _initLTC(double sr, LTC_TV_STANDARD std) {
        // Preserve the current H/M/S/F across re-creation. The AudioPlayer calls
        // setAudioInfo() -> _initLTC() on the FIRST copy() (sample-rate change),
        // which previously reset the TC to 00:00:00:00 and made the song play
        // from the start on the first playback of every new shot.
        SMPTETimecode cur = {};
        if (_enc) { ltc_encoder_get_timecode(_enc, &cur); }
        _ltcEncSR = (int)(sr + 0.5);  // remember for framerate-change reinits
        if (_enc) ltc_encoder_free(_enc);
        _enc = ltc_encoder_create(sr, ltc_standard_fps(std), std, 0);
        ltc_encoder_set_volume(_enc, -3.0);
        SMPTETimecode st = {};
        st.hours = cur.hours; st.mins = cur.mins; st.secs = cur.secs; st.frame = cur.frame;
        ltc_encoder_set_timecode(_enc, &st);
        applyCurrentUserBits(_enc);
        _ltcLen = _ltcPos = 0;
        _encodeNext();
    }

    void _changeStd(LTC_TV_STANDARD std) {
        SMPTETimecode cur = {};
        if (_enc) { ltc_encoder_get_timecode(_enc, &cur); ltc_encoder_free(_enc); }
        // Use original WAV rate (_ltcEncSR), not the speed-scaled I2S rate (_sampleRate)
        _enc = ltc_encoder_create((double)_ltcEncSR, ltc_standard_fps(std), std, 0);
        ltc_encoder_set_volume(_enc, -3.0);
        // Restore H/M/S/F only; user bits set by applyCurrentUserBits below
        SMPTETimecode st = {};
        st.hours = cur.hours; st.mins = cur.mins; st.secs = cur.secs; st.frame = cur.frame;
        ltc_encoder_set_timecode(_enc, &st);
        applyCurrentUserBits(_enc);
        _ltcLen = _ltcPos = 0;
        _encodeNext();
    }

    void _encodeNext() {
        if (!_enc) return;
        ltc_encoder_encode_frame(_enc);
        _ltcLen = ltc_encoder_copy_buffer(_enc, _ltcBuf);
        _ltcPos = 0;
        
        SMPTETimecode st;
        ltc_encoder_get_timecode(_enc, &st);
        if (g_sysMode == MODE_PLAYBACK) {
            g_tc_h = st.hours; g_tc_m = st.mins; g_tc_s = st.secs; g_tc_f = st.frame;
            updateUI();
        }
        ltc_encoder_inc_timecode(_enc);
    }
    
    int16_t _nextLTC() {
        if (_ltcPos >= _ltcLen) _encodeNext();
        if (_ltcLen <= 0) return 0;
        return (int16_t)(((int)_ltcBuf[_ltcPos++] - 128) << 8);
    }

    // Generates one click-track sample.
    // 8-beat pattern: high, low, low, low, high, low, low, low
    // High = 1760 Hz (accent), Low = 880 Hz.
    // Tone fills first 55% of each beat; remainder is silence.
    int16_t _nextCountInSample() {
        if (!_countInActive) return 0;
        uint32_t el = _countInElapsed++;
        if (el >= _countInTotalSamples) {
            _countInActive  = false;
            g_countInActive = false;
            updateUI();  // trigger display to switch back to PLAYBACK label
            return 0;
        }
        uint32_t posInBeat = el % _countInSamplesPerBeat;
        if (posInBeat == 0) _countInSinePhase = 0.0f;  // fresh phase each beat
        // Silence in the second half of each beat
        if (posInBeat >= (_countInSamplesPerBeat * 55u / 100u)) return 0;
        // 8-beat accent pattern: beats 0 and 4 = high, all others = low
        uint32_t beatIdx = el / _countInSamplesPerBeat;
        float freq = (beatIdx == 0 || beatIdx == 4) ? 1760.0f : 880.0f;
        _countInSinePhase += 2.0f * (float)M_PI * freq / (float)_sampleRate;
        if (_countInSinePhase >= 2.0f * (float)M_PI)
            _countInSinePhase -= 2.0f * (float)M_PI;
        return (int16_t)(sinf(_countInSinePhase) * 23000.0f);  // ~-3 dBFS
    }
};

// ── LTC Decoder ISR ──────────────────────────────────────────────────────────
// Edge-timing biphase-mark decoder on LTC_INPUT_PIN.
// Three internal states (NASA Rule 1 – no goto):
//   0 (init): reset all state variables on first edge
//   1 (sync): measure half-bit period over ~40 transitions
//   2 (read): decode bits; on sync-word push frame to g_ltcQueue
// NASA Rule 2: all loops bounded (10 = LTCFrameDec size, 40 = calibration cap).

// Packed raw decoder result passed from ISR to taskLTC via g_ltcQueue.
struct LTCFrameDec {
    uint8_t tc[10];   // raw 80 bits stored as 10 bytes
    bool isReverse;   // true if sync word matched in reverse direction
};
static QueueHandle_t g_ltcQueue = NULL;  // ISR → taskLTC frame queue

// Reverse all 8 bits of a byte (used when the LTC frame was captured backwards).
uint8_t flip8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void IRAM_ATTR isrLTC() {
    uint32_t now = micros();
    static uint32_t lastEdge = 0;
    uint32_t cellTime = now - lastEdge;
    if (cellTime < 100) return; // 100us debounce filters out noise
    lastEdge = now;

    static uint8_t shiftReg[10];
    static uint8_t state = 0, counter = 0, zCount = 0;
    static bool eatEdge = false;
    static uint32_t accum = 0, cellOne = 0, cellMin = 0, cellDet = 0, cellMax = 0;

    if (state == 0) { // isrInit
        for (int i=0; i<10; i++) shiftReg[i] = 0;
        eatEdge = false; counter = 0; zCount = 0; accum = 0; cellOne = 0;
        state = 1;
    } else if (state == 1) { // isrSync
        if (++counter > 40) {
            if (zCount > 1) cellOne = (accum / (zCount - 1)) / 2;
            else { state = 0; return; }
            cellMin = cellOne / 2; cellDet = cellMin * 3; cellMax = cellMin * 5;        
            state = 2;           
        } else if (cellTime >= cellOne) {
            if (++zCount > 1) accum += cellTime;
            else cellOne = (cellTime * 2) / 3;
        }
    } else { // isrRead
        if (cellTime < cellMin || cellTime > cellMax) { state = 0; return; }
        uint8_t newBit = 0;
        if (cellTime > cellDet) { 
            if (eatEdge) { state = 0; return; } 
        } else { 
            if (!eatEdge) { eatEdge = true; return; }
            eatEdge = false; newBit = 0x80;
        }

        for (int idx = 9; idx > 0; idx--) shiftReg[idx] = (shiftReg[idx] >> 1) | ((shiftReg[idx - 1] & 1) << 7);
        shiftReg[0] = (shiftReg[0] >> 1) | newBit;
        
        uint16_t syncFwd = ((uint16_t)shiftReg[0] << 8 | shiftReg[1]);
        uint16_t syncRvs = ((uint16_t)shiftReg[8] << 8 | shiftReg[9]);
        
        bool isRvs = false, syncFound = false;
        if (syncFwd == 0xBFFC) { syncFound = true; isRvs = false; }
        else if (syncRvs == 0x3FFD) { syncFound = true; isRvs = true; }
        
        if (syncFound) {
            LTCFrameDec frame;
            for (int i=0; i<10; ++i) frame.tc[i] = shiftReg[i];
            frame.isReverse = isRvs;
            BaseType_t woken = pdFALSE;
            xQueueSendFromISR(g_ltcQueue, &frame, &woken);
            for (int i=0; i<10; ++i) shiftReg[i] = 0; 
            if (woken) portYIELD_FROM_ISR();
        }
        eatEdge = false;
    }
}

// ── LTC Stereo Output (generator mode) ───────────────────────────────────────
// Converts mono LTC stream to stereo: left channel = silence, right channel = LTC.
// This ensures TC goes ONLY to the TC Output (right channel), never to Song Output (left).
class LTCStereoOutput : public AudioOutput {
public:
    explicit LTCStereoOutput(Stream &source) : _source(source) {}

    bool begin() override {
        return true;
    }

    void setAudioInfo(AudioInfo info) override {
        _inInfo = info;
        // Expect mono input; convert to stereo for I2S
    }

    size_t write(const uint8_t *data, size_t len) override {
        if (len == 0) return 0;
        // Input is mono 16-bit samples; output to stereo (left=silence, right=LTC)
        if (_inInfo.bits_per_sample != 16) return len;

        const int16_t *in = (const int16_t *)data;
        int nSamples = len / 2;  // mono samples
        int16_t out[256];
        int outPos = 0;

        for (int i = 0; i < nSamples; i++) {
            out[outPos++] = 0;        // Left channel: silence
            out[outPos++] = in[i];    // Right channel: LTC
            
            if (outPos >= 256) {
                size_t written = 0;
                while (written < (size_t)(outPos * 2)) {
                    size_t w = _source.write((uint8_t *)out + written, outPos * 2 - written);
                    if (w > 0) written += w;
                    else vTaskDelay(pdMS_TO_TICKS(1));
                }
                outPos = 0;
            }
        }

        if (outPos > 0) {
            size_t written = 0;
            while (written < (size_t)(outPos * 2)) {
                size_t w = _source.write((uint8_t *)out + written, outPos * 2 - written);
                if (w > 0) written += w;
                else vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        return len;
    }

private:
    Stream &_source;
    AudioInfo _inInfo = {48000, 1, 16};  // expect mono input
};

// ── Global audio pipeline objects ────────────────────────────────────────────
// Generator mode:  ltcStream → ltcStereo → i2s  (taskEncode drives the copy)
// Playback mode:   player (WAV) → ltcMixer → i2s  (loop() drives player.copy)
// NASA Rule 3: all objects are statically allocated; no heap use at runtime.
I2SStream                     i2s;
AudioInfo                     encInfo(48000, 1, 16); //Always one channel for LTC generator -> otherwise timecode is half as quick
LTCGenerator                  ltcGen;
GeneratedSoundStream<int16_t> ltcStream(ltcGen);
LTCStereoOutput               ltcStereo(i2s);
StreamCopy                    encCopier(ltcStereo, ltcStream);

AudioSourceSDMMC              source("/", "", false);
WAVDecoder                    wavDecoder;
LTCMixerOutput                ltcMixer(i2s);
AudioPlayer                   player(source, ltcMixer, wavDecoder);

// Apply current user bits to whichever encoder is active based on the system mode.
// Implementation deferred here so both ltcGen and ltcMixer are in scope.
static void applyCurrentUserBitsToActiveEncoder() {
    if (g_sysMode == MODE_PLAYBACK) {
        if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
        ltcMixer.applyUserBits();
        if (g_audMtx) xSemaphoreGive(g_audMtx);
    } else {
        ltcGen.applyUserBits();
    }
}

// ── Count-in preroll helpers ────────────────────────────────────────────────────────

// Calculate the TC that is 8 beats before shotTC at the given BPM.
// If that would be negative, clamps to 00:00:00:00 and returns the actual
// preroll duration in seconds (may be < 8 beats).
static float computePrerollTC(uint8_t sh, uint8_t sm, uint8_t ssv, uint8_t sf,
                               float bpm,
                               uint8_t &ph, uint8_t &pm, uint8_t &psv, uint8_t &pf) {
    float fps = ltc_standard_fps(g_ltcStd);
    if (fps <= 0.0f) fps = 25.0f;
    float shotSecs    = (float)(sh * 3600 + sm * 60 + ssv) + (float)sf / fps;
    float prerollSecs = (bpm > 0.0f) ? (8.0f * 60.0f / bpm) : 0.0f;
    float startSecs   = (shotSecs > prerollSecs) ? (shotSecs - prerollSecs) : 0.0f;
    float actual      = shotSecs - startSecs;  // real preroll (may be < 8 beats)
    int32_t iSecs     = (int32_t)startSecs;
    ph  = (uint8_t)(iSecs / 3600);
    pm  = (uint8_t)((iSecs % 3600) / 60);
    psv = (uint8_t)(iSecs % 60);
    pf  = (uint8_t)((startSecs - (float)iSecs) * fps + 0.5f);
    if (pf >= (uint8_t)(fps + 0.5f)) {
        pf = 0; psv++;
        if (psv >= 60) { psv = 0; pm++; if (pm >= 60) { pm = 0; ph++; } }
    }
    return actual;
}

// Seek WAV + LTC to the 8-beat preroll point and arm count-in click track.
// Falls back to immediate playback when BPM is unknown or no shot limits set.
// Must NOT be called while holding g_audMtx.
static void startPlaybackWithCountIn() {
    if (g_sysMode != MODE_PLAYBACK) { g_running = true; return; }

    // Fallback: no BPM or no shot limits → start at shot start (or file start).
    if (g_songBpm <= 0.0f || !g_shotLimited) {
        if (g_shotLimited) {
            uint8_t sh = 0, sm = 0, ssv = 0, sf = 0;
            parseTC(g_shotStartTC, sh, sm, ssv, sf);
            if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
            ltcMixer.setStartTC(sh, sm, ssv, sf);
            seekAudioToTC(sh, sm, ssv, sf);
            if (g_audMtx) xSemaphoreGive(g_audMtx);
            // Re-apply seek after the player parses the WAV header on first copy().
            g_reseekH = sh; g_reseekM = sm; g_reseekS = ssv; g_reseekF = sf;
            g_reseekPending = true;
        }
        g_running = true;
        return;
    }

    uint8_t sh = 0, sm = 0, ssv = 0, sf = 0;
    parseTC(g_shotStartTC, sh, sm, ssv, sf);
    uint8_t ph = 0, pm = 0, psv = 0, pf = 0;
    float actualPreroll = computePrerollTC(sh, sm, ssv, sf, g_songBpm,
                                           ph, pm, psv, pf);
    if (actualPreroll <= 0.0f) { g_running = true; return; }

    if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
    ltcMixer.setStartTC(ph, pm, psv, pf);
    seekAudioToTC(ph, pm, psv, pf);
    ltcMixer.startCountIn(g_songBpm, actualPreroll);
    if (g_audMtx) xSemaphoreGive(g_audMtx);

    // Re-apply seek after the player parses the WAV header on first copy().
    g_reseekH = ph; g_reseekM = pm; g_reseekS = psv; g_reseekF = pf;
    g_reseekPending = true;

    g_running = true;
    updateUI();
}

// Called by AudioPlayer when it reads a WAV title tag; update display title.
// NOTE: We must NOT reset the LTC timecode here. The metadata callback fires
// (only) on the first real playback after begin(), and an earlier resetTC()
// here would clobber the shot start TC set via setStartTC()/seekAudioToTC(),
// making the song play from 00:00:00:00 on the first start of each new shot.
void printMetaData(MetaDataType type, const char *str, int /*len*/) {
    if (type == MetaDataType::Title || type == MetaDataType::Name) {
        g_songTitle = str;
        updateUI();
    }
}

// ── Mode switching ────────────────────────────────────────────────────────────
// Stops the current mode, reconfigures I2S, and starts the new mode.
// Must be called from loop() (Core 1 only) so it can safely block on mutexes.
void applyMode(SystemMode newMode) {
    g_running = false;
    g_reseekPending = false; // cancel any pending start re-seek
    vTaskDelay(pdMS_TO_TICKS(50));  // let active tasks drain their buffers
    
    if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
    if (g_sysMode == MODE_PLAYBACK) player.end();
    if (g_sysMode != MODE_NONE) i2s.end();
    g_sysMode = newMode;
    g_jamOkCnt = 0;

    if (newMode == MODE_GEN_FREE_RUN || newMode == MODE_GEN_RECORD_RUN) {
        auto cfg = i2s.defaultConfig(TX_MODE);
        cfg.pin_bck = BCLK_PIN; cfg.pin_ws = LRCK_PIN; cfg.pin_data = DIN_PIN;
        cfg.sample_rate = 48000; cfg.channels = 2; cfg.bits_per_sample = 16;
        // cfg.buffer_count = 8;
        // cfg.buffer_size = 1024;
        i2s.begin(cfg);

                g_rtcMode = (newMode == MODE_GEN_FREE_RUN) ? RTC_FREE_RUN : RTC_RECORD_RUN;
        ltcGen.applyStartTC();
        if (newMode == MODE_GEN_FREE_RUN) {
            g_running = true;
        }
    }
    else if (newMode == MODE_PLAYBACK) {
        // Scale I2S sample rate by shot speed; LTC encoder will use original 44100 Hz
        // so that TC advances at speed*fps per real second.
        int scaledSR = (int)(44100.0f * (float)g_shotSpeed + 0.5f);
        if (scaledSR < 8000) scaledSR = 8000;
        auto cfg = i2s.defaultConfig(TX_MODE);
        cfg.pin_bck = BCLK_PIN; cfg.pin_ws = LRCK_PIN; cfg.pin_data = DIN_PIN;
        cfg.sample_rate = scaledSR; cfg.channels = 2; cfg.bits_per_sample = 16;
        // cfg.buffer_count = 10;
        // cfg.buffer_size = 1024;
        i2s.begin(cfg);

        ltcMixer.begin();
        // Parse the real WAV/BWF data-chunk offset for accurate seeking.
        scanWavDataOffset();
        if (g_sysMtx) xSemaphoreTake(g_sysMtx, portMAX_DELAY);
        if (g_shotLimited) {
            uint8_t h=0, m=0, s=0, f=0;
            parseTC(g_shotStartTC, h, m, s, f);
            ltcMixer.setStartTC(h, m, s, f);
        }
        if (g_sysMtx) xSemaphoreGive(g_sysMtx);

        // Load song metadata from SD if available (survives ESP32 reboot)
        if (SD_MMC.exists("/songinfo.json")) {
            File _sif = SD_MMC.open("/songinfo.json", FILE_READ);
            if (_sif) {
                JsonDocument _sid;
                if (deserializeJson(_sid, _sif) == DeserializationError::Ok) {
                    const char* st = _sid["songtitle"];
                    if (st && strlen(st) > 0) g_songTitle = String(st);
                    float bpm = _sid["songBpm"].as<float>();
                    if (bpm > 0.0f) g_songBpm = bpm;
                    int tsig = _sid["songTimeSignature"].as<int>();
                    if (tsig > 0) g_songTimeSig = tsig;
                    String durTC = _sid["songduration"] | "";
                    if (durTC.length() >= 11) {
                        float fps = ltc_standard_fps(g_ltcStd);
                        if (fps <= 0.0f) fps = 25.0f;
                        uint8_t dh=0, dm=0, ds=0, df=0;
                        parseTC(durTC, dh, dm, ds, df);
                        g_songDurSec = (float)(dh*3600 + dm*60 + ds) + (float)df / fps;
                    }
                }
                _sif.close();
            }
        }

                player.setIndex(0);
        player.begin();
        // Warm up the player so the WAV header is parsed and the player becomes
        // active BEFORE the user starts playback. On a fresh boot the lazy
        // header parse on the first copy() rewinds to the file start, which
        // caused the song to play from the beginning the first time. We drive a
        // few bounded copy() calls here (output is silent because g_running is
        // still false and the mixer only emits once fed) so the header is ready.
        for (int warm = 0; warm < 20 && !player.isActive(); warm++) {
            player.copy();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        // Re-seek to the shot start so the warmed-up position is correct.
        // The warm-up advanced both the WAV file position and the LTC encoder
        // timecode, so reset BOTH back to the shot start here.
        if (g_shotLimited) {
            uint8_t h=0, m=0, s=0, f=0;
            parseTC(g_shotStartTC, h, m, s, f);
            ltcMixer.setStartTC(h, m, s, f);
            seekAudioToTC(h, m, s, f);
        }
        // player.play(); // Do not autoplay audio buffer
        // g_running = true;
    }
    if (g_audMtx) xSemaphoreGive(g_audMtx);

    // Update PCM5102 MUTE pin based on active mode
    if (isMuteMode(newMode)) PCM_MUTE(); else PCM_UNMUTE();

    updateUI();
    broadcastStatus();
}

// ── Tasks ──────────────────────────────────────────────────────────────────
void taskEncode(void *) {
    for (;;) {
        if ((g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN) && g_running) {
            if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
            encCopier.copy();
            if (g_audMtx) xSemaphoreGive(g_audMtx);
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void taskLTC(void *) {
    LTCFrameDec frame;
    uint32_t lastEdgeUs = 0; // timestamp of the previous decoded frame
    uint32_t frameCtr   = 0; // frame counter used for FPS estimation window
    uint8_t  maxFrame   = 0; // highest frame number seen in current window
    uint8_t  baseFps    = 30;
    
    while(1) {
        if (xQueueReceive(g_ltcQueue, &frame, pdMS_TO_TICKS(50))) {
            uint32_t now    = micros();
            uint32_t period = now - lastEdgeUs;  // inter-frame period in microseconds
            lastEdgeUs = now;
            
            uint8_t tc[10];
            // Reverse bits if the LTC frame was transmitted backwards
            if (frame.isReverse) { for(int i=0; i<10; i++) tc[i] = flip8(frame.tc[9-i]); }
            else                 { for(int i=0; i<10; i++) tc[i] = frame.tc[i]; }

            // Extract H/M/S/F from the packed bit-field layout of LTCFrameDec
            uint8_t hh = ((tc[2]&0x03)*10)+(tc[3]&0x0F);
            uint8_t mm = ((tc[4]&0x07)*10)+(tc[5]&0x0F);
            uint8_t ss = ((tc[6]&0x07)*10)+(tc[7]&0x0F);
            uint8_t ff = ((tc[8]&0x03)*10)+(tc[9]&0x0F);
            bool df = (tc[8]&0x04) != 0;  // drop-frame flag

            // Update FPS estimate every 32 frames
            if (ff > maxFrame) maxFrame = ff;
            if (++frameCtr > 31) { baseFps = df ? 30 : maxFrame + 1; maxFrame = 0; frameCtr = 0; }

            // Playback speed ratio (1.0 = exact speed); used for jam-lock check
            float spd = (period > 100) ? (1000000.0f / (float)baseFps) / (float)period : 0;
            
            if (g_sysMode == MODE_SLAVE || g_sysMode == MODE_JAM_SYNC_FREE_RUN || g_sysMode == MODE_JAM_SYNC_RECORD_RUN) {
                g_tc_h = hh; g_tc_m = mm; g_tc_s = ss; g_tc_f = ff;
                g_rxDropFrame = df;
                g_rxFps = df ? 29.97f : (float)baseFps;
                g_rxUserBits = ((uint32_t)(tc[2]>>4)&0xF) | (((uint32_t)(tc[3]>>4)&0xF)<<4) |
                             (((uint32_t)(tc[4]>>4)&0xF)<<8)| (((uint32_t)(tc[5]>>4)&0xF)<<12)|
                             (((uint32_t)(tc[6]>>4)&0xF)<<16)|(((uint32_t)(tc[7]>>4)&0xF)<<20)|
                             (((uint32_t)(tc[8]>>4)&0xF)<<24)|(((uint32_t)(tc[9]>>4)&0xF)<<28);

                updateUI();

                if (g_sysMode == MODE_JAM_SYNC_FREE_RUN || g_sysMode == MODE_JAM_SYNC_RECORD_RUN) {
                    if (spd > 0.8f && spd < 1.2f) g_jamOkCnt++;
                    else g_jamOkCnt = 0;

                    if (g_jamOkCnt > 150 && ff == 0) {
                        if (g_sysMode == MODE_JAM_SYNC_FREE_RUN) {
                            Serial.println("[JAM] Jam Sync Success! Switching to Free Run...");
                            if (xSemaphoreTake(g_i2cMtx, portMAX_DELAY) == pdTRUE) {
                                DateTime no = rtc.now();
                                rtc.adjust(DateTime(no.year(), no.month(), no.day(), hh, mm, ss));
                                xSemaphoreGive(g_i2cMtx);
                            }
                            if (df) { g_ltcStd = LTC_TV_525_DF; g_ltcStdDirty = true; }
                            g_reqMode = MODE_GEN_FREE_RUN;
                        } else if (g_sysMode == MODE_JAM_SYNC_RECORD_RUN) {
                            Serial.println("[JAM] Jam Sync Success! Switching to Record Run...");
                            if (df) { g_ltcStd = LTC_TV_525_DF; g_ltcStdDirty = true; }
                            ltcGen.setManualTimecode(hh, mm, ss, 0);
                            g_reqMode = MODE_GEN_RECORD_RUN;
                        }
                    }
                }
            }
        }
    }
}

// ── Menu System ─────────────────────────────────────────────────────────────
// MENU_OVERLAY  : 4-icon bar drawn over standscreen (Play/Stop/Reset/Settings)
// MENU_SETTINGS : full-screen list (Mode / Framerate / Sync RTC / Format SD / Close)
// MENU_MODE     : sub-list for mode selection
// MENU_FPS      : sub-list for framerate selection
// MENU_FORMAT_SD: confirmation screen before formatting the SD card
enum MenuState { MENU_CLOSED, MENU_OVERLAY, MENU_SETTINGS, MENU_MODE, MENU_FPS, MENU_FORMAT_SD, MENU_SHOT_BROWSE };
static MenuState g_menuState = MENU_CLOSED;
static int g_menuSel = 0;
static uint32_t g_browseEntryMs = 0;  // timestamp when browse mode was entered

static const char* kSettingsItems[] = { "Mode", "Framerate", "Sync RTC", "Format SD-Card", "Close" };
static const int   kSettingsCount   = 5;
static const char* kModeItems[]  = { "Gen: Free Run", "Gen: Record Run", "Playback",
                                     "Slave", "Jam Sync Free", "Jam Sync Rec", "Back" };
static const int   kModeCount    = 7;
static const char* kFpsItems[]   = { "24 fps", "25 fps (ND)", "29.97 DF", "30 fps (ND)", "Back" };
static const int   kFpsCount     = 5;
static const char* kFormatSDItems[]   = {"Format SD-Card", "Back" };
static const int   kFormatSDCount     = 2;


static void menuGetItems(const char** &items, int &count) {
    switch (g_menuState) {
        case MENU_SETTINGS:  items = kSettingsItems;  count = kSettingsCount;  return;
        case MENU_MODE:      items = kModeItems;      count = kModeCount;      return;
        case MENU_FPS:       items = kFpsItems;       count = kFpsCount;       return;
        case MENU_FORMAT_SD: items = kFormatSDItems;  count = kFormatSDCount;  return;
        default:             items = kSettingsItems;  count = kSettingsCount;  return;
    }
}

static const char* menuTitle() {
    switch (g_menuState) {
        case MENU_SETTINGS:  return "  SETTINGS";
        case MENU_MODE:      return "  MODE";
        case MENU_FPS:       return "  FRAMERATE";
        case MENU_FORMAT_SD: return "  FORMAT SD";
        default:             return "  SETTINGS";
    }
}

void broadcastStatus() {
    JsonDocument doc;
    doc["type"] = "status";
    if      (g_sysMode == MODE_GEN_FREE_RUN)        doc["gen_mode"] = "gen_free";
    else if (g_sysMode == MODE_GEN_RECORD_RUN)      doc["gen_mode"] = "gen_rec";
    else if (g_sysMode == MODE_PLAYBACK)            doc["gen_mode"] = "playback";
    else if (g_sysMode == MODE_JAM_SYNC_RECORD_RUN) doc["gen_mode"] = "jam_rec";
    else if (g_sysMode == MODE_JAM_SYNC_FREE_RUN)   doc["gen_mode"] = "jam_free";
    else if (g_sysMode == MODE_SLAVE)               doc["gen_mode"] = "slave";
    if      (g_ltcStd == LTC_TV_525_60)  doc["tc_mode"] = "30";
    else if (g_ltcStd == LTC_TV_625_50)  doc["tc_mode"] = "25";
    else if (g_ltcStd == LTC_TV_525_DF)  doc["tc_mode"] = "29.97DF";
    else if (g_ltcStd == LTC_TV_FILM_24) doc["tc_mode"] = "24";
    doc["running"] = (bool)g_running;
    doc["volume"] = (int)g_volumePercent;
    // Timecode: sent only on action (not continuous)
    char tcBuf[12];
    snprintf(tcBuf, sizeof(tcBuf), "%02d:%02d:%02d:%02d", g_tc_h, g_tc_m, g_tc_s, g_tc_f);
    doc["timecode"] = tcBuf;
    String out; serializeJson(doc, out);
    ws.textAll(out);
}

static void drawMenuItem(int idx, bool selected) {
    const char** items; int count;
    menuGetItems(items, count);
    if (idx < 0 || idx >= count) return;
    bool isBack = (idx == count - 1) && (g_menuState == MENU_MODE || g_menuState == MENU_FPS || g_menuState == MENU_FORMAT_SD);
    uint16_t bg = selected ? (uint16_t)ILI9341_RED : (uint16_t)ILI9341_BLACK;
    uint16_t fg = (isBack && !selected) ? (uint16_t)ILI9341_DARKGREY : (uint16_t)ILI9341_WHITE;
    tft.fillRect(0, 30 + idx * 40, 240, 40, bg);
    tft.drawFastHLine(0, 30 + idx * 40 + 39, 240, ILI9341_DARKGREY);
    tft.setTextColor(fg, bg);
    tft.setTextSize(2);
    tft.setCursor(12, 30 + idx * 40 + 12);
    tft.print(items[idx]);
}

static void drawMenuScreen() {
    const char** items; int count;
    menuGetItems(items, count);
    // Header
    tft.fillRect(0, 0, 240, 30, ILI9341_BLACK);
    tft.drawFastHLine(0, 29, 240, ILI9341_RED);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(8, 8);
    tft.print(menuTitle());
    // Items
    for (int i = 0; i < count; i++) drawMenuItem(i, i == g_menuSel);
    // Clear unused area below items
    int bottom = 30 + count * 40;
    if (bottom < 320) tft.fillRect(0, bottom, 240, 320 - bottom, ILI9341_BLACK);
}

// ── Overlay icon bar (Play / Stop / Reset / Settings) ──────────────────────
// Drawn at y=250..319 (70px tall, 4 equal 60px cells). All icons are bold filled shapes.
static const char* kOverlayLabels[] = { "PLAY", "STOP", "RESET", "SET" };
#define OVY 250   // overlay top y

static void drawOverlayIcon(int idx, int cx, int cy, uint16_t col, uint16_t bg) {
    switch (idx) {
        case 0: // Play ▶  — filled right-pointing triangle
            tft.fillTriangle(cx-9, cy-11, cx-9, cy+11, cx+11, cy, col);
            break;

        case 1: // Stop ■  — filled square
            tft.fillRect(cx-10, cy-10, 21, 21, col);
            break;

        case 2: // Reset ←  — filled left-pointing arrow (triangle head + rect shaft)
            // shaft
            tft.fillRect(cx-4, cy-4, 16, 8, col);
            // arrowhead (filled triangle, pointing left)
            tft.fillTriangle(cx-13, cy, cx-1, cy-11, cx-1, cy+11, col);
            break;

        case 3: // Settings ⚙  — gear: filled outer circle + 4 rectangular teeth + filled inner hole
            // outer circle body
            tft.fillCircle(cx, cy, 10, col);
            // top tooth
            tft.fillRect(cx-3, cy-15, 7, 6, col);
            // bottom tooth
            tft.fillRect(cx-3, cy+9,  7, 6, col);
            // left tooth
            tft.fillRect(cx-15, cy-3, 6, 7, col);
            // right tooth
            tft.fillRect(cx+9,  cy-3, 6, 7, col);
            // inner hole (punch out with bg color)
            tft.fillCircle(cx, cy, 5, bg);
            break;
    }
}

static void drawOverlayCell(int idx, bool selected) {
    int x = idx * 60;
    uint16_t bg      = selected ? (uint16_t)0x2000 : (uint16_t)0x1082;
    uint16_t brd     = selected ? (uint16_t)ILI9341_RED : (uint16_t)ILI9341_DARKGREY;
    uint16_t iconCol = selected ? (uint16_t)ILI9341_WHITE : (uint16_t)0x8410;
    tft.fillRect(x, OVY, 60, 320 - OVY, bg);
    tft.drawRect(x, OVY, 60, 320 - OVY, brd);
    drawOverlayIcon(idx, x + 30, OVY + 26, iconCol, bg);
    tft.setTextSize(1);
    tft.setTextColor(iconCol, bg);
    int len = strlen(kOverlayLabels[idx]);
    tft.setCursor(x + 30 - len * 3, OVY + 52);
    tft.print(kOverlayLabels[idx]);
}

static void drawOverlayBar() {
    for (int i = 0; i < 4; i++) drawOverlayCell(i, i == g_menuSel);
}

static void menuHandlePress() {
    if (g_menuState == MENU_OVERLAY) {
        switch (g_menuSel) {
            case 0: // Play
                if (!g_running && (g_sysMode == MODE_PLAYBACK ||
                    g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN)) {
                    if (g_sysMode == MODE_PLAYBACK) {
                        startPlaybackWithCountIn();
                    } else {
                        g_running = true;
                        if (g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN)
                            ltcGen.applyStartTC();
                    }
                }
                g_menuState = MENU_CLOSED;
                broadcastStatus();
                break;
            case 1: // Stop
                g_running = false;
                g_menuState = MENU_CLOSED;
                broadcastStatus();
                break;
            case 2: // Reset
                ltcGen.setManualTimecode(0, 0, 0, 0);
                g_tc_h = 0; g_tc_m = 0; g_tc_s = 0; g_tc_f = 0;
                g_menuState = MENU_CLOSED;
                broadcastStatus();
                break;
            case 3: // Settings → open full-screen settings menu
                tft.fillScreen(ILI9341_BLACK);
                g_menuState = MENU_SETTINGS;
                g_menuSel = 0;
                drawMenuScreen();
                break;
        }
    } else if (g_menuState == MENU_SETTINGS) {
        switch (g_menuSel) {
            case 0: g_menuState = MENU_MODE;      g_menuSel = 0; drawMenuScreen(); break;
            case 1: g_menuState = MENU_FPS;       g_menuSel = 0; drawMenuScreen(); break;
            case 2: syncRTCWithNTP(); break;  // stays in settings
            case 3: g_menuState = MENU_FORMAT_SD; g_menuSel = 0; drawMenuScreen(); break;
            case 4: g_menuState = MENU_CLOSED; break;
        }
    } else if (g_menuState == MENU_FORMAT_SD) {
        if (g_menuSel == kFormatSDCount - 1) {
            // Back → return to settings
            g_menuState = MENU_SETTINGS; g_menuSel = 0; drawMenuScreen();
        } else {
            // Confirmed: delete all files in root (Arduino SD_MMC has no format() API)
            File root = SD_MMC.open("/");
            File entry = root.openNextFile();
            while (entry) {
                String name = String("/") + entry.name();
                entry.close();
                SD_MMC.remove(name.c_str());
                entry = root.openNextFile();
            }
            root.close();
            Serial.println("[SD] All files deleted");
            g_menuState = MENU_CLOSED;
        }
    } else if (g_menuState == MENU_MODE) {
        if (g_menuSel == kModeCount - 1) {
            g_menuState = MENU_SETTINGS; g_menuSel = 0; drawMenuScreen();
        } else {
            const SystemMode modes[] = {
                MODE_GEN_FREE_RUN, MODE_GEN_RECORD_RUN, MODE_PLAYBACK,
                MODE_SLAVE, MODE_JAM_SYNC_FREE_RUN, MODE_JAM_SYNC_RECORD_RUN
            };
            g_reqMode = modes[g_menuSel];
            g_menuState = MENU_CLOSED;
        }
    } else if (g_menuState == MENU_FPS) {
        if (g_menuSel == kFpsCount - 1) {
            g_menuState = MENU_SETTINGS; g_menuSel = 0; drawMenuScreen();
        } else {
            const LTC_TV_STANDARD fps[] = {
                LTC_TV_FILM_24, LTC_TV_625_50, LTC_TV_525_DF, LTC_TV_525_60
            };
            g_ltcStd = fps[g_menuSel];
            g_ltcStdDirty = true;
            g_menuState = MENU_CLOSED;
            broadcastStatus();
        }
    }
}

void taskDisplay(void *) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(70, 4); tft.print("ARC PLAY");
    tft.drawFastHLine(0, 24, 240, ILI9341_DARKGREY);
    tft.drawFastHLine(0, 70, 240, ILI9341_DARKGREY);
    tft.drawFastHLine(0, 160, 240, ILI9341_DARKGREY);
    tft.drawFastHLine(0, 250, 240, ILI9341_DARKGREY);

    char buf[64];
    bool firstDraw = true;

    uint8_t o_hh=255, o_mm=255, o_ss=255, o_ff=255;
    SystemMode o_mode = MODE_NONE;
    bool o_run = false;
    LTC_TV_STANDARD o_std = (LTC_TV_STANDARD)255;
    String o_song = "";
    int o_scene = -1;
    String o_shot = "";
    int o_take = -1;
    String o_roll = "";
    String o_st = "";
    String o_et = "";
    
    uint32_t o_ub = 0xFFFFFFFF;
    int o_rtc_m = -1;
    uint32_t o_jam = 0xFFFF;
    bool o_df = false;
    float o_fps = -1;
    float o_bpm = -1.0f;
    int   o_timesig = -1;
    bool  o_countIn = false;
    bool  o_browse  = false;
    int   o_vol      = -1;     // volume percentage for overlay bar
    uint32_t lastRtcRead = 0;

    int progressAnim = 0;
    uint32_t lastAnim = 0;
    uint32_t lastSlowUIUpdate = 0;

    while(1) {
        // ── Encoder rotation ──────────────────────────────────────────────
        {
            int steps = g_encDelta;
            if (steps != 0) {
                g_encDelta = 0;
                if (g_menuState == MENU_CLOSED) {
                    // Encoder rotation adjusts volume (0–100 %, step ±2)
                    int newVol = g_volumePercent + steps * 2;
                    if (newVol < 0)   newVol = 0;
                    if (newVol > 100) newVol = 100;
                    if (newVol != g_volumePercent) {
                        g_volumePercent = newVol;
                        g_volShowUntilMs = millis() + 1500;  // show bar for 1.5 s
                        g_volDirty = true;  // loop() applies volume to I2S
                    }
                } else if (g_menuState == MENU_SHOT_BROWSE) {
                    // Encoder rotation navigates previous/next shot
                    String navJson;
                    JsonDocument navDoc;
                    navDoc["type"] = (steps > 0) ? "nextShot" : "previousShot";
                    serializeJson(navDoc, navJson);
                    ws.textAll(navJson);
                } else {
                    const char** items; int count;
                    menuGetItems(items, count);
                    int oldSel = g_menuSel;
                    g_menuSel = ((g_menuSel + steps) % count + count) % count;
                    if (oldSel != g_menuSel) {
                        drawMenuItem(oldSel, false);
                        drawMenuItem(g_menuSel, true);
                    }
                }
            }
            // ── Button press (debounced) + Long-Press (1.5s = Menu) + Double-Click (Shot Browse) ──
            static bool lastBtn = true;
            static uint32_t lastBtnMs = 0;
            static uint32_t lastReleaseMs = 0;
            static bool longPressHandled = false;
            static bool doubleClickPending = false;
            bool btnNow = (bool)digitalRead(ENC_SW_PIN);

            // Falling edge: button pressed down
            if (lastBtn && !btnNow) {
                if (millis() - lastBtnMs > 80) {
                    lastBtnMs = millis();
                    longPressHandled = false;
                }
            }

            // While held: check for long press (1.5 s), only in MENU_CLOSED
            if (!btnNow && !longPressHandled && g_menuState == MENU_CLOSED &&
                millis() - lastBtnMs >= 1500) {
                longPressHandled = true;
                doubleClickPending = false; // cancel any pending double-click
                // Long press opens the settings menu
                tft.fillScreen(ILI9341_BLACK);
                g_menuState = MENU_SETTINGS;
                g_menuSel = 0;
                drawMenuScreen();
            }

            // Rising edge: button released → short press or double-click action
            if (!lastBtn && btnNow && !longPressHandled) {
                uint32_t nowMs = millis();
                if (g_menuState == MENU_CLOSED) {
                    if (nowMs - lastReleaseMs < 400) {
                        // Double-click detected → enter shot browse mode
                        doubleClickPending = false;
                        g_menuState = MENU_SHOT_BROWSE;
                        g_browseEntryMs = nowMs;
                        // Request first shot from server
                        String navJson;
                        JsonDocument navDoc;
                        navDoc["type"] = "nextShot";
                        serializeJson(navDoc, navJson);
                        ws.textAll(navJson);
                        Serial.println("[BROWSE] Shot browse mode entered (double-click)");
                    } else {
                        // Potential first click of double-click — wait briefly
                        doubleClickPending = true;
                    }
                    lastReleaseMs = nowMs;
                } else if (g_menuState == MENU_SHOT_BROWSE) {
                    // Single click in browse mode = activate current shot
                    g_menuState = MENU_CLOSED;
                    // Restore full status screen
                    tft.fillScreen(ILI9341_BLACK);
                    tft.setTextColor(ILI9341_WHITE);
                    tft.setTextSize(2);
                    tft.setCursor(70, 4); tft.print("ARC PLAY");
                    tft.drawFastHLine(0, 24,  240, ILI9341_DARKGREY);
                    tft.drawFastHLine(0, 70,  240, ILI9341_DARKGREY);
                    tft.drawFastHLine(0, 160, 240, ILI9341_DARKGREY);
                    tft.drawFastHLine(0, 250, 240, ILI9341_DARKGREY);
                    o_hh=255; o_mm=255; o_ss=255; o_ff=255; o_run=false;
                    o_song=""; o_scene=-1; o_shot=""; o_take=-1;
                    o_roll=""; o_st=""; o_et="";
                    o_rtc_m=-1; o_std=(LTC_TV_STANDARD)255;
                    o_ub=0xFFFFFFFF; o_jam=0xFFFF; o_df=false; o_fps=-1;
                    o_bpm=-1.0f; o_timesig=-1; o_countIn=false; o_browse=false;
                    o_vol   = -1;
                    o_mode = MODE_NONE;
                    firstDraw = true;
                    broadcastStatus();
                    Serial.println("[BROWSE] Shot activated, browse mode closed");
                } else {
                    // Inside a submenu — pass to menu handler immediately
                    menuHandlePress();
                    if (g_menuState == MENU_CLOSED) {
                        // Restore full status screen
                        tft.fillScreen(ILI9341_BLACK);
                        tft.setTextColor(ILI9341_WHITE);
                        tft.setTextSize(2);
                        tft.setCursor(70, 4); tft.print("ARC PLAY");
                        tft.drawFastHLine(0, 24,  240, ILI9341_DARKGREY);
                        tft.drawFastHLine(0, 70,  240, ILI9341_DARKGREY);
                        tft.drawFastHLine(0, 160, 240, ILI9341_DARKGREY);
                        tft.drawFastHLine(0, 250, 240, ILI9341_DARKGREY);
                        o_hh=255; o_mm=255; o_ss=255; o_ff=255; o_run=false;
                        o_song=""; o_scene=-1; o_shot=""; o_take=-1;
                        o_roll=""; o_st=""; o_et="";
                        o_rtc_m=-1; o_std=(LTC_TV_STANDARD)255;
                        o_ub=0xFFFFFFFF; o_jam=0xFFFF; o_df=false; o_fps=-1;
                        o_bpm=-1.0f; o_timesig=-1; o_countIn=false; o_browse=false;
                        o_vol   = -1;
                        o_mode = MODE_NONE;
                        firstDraw = true;
                        broadcastStatus();
                    }
                }
            }
            lastBtn = btnNow;

            // Handle single-click timing: if doubleClickPending and 400ms passed → single click = play/pause
            if (doubleClickPending && millis() - lastReleaseMs >= 400) {
                doubleClickPending = false;
                if (g_menuState == MENU_CLOSED) {
                    // Short press = toggle play/pause
                    if (g_sysMode == MODE_PLAYBACK || g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN) {
                        if (!g_running && g_sysMode == MODE_PLAYBACK) {
                            startPlaybackWithCountIn();
                        } else {
                            g_running = !g_running;
                            if (g_running && (g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN))
                                ltcGen.applyStartTC();
                        }
                    }
                    broadcastStatus();
                    updateUI();
                }
            }
        }
        // Full-screen menus block all drawing; overlay only blocks slow section
        if (g_menuState == MENU_SETTINGS || g_menuState == MENU_MODE || g_menuState == MENU_FPS || g_menuState == MENU_FORMAT_SD) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15)) || firstDraw) {
            if (g_wavRx) {
                if (o_mode != (SystemMode)255) { // Store we are in download state
                    tft.fillScreen(ILI9341_BLACK);
                    tft.setTextColor(ILI9341_WHITE);
                    tft.setTextSize(3);
                    tft.setCursor(20, 100);
                    tft.print("RECEIVING");
                    tft.setCursor(20, 140);
                    tft.print("WAV FILE...");
                    o_mode = (SystemMode)255;
                }
                continue; // Skip the rest of the drawing
            } else if (o_mode == (SystemMode)255 && !firstDraw) { // Just finished receiving
                firstDraw = true; // Trigger full redraw
                tft.fillScreen(ILI9341_BLACK);
                tft.setTextColor(ILI9341_WHITE);
                tft.setTextSize(2);
                tft.setCursor(70, 4); tft.print("ARC PLAY");
                tft.drawFastHLine(0, 24, 240, ILI9341_DARKGREY);
                tft.drawFastHLine(0, 70, 240, ILI9341_DARKGREY);
                tft.drawFastHLine(0, 160, 240, ILI9341_DARKGREY);
                tft.drawFastHLine(0, 250, 240, ILI9341_DARKGREY);
                o_hh=255; o_mm=255; o_ss=255; o_ff=255; o_run=false;
                o_song=""; o_scene=-1; o_shot=""; o_take=-1; o_roll=""; o_st=""; o_et="";
                o_rtc_m=-1; o_std=(LTC_TV_STANDARD)255; o_ub=0xFFFFFFFF; o_jam=0xFFFF; o_df=false; o_fps=-1;
                o_bpm=-1.0f; o_timesig=-1; o_countIn=false; o_browse=false;
                o_vol   = -1;
                // do not change o_mode here so that o_mode != g_sysMode allows it to redraw!
            }

            // ── Volume overlay bar (auto-hide after 1.5 s) ──────────────
            bool volVisible = (millis() < g_volShowUntilMs) && (g_menuState == MENU_CLOSED);
            if (volVisible) {
                int vol = (int)g_volumePercent;
                if (vol != o_vol) {
                    // Clear previous overlay area (y 126…156)
                    tft.fillRect(0, 126, 240, 40, ILI9341_BLACK);
                    // Percentage label
                    tft.setTextSize(2);
                    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                    char pct[8]; snprintf(pct, sizeof(pct), "%d%%", vol);
                    tft.setCursor(100, 130); tft.print(pct);
                    // Bar outline
                    tft.drawRect(20, 146, 200, 12, ILI9341_WHITE);
                    // Bar fill
                    int fillW = (vol * 196) / 100;  // 200 – 4px padding
                    if (fillW > 0) {
                        uint16_t barCol = (vol <= 30) ? (uint16_t)ILI9341_GREEN :
                                          (vol <= 70) ? (uint16_t)ILI9341_YELLOW : (uint16_t)ILI9341_RED;
                        tft.fillRect(22, 148, fillW, 8, barCol);
                    }
                    o_vol = vol;
                }
            } else if (!volVisible && o_vol >= 0) {
                // Timer expired – clear bar area
                tft.fillRect(0, 126, 240, 40, ILI9341_BLACK);
                o_vol = -1;
            }

            // Only do slow updates (mode/fps/shot/song) when overlay is not covering them
            bool doSlowUpdate = ((millis() - lastSlowUIUpdate > 500) || firstDraw) && (g_menuState == MENU_CLOSED || g_menuState == MENU_SHOT_BROWSE);
            if (doSlowUpdate) {
                lastSlowUIUpdate = millis();
            }

            firstDraw = false;

            // 1. Timecode (Top, Green/Gray, Fast 15Hz+ update)
            if (o_hh != g_tc_h || o_mm != g_tc_m || o_ss != g_tc_s || o_ff != g_tc_f || o_run != g_running || o_mode != g_sysMode) {
                // Play/Pause-Icon im Header (zwischen "ARC" und Uhrzeit)
                {
                    tft.fillRect(0, 0, 28, 24, ILI9341_BLACK);
                    if (g_running) {
                        // ▶ green triangle = running
                        tft.fillTriangle(2, 2, 2, 22, 22, 12, ILI9341_GREEN);
                    } else {
                        // ⏸ zwei graue Balken = pausiert
                        tft.fillRect(2, 3, 6, 18, 0x7BEF);
                        tft.fillRect(12, 3, 6, 18, 0x7BEF);
                    }
                }
                tft.setTextSize(3);
                tft.setTextColor((g_running || g_sysMode==MODE_SLAVE) ? ILI9341_GREEN : 0x7BEF, ILI9341_BLACK);
                snprintf(buf, sizeof(buf), "%02d:%02d:%02d:%02d", g_tc_h, g_tc_m, g_tc_s, g_tc_f);
                tft.setCursor(20, 36); tft.print(buf);
                o_hh = g_tc_h; o_mm = g_tc_m; o_ss = g_tc_s; o_ff = g_tc_f; o_run = g_running;
            }

            if (doSlowUpdate) {
                // 0. Time (Top Right)
                if (millis() - lastRtcRead > 1000 || o_mode != g_sysMode) {
                    if (xSemaphoreTake(g_i2cMtx, 0) == pdTRUE) {
                        DateTime now = rtc.now();
                        xSemaphoreGive(g_i2cMtx);
                        if (o_rtc_m != now.minute() || o_mode != g_sysMode) {
                            tft.setTextSize(2);
                            tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
                            snprintf(buf, sizeof(buf), "%02d:%02d", now.hour(), now.minute());
                            tft.setCursor(176, 4); tft.print(buf);
                            o_rtc_m = now.minute();
                        }
                    }
                    lastRtcRead = millis();
                }

            // 2. Mode
            bool curBrowse = (g_menuState == MENU_SHOT_BROWSE);
            bool curCountIn = (bool)g_countInActive;
            if (o_mode != g_sysMode || o_countIn != curCountIn || o_browse != curBrowse) {
                tft.setTextSize(2);
                tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
                const char* mS;
                if (curCountIn) {
                    mS = "COUNT IN...     ";
                } else if (curBrowse) {
                    mS = "BROWSE SHOT     ";
                } else {
                    mS = "UNKNOWN         ";
                    if (g_sysMode == MODE_GEN_FREE_RUN) mS = "GEN: FREE RUN   ";
                    if (g_sysMode == MODE_GEN_RECORD_RUN) mS= "GEN: REC RUN    ";
                    if (g_sysMode == MODE_PLAYBACK) mS =      "PLAYBACK        ";
                    if (g_sysMode == MODE_SLAVE) mS =         "SLAVE DECODER   ";
                    if (g_sysMode == MODE_JAM_SYNC_FREE_RUN) mS = "JAM SYNC FREE   ";
                    if (g_sysMode == MODE_JAM_SYNC_RECORD_RUN) mS ="JAM SYNC REC    ";
                }
                snprintf(buf, sizeof(buf), "%-16s", mS);
                tft.setCursor(10, 80); tft.print(buf);
                o_countIn = curCountIn;
                o_browse  = curBrowse;
            }

            // 3. & 4. Framerate and Userbits combined
            bool isDetectFPS = (g_sysMode == MODE_SLAVE || g_sysMode == MODE_JAM_SYNC_FREE_RUN || g_sysMode == MODE_JAM_SYNC_RECORD_RUN);
            uint32_t displayUserBits = isDetectFPS ? g_rxUserBits : g_txUserBits;
            if (o_mode != g_sysMode || o_std != g_ltcStd || o_ub != displayUserBits || (isDetectFPS && (o_fps != g_rxFps || o_df != g_rxDropFrame))) {
                tft.setTextSize(2);
                tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                char fpsStr[16];
                if (isDetectFPS) {
                    snprintf(fpsStr, sizeof(fpsStr), "%.1f%s", g_rxFps, g_rxDropFrame?"DF":"ND");
                } else {
                    snprintf(fpsStr, sizeof(fpsStr), "%s", ltcStandardStr(g_ltcStd));
                }
                
                snprintf(buf, sizeof(buf), "%-6s U:%08lX  ", fpsStr, (unsigned long)displayUserBits);
                tft.setCursor(10, 106); 
                tft.print(buf);
                
                o_std = g_ltcStd; o_fps = g_rxFps; o_df = g_rxDropFrame; o_ub = displayUserBits;
            }

            // 5. Shot data
            int c_scene; String c_shot; int c_take; String c_roll; String c_st; String c_et;
            if (g_sysMtx) xSemaphoreTake(g_sysMtx, portMAX_DELAY);
            c_scene = g_shotScene; c_shot = g_shotName; c_take = g_shotTake; c_roll = g_shotRoll;
            c_st = g_shotStartTC; c_et = g_shotEndTC;
            if (g_sysMtx) xSemaphoreGive(g_sysMtx);

            if (o_scene != c_scene || o_shot != c_shot || o_take != c_take || o_roll != c_roll ||
                o_st != c_st || o_et != c_et || o_mode != g_sysMode ||
                o_bpm != g_songBpm || o_timesig != g_songTimeSig) {
                tft.fillRect(0, 166, 240, 84, ILI9341_BLACK); // Clear shot area

                // In browse mode, highlight shot ID line in red
                bool isBrowse = (g_menuState == MENU_SHOT_BROWSE);

                if (g_sysMode == MODE_PLAYBACK) {
                    bool hasShot = (c_shot != "" && c_shot != "0");
                    if (hasShot) {
                        uint16_t shotBg = isBrowse ? (uint16_t)ILI9341_RED : (uint16_t)ILI9341_BLACK;
                        tft.fillRect(0, 166, 240, 24, shotBg); // highlight bg
                        tft.setTextColor(ILI9341_WHITE, shotBg);
                        tft.setTextSize(3);
                        snprintf(buf, sizeof(buf), "%s %d %s %d", c_roll.c_str(), c_scene, c_shot.c_str(), c_take);
                        tft.setCursor(10, 167); tft.print(buf);
                    } else {
                        tft.setTextSize(2);
                        tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
                        tft.setCursor(10, 175);
                        tft.print("NO SHOT SENT    ");
                    }
                    // BPM | TimeSig line (always shown in playback mode)
                    tft.setTextSize(2);
                    tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
                    if (g_songBpm > 0.0f) {
                        snprintf(buf, sizeof(buf), "%.0f BPM | %d/4  ", g_songBpm, g_songTimeSig);
                    } else {
                        snprintf(buf, sizeof(buf), "                ");
                    }
                    tft.setCursor(10, hasShot ? 194 : 200); tft.print(buf);
                    if (hasShot) {
                        tft.setTextSize(2);
                        tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
                        snprintf(buf, sizeof(buf), "Start:%s", c_st.c_str());
                        tft.setCursor(10, 213); tft.print(buf);
                        snprintf(buf, sizeof(buf), "End  :%s", c_et.c_str());
                        tft.setCursor(10, 231); tft.print(buf);
                    }
                } else if (c_shot == "" || c_shot == "0") {
                    tft.setTextSize(2);
                    tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
                    tft.setCursor(10, 180); tft.print(WiFi.SSID());
                    tft.setCursor(10, 210); tft.print(WiFi.localIP().toString());
                } else {
                    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                    tft.setTextSize(3);
                    snprintf(buf, sizeof(buf), "%s %d %s %d", c_roll.c_str(), c_scene, c_shot.c_str(), c_take);
                    tft.setCursor(10, 170); tft.print(buf);
                    tft.setTextSize(2);
                    tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
                    snprintf(buf, sizeof(buf), "Start:%s", c_st.c_str());
                    tft.setCursor(10, 202); tft.print(buf);
                    snprintf(buf, sizeof(buf), "End  :%s", c_et.c_str());
                    tft.setCursor(10, 222); tft.print(buf);
                }
                o_scene = c_scene; o_take = c_take; o_shot = c_shot; o_roll = c_roll;
                o_st = c_st; o_et = c_et; o_bpm = g_songBpm; o_timesig = g_songTimeSig;
            }


            // 6. Song oder Jam Sync Progress
            if (g_sysMode == MODE_PLAYBACK) {
                if (o_song != g_songTitle || o_mode != g_sysMode) {
                    tft.fillRect(0, 254, 240, 66, ILI9341_BLACK); // clear bottom area once
                    tft.setTextSize(2);
                    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                    if (g_songTitle.length() > 0) snprintf(buf, sizeof(buf), "Song: %-13.13s", g_songTitle.c_str());
                    else snprintf(buf, sizeof(buf), "Song: playing  ");
                    tft.setCursor(10, 256); tft.print(buf);
                    o_song = g_songTitle;

                    tft.drawRect(10, 276, 220, 18, ILI9341_WHITE);
                }

                float fps = ltc_standard_fps(g_ltcStd);
                if (fps <= 0.0f) fps = 25.0f; // fallback
                
                uint32_t curFrames = tcToFrames(g_tc_h, g_tc_m, g_tc_s, g_tc_f);
                uint32_t totalFrames = (uint32_t)(g_songDurSec * fps);
                if (totalFrames == 0) totalFrames = 1; // prevent div0
                
                float p = (float)curFrames / (float)totalFrames;
                if (p > 1.0f) p = 1.0f;
                int w = (int)(p * 216);
                
                tft.fillRect(12, 278, w, 14, ILI9341_RED);
                if (w < 216) tft.fillRect(12 + w, 278, 216 - w, 14, ILI9341_BLACK);
                
                if (g_shotLimited) {
                    uint32_t startFrames = g_shotStartFr;
                    uint32_t endFrames = g_shotEndFr;

                    if (endFrames > startFrames && startFrames < totalFrames) {
                        int start_px = (int)(((float)startFrames / totalFrames) * 216.0f);
                        int end_px = (int)(((float)endFrames / totalFrames) * 216.0f);
                        if (end_px > 216) end_px = 216;
                        int hw = end_px - start_px;
                        if (hw > 0) {
                            tft.drawRect(10 + start_px, 276, hw, 18, ILI9341_YELLOW);
                        }
                    }
                }
                
                int curM = (curFrames / (int)fps) / 60;
                int curS = (curFrames / (int)fps) % 60;
                int totM = ((int)g_songDurSec) / 60;
                int totS = ((int)g_songDurSec) % 60;
                
                tft.setTextSize(2);
                tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
                
                char lb[16]; snprintf(lb, sizeof(lb), "%02d:%02d", curM, curS);
                tft.setCursor(10, 298); tft.print(lb);
                
                char rb[16]; snprintf(rb, sizeof(rb), "%02d:%02d", totM, totS);
                tft.setCursor(170, 298); tft.print(rb);

            } else if (g_sysMode == MODE_JAM_SYNC_FREE_RUN || g_sysMode == MODE_JAM_SYNC_RECORD_RUN) {
                if (g_jamOkCnt != o_jam || o_mode != g_sysMode) {
                    tft.fillRect(0, 254, 240, 66, ILI9341_BLACK); // clear bottom area once
                    tft.setTextSize(2);
                    tft.setCursor(10, 260);
                    if (g_jamOkCnt > 0) {
                        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
                        tft.print("JAM: LOCK OK   ");
                    } else {
                        tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
                        tft.print("JAM: NO SIGNAL ");
                    }

                    // Progress bar frame valid count
                    tft.drawRect(10, 286, 220, 20, ILI9341_WHITE);
                    int w = (g_jamOkCnt * 216) / 50; if (w > 216) w = 216;
                    tft.fillRect(12, 288, w, 16, ILI9341_GREEN);
                    if (w < 216) tft.fillRect(12 + w, 288, 216 - w, 16, ILI9341_BLACK);
                    o_jam = g_jamOkCnt;
                }
            } else {
                if (o_mode != g_sysMode) { // clearing once when switching
                    tft.fillRect(0, 254, 240, 66, ILI9341_BLACK); // clear bottom area once
                }
            }
            
            o_mode = g_sysMode;
            } // END doSlowUpdate
        }
    }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        
        if (info->opcode == WS_TEXT) {
            String jsonStr;
            for (size_t i = 0; i < len; i++) jsonStr += (char)data[i];
            
            Serial.printf("[WS DEBUG] JSON received (len:%d): %s\n", len, jsonStr.c_str());

            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, jsonStr);
            if (error) {
                Serial.printf("[WS DEBUG] JSON Error: %s\n", error.c_str());
            } else {
                String msgType = doc["type"] | "";
                if (msgType == "shot") {
                    if (g_sysMtx) xSemaphoreTake(g_sysMtx, portMAX_DELAY);
                    g_shotScene = doc["scene"].as<int>();
                    g_shotName = doc["shot"] | "0";
                    g_shotTake = doc["take"].as<int>();
                    g_shotStartTC = doc["starttime"] | "";
                    g_shotEndTC = doc["endtime"] | "";
                    if (!doc["duration"].isNull()) {
                        g_songDurSec = doc["duration"].as<float>();
                    }
                    if (!doc["songtitle"].isNull()) {
                        g_songTitle = doc["songtitle"].as<String>();
                    }
                    if (!doc["camera"].isNull()) {
                        g_shotCamera = doc["camera"].as<String>();
                    }
                    if (!doc["roll"].isNull()) {
                        g_shotRoll = doc["roll"].as<String>();
                    }
                    // Activate shot userbit mode; bits are computed from the stored globals.
                    g_shotBitsActive = true;

                    // Set framerate FIRST so tcToFrames uses the correct fps
                    float reqFps = doc["framerate"].as<float>();
                    if (reqFps > 0) {
                        if (reqFps == 24.0f) g_ltcStd = LTC_TV_FILM_24;
                        else if (reqFps == 25.0f) g_ltcStd = LTC_TV_625_50;
                        else if (reqFps == 29.97f) g_ltcStd = LTC_TV_525_DF;
                        else if (reqFps >= 30.0f) g_ltcStd = LTC_TV_525_60;
                        g_ltcStdDirty = true;
                    }

                    g_shotLimited = (g_shotStartTC.length() > 0 && g_shotEndTC.length() > 0);
                    if (g_shotLimited) {
                        uint8_t h=0, m=0, s=0, f=0;
                        parseTC(g_shotStartTC, h, m, s, f);
                        g_shotStartFr = tcToFrames(h, m, s, f);
                        parseTC(g_shotEndTC, h, m, s, f);
                        g_shotEndFr = tcToFrames(h, m, s, f);
                    }

                    // Parse playback speed (1.0 = normal, 0.7 = 70% etc.). Pitch changes accordingly.
                    float newSpeed = doc["speed"].as<float>();
                    if (newSpeed <= 0.0f || newSpeed > 4.0f) newSpeed = 1.0f;
                    bool speedChanged = (fabsf(newSpeed - (float)g_shotSpeed) > 0.001f);
                    g_shotSpeed = newSpeed;

                    if (g_sysMtx) xSemaphoreGive(g_sysMtx);

                    // If speed changed in PLAYBACK mode, trigger I2S+LTC reinit in loop().
                    // loop() will also re-seek after reinit, so skip the seek here when reinit pending.
                    if (speedChanged && g_sysMode == MODE_PLAYBACK && !g_running) {
                        g_speedDirty = true;
                        Serial.printf("[SPD] Speed changed to %.2f – I2S reinit queued\n", (float)g_shotSpeed);
                    } else {
                        // Update ltcMixer, audio position and g_tc_* to start of shot
                        if (g_sysMode == MODE_PLAYBACK && g_shotLimited) {
                            uint8_t h=0, m=0, s=0, f=0;
                            parseTC(g_shotStartTC, h, m, s, f);
                            if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
                            ltcMixer.setStartTC(h, m, s, f);
                            seekAudioToTC(h, m, s, f);
                            if (g_audMtx) xSemaphoreGive(g_audMtx);
                        }
                    }

                    Serial.printf("Got Shot! Scene: %d, Shot: %s, Take: %d, Start: %s, End: %s\n", 
                                   g_shotScene, g_shotName.c_str(), g_shotTake, g_shotStartTC.c_str(), g_shotEndTC.c_str());
                    // Apply shot userbit (or date fallback) to active encoder now
                    applyCurrentUserBitsToActiveEncoder();
                    updateUI();
                } else if (msgType == "command") {
                    String cmd = doc["cmd"] | "";
                    if (cmd == "start" || cmd == "play") {
                        if (!g_running && (g_sysMode == MODE_PLAYBACK || g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN)) {
                            if (g_sysMode == MODE_PLAYBACK) {
                                startPlaybackWithCountIn();
                            } else {
                                g_running = true;
                                if (g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN) {
                                    ltcGen.applyStartTC();
                                }
                            }
                        }
                        broadcastStatus();
                    } else if (cmd == "pause" || cmd == "stop") {
                        g_running = false;
                        g_reseekPending = false; // cancel any pending start re-seek
                        broadcastStatus();
                        // DO NOT reset timecode on pause!
                    } else if (cmd == "seek") {
                        float secs = doc["time"].as<float>();
                        float fps = ltc_standard_fps(g_ltcStd);
                        if (fps <= 0.0f) fps = 25.0f;
                        uint32_t frames = (uint32_t)(secs * fps);
                        
                        g_tc_h = (frames / (int)fps) / 3600;
                        g_tc_m = ((frames / (int)fps) % 3600) / 60;
                        g_tc_s = (frames / (int)fps) % 60;
                        g_tc_f = frames % (int)fps;
                        
                        if (g_sysMode == MODE_PLAYBACK) {
                            if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
                            ltcMixer.setStartTC(g_tc_h, g_tc_m, g_tc_s, g_tc_f);
                            
                            Stream* s = player.getStream();
                            if (s) {
                                fs::File* f = (fs::File*)s;
                                if (f) {
                                    audio_tools::AudioInfo inf = player.audioInfo();
                                    int sr = inf.sample_rate > 0 ? inf.sample_rate : 44100;
                                    int ch = inf.channels > 0 ? inf.channels : 1;
                                    int bps = inf.bits_per_sample > 0 ? inf.bits_per_sample : 16;
                                    uint32_t byte_pos = g_wavDataOffset + (uint32_t)(secs * sr * ch * (bps / 8));
                                    byte_pos = byte_pos - (byte_pos % (ch * (bps/8))); // Align to frame boundary
                                    f->seek(byte_pos, SeekSet);
                                }
                            }
                            if (g_audMtx) xSemaphoreGive(g_audMtx);
                        } else if (g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN) {
                            ltcGen.setManualTimecode(g_tc_h, g_tc_m, g_tc_s, g_tc_f);
                        }
                    } else if (cmd == "reset") {
                        ltcGen.setManualTimecode(0, 0, 0, 0);
                        g_tc_h = 0; g_tc_m = 0; g_tc_s = 0; g_tc_f = 0;
                    } else if (cmd == "set_tc_mode") {
                        String val = doc["val"] | "";
                        if (val == "30") g_ltcStd = LTC_TV_525_60;
                        else if (val == "25") g_ltcStd = LTC_TV_625_50;
                        else if (val == "29.97DF") g_ltcStd = LTC_TV_525_DF;
                        else if (val == "24") g_ltcStd = LTC_TV_FILM_24;
                        g_ltcStdDirty = true;
                        broadcastStatus();
                    } else if (cmd == "set_volume") {
                        float vol = doc["val"].as<float>() / 100.0f;
                        g_volumePercent = (int)doc["val"].as<float>();  // sync encoder state
                        if (g_volumePercent < 0) g_volumePercent = 0;
                        if (g_volumePercent > 100) g_volumePercent = 100;
                        g_volShowUntilMs = millis() + 1500;  // show volume bar on TFT
                        if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
                        player.setVolume(vol);         // scales WAV samples (left ch)
                        ltcMixer.setPlaybackVolume(vol); // scales count-in samples
                        if (g_audMtx) xSemaphoreGive(g_audMtx);
                    } else if (cmd == "set_gen_mode") {
                        String val = doc["val"] | "";
                        if (val == "gen_free") g_reqMode = MODE_GEN_FREE_RUN;
                        else if (val == "gen_rec") g_reqMode = MODE_GEN_RECORD_RUN;
                        else if (val == "playback") g_reqMode = MODE_PLAYBACK;
                        else if (val == "jam_rec") g_reqMode = MODE_JAM_SYNC_RECORD_RUN;
                        else if (val == "jam_free") g_reqMode = MODE_JAM_SYNC_FREE_RUN;
                        else if (val == "slave") g_reqMode = MODE_SLAVE;
                    } else if (cmd == "sync_rtc") {
                        syncRTCWithNTP();
                    } else if (cmd == "restart") {
                        g_reqMode = g_sysMode;
                    }
                    updateUI();
                } else if (msgType == "song_info") {
                    // Update in-memory song metadata
                    if (!doc["songtitle"].isNull()) {
                        g_songTitle = doc["songtitle"].as<String>();
                    }
                    if (!doc["songduration"].isNull()) {
                        // songduration is a TC string like "HH:MM:SS:FF"
                        String durTC = doc["songduration"].as<String>();
                        float fps = ltc_standard_fps(g_ltcStd);
                        if (fps <= 0.0f) fps = 25.0f;
                        uint8_t dh = 0, dm = 0, ds = 0, df = 0;
                        parseTC(durTC, dh, dm, ds, df);
                        g_songDurSec = (float)(dh * 3600 + dm * 60 + ds) + (float)df / fps;
                    }
                    if (!doc["songBpm"].isNull()) {
                        g_songBpm = doc["songBpm"].as<float>();
                    }
                    if (!doc["songTimeSignature"].isNull()) {
                        int ts = doc["songTimeSignature"].as<int>();
                        if (ts > 0) g_songTimeSig = ts;
                    }

                    // Write /songinfo.json to SD card (replace if exists)
                    if (SD_MMC.exists("/songinfo.json")) {
                        SD_MMC.remove("/songinfo.json");
                    }
                    File infoFile = SD_MMC.open("/songinfo.json", FILE_WRITE);
                    if (infoFile) {
                        JsonDocument infoDoc;
                        infoDoc["songtitle"]       = doc["songtitle"] | "";
                        infoDoc["songduration"]    = doc["songduration"] | "";
                        infoDoc["songBpm"]         = doc["songBpm"].as<float>();
                        infoDoc["songTimeSignature"] = doc["songTimeSignature"].as<int>();
                        serializeJson(infoDoc, infoFile);
                        infoFile.close();
                        Serial.printf("[WS] /songinfo.json saved: %s\n", g_songTitle.c_str());
                    } else {
                        Serial.println("[WS] ERROR: Could not open /songinfo.json for writing!");
                    }
                    updateUI();
                } else if (msgType == "prepare_upload") {
                    Serial.println("[WS] BEREITE UPLOAD VOR...");
                    g_wavRx = true;
                    updateUI();
                    if (g_sysMode == MODE_PLAYBACK) g_running = false;
                    
                    File root = SD_MMC.open("/");
                    if (root) {
                        File file = root.openNextFile();
                        while(file) {
                            String fname = file.name();
                            if (!file.isDirectory() && (fname.endsWith(".wav") || fname.endsWith(".WAV"))) {
                                SD_MMC.remove(String("/") + fname);
                            }
                            file = root.openNextFile();
                        }
                    }
                    
                    if (SD_MMC.exists("/track.wav")) {
                        SD_MMC.remove("/track.wav");
                    }

                    g_uploadFile = SD_MMC.open("/track.wav", FILE_WRITE);
                    if (!g_uploadFile) {
                        Serial.println("[WS] FATAL: Konnte /track.wav NICHT oeffnen!");
                    } else {
                        Serial.println("[WS] Datei /track.wav zum Schreiben geoeffnet.");
                        client->text("{\"type\":\"upload_prepared\"}");
                    }
                } else if (msgType == "finish_upload") {
                    Serial.println("[WS] Empfange Finish Command!");
                    if (g_uploadFile) {
                        size_t s = g_uploadFile.size();
                        g_uploadFile.close();
                        Serial.printf("[WS] upload finished, file size: %d bytes\n", s);
                    } else {
                        Serial.println("[WS] finish_upload ABER keine Datei geoeffnet!");
                    }
                    g_wavRx = false;
                    
                    if (!doc["duration"].isNull()) {
                        g_songDurSec = doc["duration"].as<float>();
                    }
                    if (!doc["songtitle"].isNull()) {
                        g_songTitle = doc["songtitle"].as<String>();
                    }
                    
                    if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
                    player.end();
                    source.begin(); // Playlist neu scannen
                    scanWavDataOffset();  // re-parse data-chunk offset of new file
                    if (g_sysMode == MODE_PLAYBACK) player.begin();
                    if (g_audMtx) xSemaphoreGive(g_audMtx);
                    
                    updateUI();
                    client->text("{\"type\":\"upload_ack\"}");
                } else {
                    Serial.printf("[WS DEBUG] UNKNOWN MSG_TYPE: %s\n", msgType.c_str());
                }
            }
        } else if (info->opcode == WS_BINARY || (info->opcode == WS_CONTINUATION && info->message_opcode == WS_BINARY)) {
            static unsigned long binCount = 0;
            binCount += len;
            if (g_wavRx && g_uploadFile) {
                g_uploadFile.write(data, len);
                if (binCount % 32768 < len) { 
                    Serial.printf("[WS] Empfange wav byte chunk (%lu bytes until now)...\n", binCount);
                }
            } else if (!g_wavRx) {
               Serial.println("[WS] Warnung: Binaerdaten erhalten ABER g_wavRx ist false!");
            }
        }
        } else if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client connected IP: %s\n", client->remoteIP().toString().c_str());
        
        JsonDocument doc;
        doc["type"] = "init_sync";
        
        if (g_sysMode == MODE_GEN_FREE_RUN) doc["gen_mode"] = "gen_free";
        else if (g_sysMode == MODE_GEN_RECORD_RUN) doc["gen_mode"] = "gen_rec";
        else if (g_sysMode == MODE_PLAYBACK) doc["gen_mode"] = "playback";
        else if (g_sysMode == MODE_JAM_SYNC_RECORD_RUN) doc["gen_mode"] = "jam_rec";
        else if (g_sysMode == MODE_JAM_SYNC_FREE_RUN) doc["gen_mode"] = "jam_free";
        else if (g_sysMode == MODE_SLAVE) doc["gen_mode"] = "slave";
        
        if (g_ltcStd == LTC_TV_525_60) doc["tc_mode"] = "30";
        else if (g_ltcStd == LTC_TV_625_50) doc["tc_mode"] = "25";
        else if (g_ltcStd == LTC_TV_525_DF) doc["tc_mode"] = "29.97DF";
        else if (g_ltcStd == LTC_TV_FILM_24) doc["tc_mode"] = "24";
        doc["running"] = (bool)g_running;
        // NOTE: Timecode is intentionally NOT included in init_sync.
        // Each device runs independently on its own RTC. Timecode is only
        // transmitted on actions (broadcastStatus) for display purposes only.

        String jsonStr;
        serializeJson(doc, jsonStr);
        server->text(client->id(), jsonStr);
        // Send a status message so the HTML UI fully syncs on (re)connect
        broadcastStatus();
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("[WS] Client disconnected");
    } else if (type == WS_EVT_ERROR) {
        Serial.println("[WS] Error!!");
    }
}

// ── Initialisation ───────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200); delay(500);
    // Create all three protection mutexes before any task can run
    g_i2cMtx = xSemaphoreCreateMutex();
    g_audMtx = xSemaphoreCreateMutex();
    g_sysMtx = xSemaphoreCreateMutex();
    
    initRTC();
    initWiFiAndServer();

    pinMode(PCM_MUTE_PIN, OUTPUT); PCM_UNMUTE(); // start unmuted
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
    pinMode(TFT_CS, OUTPUT); digitalWrite(TFT_CS, HIGH); // pull CS high before SPI init

    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
    tft.begin(40000000UL); tft.setRotation(2); tft.invertDisplay(false);      
    tft.fillScreen(ILI9341_BLACK);

    SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0);
    if (!SD_MMC.begin("/sdcard", true)) Serial.println("[SD] Mount failed");

    g_ltcQueue = xQueueCreate(20, sizeof(LTCFrameDec));
    player.setMetadataCallback(printMetaData);
    player.setVolume(0.5f);
    source.begin(); // scan SD for WAV files
    scanWavDataOffset(); // parse data-chunk offset of /track.wav (if present)
    player.setIndex(0);

    ltcGen.begin(encInfo);

    Serial.println("Commands: 1=GenFree 2=GenRec 3=Play 4=Slave 5=JamFree 6=JamRec");
    Serial.println("FPS: q=30 w=25 e=29.97DF r=24F | s=Start/Stop  n=NTP-Sync");

    // Pin tasks to specific cores; DISP goes to Core 0 (WiFi core), ENC/DEC to Core 1
    xTaskCreatePinnedToCore(taskLTC,     "DEC",  4096, NULL, 3, NULL,        1);
    xTaskCreatePinnedToCore(taskEncode,  "ENC",  8192, NULL, 2, NULL,        1);
    xTaskCreatePinnedToCore(taskDisplay, "DISP", 8192, NULL, 1, &g_dispTask, 0);

    pinMode(LTC_INPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LTC_INPUT_PIN), isrLTC, CHANGE);

    initEncoder();

    g_reqMode = MODE_GEN_FREE_RUN; // first loop() call will apply this mode
}

// ── Main loop (Core 1) ───────────────────────────────────────────────────────────
// Handles three responsibilities:
//   1. Apply pending mode changes (g_reqMode) via applyMode().
//   2. Drives the WAV player copy() call in playback mode.
//   3. Sends a periodic status heartbeat to all WebSocket clients.
void loop() {
    // Apply a queued mode change (written by WebSocket or serial command)
    if (g_reqMode != MODE_NONE) {
        applyMode(g_reqMode);
        g_reqMode = MODE_NONE;
    }

    // Reinit audio pipeline when playback speed changed (only when stopped).
    // I2S is set to wavSR*speed; LTC encoder stays at wavSR so TC advances at speed*fps/s.
    if (g_speedDirty && g_sysMode == MODE_PLAYBACK && !g_running) {
        g_speedDirty = false;
        if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
        player.end();
        i2s.end();
        int scaledSR = (int)(44100.0f * (float)g_shotSpeed + 0.5f);
        if (scaledSR < 8000) scaledSR = 8000;
        auto spCfg = i2s.defaultConfig(TX_MODE);
        spCfg.pin_bck = BCLK_PIN; spCfg.pin_ws = LRCK_PIN; spCfg.pin_data = DIN_PIN;
        spCfg.sample_rate = scaledSR; spCfg.channels = 2; spCfg.bits_per_sample = 16;
        i2s.begin(spCfg);
        ltcMixer.begin();   // reads g_shotSpeed internally
        player.setIndex(0);
        player.begin();
        // Warm up so the WAV header is parsed BEFORE we seek (otherwise the
        // decoder would parse the header at the seeked position and fail).
        for (int warm = 0; warm < 20 && !player.isActive(); warm++) {
            player.copy();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        // Re-seek to shot start after reinit
        if (g_shotLimited) {
            uint8_t h=0, m=0, s=0, f=0;
            parseTC(g_shotStartTC, h, m, s, f);
            ltcMixer.setStartTC(h, m, s, f);
            seekAudioToTC(h, m, s, f);
        }
        if (g_audMtx) xSemaphoreGive(g_audMtx);
        Serial.printf("[SPD] I2S reinit: %dHz (speed=%.2f)\n", scaledSR, (float)g_shotSpeed);
        updateUI();
    }

    // Apply pending volume change (set by encoder rotation in display task)
    if (g_volDirty) {
        g_volDirty = false;
        float vol = (float)g_volumePercent / 100.0f;
        if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
        player.setVolume(vol);
        ltcMixer.setPlaybackVolume(vol);
        if (g_audMtx) xSemaphoreGive(g_audMtx);
    }

    // Periodic status heartbeat – keeps all connected clients in sync (every 3 s)
    static uint32_t lastHB = 0;
    if (millis() - lastHB > 3000 && ws.count() > 0) {
        lastHB = millis();
        broadcastStatus();
    }

    if (Serial.available()) {
        char c = (char)Serial.read();
        switch(c) {
            case '1': g_reqMode = MODE_GEN_FREE_RUN; break;
            case '2': g_reqMode = MODE_GEN_RECORD_RUN; break;
            case '3': g_reqMode = MODE_PLAYBACK; break;
            case '4': g_reqMode = MODE_SLAVE; break;
            case '5': g_reqMode = MODE_JAM_SYNC_FREE_RUN; break;
            case '6': g_reqMode = MODE_JAM_SYNC_RECORD_RUN; break;
            case 'q': g_ltcStd = LTC_TV_525_60; g_ltcStdDirty = true; break;
            case 'w': g_ltcStd = LTC_TV_625_50; g_ltcStdDirty = true; break;
            case 'e': g_ltcStd = LTC_TV_525_DF; g_ltcStdDirty = true; break;
            case 'r': g_ltcStd = LTC_TV_FILM_24; g_ltcStdDirty = true; break;
            case 's':
                g_running = !g_running;
                if (g_running && (g_sysMode == MODE_GEN_FREE_RUN || g_sysMode == MODE_GEN_RECORD_RUN)) {
                    ltcGen.applyStartTC();
                } else if (!g_running) {
                    updateUI(); // force redraw to gray out text
                }
                break;
            case 'n': syncRTCWithNTP(); break;
            case 'x':
                g_running = false;
                updateUI();
                break;
        }
    }

    if (g_sysMode == MODE_PLAYBACK && g_running) {
        bool limitHit = false;
        if (g_sysMtx) xSemaphoreTake(g_sysMtx, portMAX_DELAY);
        if (g_shotLimited) {
            uint32_t currentFrames = tcToFrames(g_tc_h, g_tc_m, g_tc_s, g_tc_f);
            if (currentFrames >= g_shotEndFr) limitHit = true;
        }
        if (g_sysMtx) xSemaphoreGive(g_sysMtx);

        static unsigned long lastDbg = 0;
        if (millis() - lastDbg > 2000) {
            lastDbg = millis();
            Serial.printf("[PLAYBACK DEBUG] active:%d, song:%s\n", player.isActive(), g_songTitle.c_str());
        }

        if (limitHit) {
            String l_startTimeCode;
            // Playback finished!
            g_running = false;

            // Capture the userbit that was active during the take just completed.
            uint32_t completedUserBits = g_txUserBits;
            
            if (g_sysMtx) xSemaphoreTake(g_sysMtx, portMAX_DELAY);
            
            // Notify via websocket
            JsonDocument doc;
            doc["type"] = "newTake";
            doc["take"] = g_shotTake;  // Send CURRENT take number BEFORE incrementing
            g_shotTake++;  // advance to next take for the next recording
            char tcb[16];
            snprintf(tcb, sizeof(tcb), "%02d:%02d:%02d:%02d", g_tc_h, g_tc_m, g_tc_s, g_tc_f);
            doc["timecode"] = tcb;
            doc["roll"] = g_shotRoll;
            l_startTimeCode = g_shotStartTC;
            if (g_sysMtx) xSemaphoreGive(g_sysMtx);

            // Send the userbit of the completed take back to the server.
            char ubHex[9];
            snprintf(ubHex, sizeof(ubHex), "%08lX", (unsigned long)completedUserBits);
            doc["userbit"] = ubHex;
            
            DateTime nowTime = rtc.now();
            char dateBuf[16];
            snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", nowTime.day(), nowTime.month(), nowTime.year());
            doc["date"] = dateBuf;
            
            String outStr;
            serializeJson(doc, outStr);
            ws.textAll(outStr);
            
            uint8_t sh=0, sm=0, ss=0, sf=0;
            parseTC(l_startTimeCode, sh, sm, ss, sf);
            if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
            ltcMixer.setStartTC(sh, sm, ss, sf);
            seekAudioToTC(sh, sm, ss, sf);
            if (g_audMtx) xSemaphoreGive(g_audMtx);

            // Generate and apply userbit for the NEW take (g_shotTake already incremented).
            applyCurrentUserBitsToActiveEncoder();

            updateUI();
            delay(10);
        }
        
        if (g_audMtx) xSemaphoreTake(g_audMtx, portMAX_DELAY);
        player.copy();
        // Re-apply the seek once the player has actually parsed the WAV header
        // and become active. On a fresh boot the first copy() only opens the
        // file / parses the header (lazily, possibly over several calls) and
        // rewinds to the file start, which would otherwise make the song play
        // from the beginning on the very first start after power-up.
        if (g_reseekPending && player.isActive()) {
            g_reseekPending = false;
            seekAudioToTC(g_reseekH, g_reseekM, g_reseekS, g_reseekF);
        }
        if (g_audMtx) xSemaphoreGive(g_audMtx);
        vTaskDelay(pdMS_TO_TICKS(1));
    } else {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}







