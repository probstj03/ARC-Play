#include "play_ui.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ArcCore.h>
#include <SD_MMC.h>
#include <SPI.h>
#include "pins.h"
#include "play_audio.h"
#include "play_link.h"
#include "play_state.h"

using arc::engine::Mode;
using arc::TcRate;

namespace {

Adafruit_ILI9341 s_tft(&SPI, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST);
TaskHandle_t s_task = nullptr;

// ── encoder (full-step state table) ─────────────────────────────────────
constexpr uint8_t R_START = 0x0, R_CW_FINAL = 0x1, R_CW_BEGIN = 0x2, R_CW_NEXT = 0x3;
constexpr uint8_t R_CCW_BEGIN = 0x4, R_CCW_FINAL = 0x5, R_CCW_NEXT = 0x6;
constexpr uint8_t DIR_CW = 0x10, DIR_CCW = 0x20;
const uint8_t kTable[7][4] = {
    {R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START},
    {R_CW_NEXT, R_START, R_CW_FINAL, R_START | DIR_CW},
    {R_CW_NEXT, R_CW_BEGIN, R_START, R_START},
    {R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START},
    {R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START},
    {R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | DIR_CCW},
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
volatile uint8_t s_encState = R_START;
volatile int32_t s_encSteps = 0;
portMUX_TYPE s_encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrEncoder() {
    const uint8_t pins = (digitalRead(PIN_ENC_DT) << 1) | digitalRead(PIN_ENC_CLK);
    s_encState = kTable[s_encState & 0x0F][pins];
    const uint8_t dir = s_encState & 0x30;
    portENTER_CRITICAL_ISR(&s_encMux);
    if (dir == DIR_CW) s_encSteps = s_encSteps + 1;
    else if (dir == DIR_CCW) s_encSteps = s_encSteps - 1;
    portEXIT_CRITICAL_ISR(&s_encMux);
}

// ── menu ────────────────────────────────────────────────────────────────
enum MenuState { MENU_CLOSED, MENU_SETTINGS, MENU_MODE, MENU_FPS, MENU_FORMAT_SD, MENU_SHOT_BROWSE };
MenuState s_menu = MENU_CLOSED;
int s_sel = 0;

const char* const kSettings[] = {"Mode", "Framerate", "Sync RTC (NTP)", "Format SD-Card", "Close"};
const char* const kModes[] = {"Gen: Free Run", "Gen: Record Run", "Playback", "Slave", "Jam Sync Free", "Jam Sync Rec", "Back"};
const Mode kModeValues[] = {Mode::FreeRun, Mode::RecordRun, Mode::Playback, Mode::Slave, Mode::JamFree, Mode::JamRec};
const char* const kFps[] = {"24 fps", "25 fps (ND)", "29.97 DF", "30 fps (ND)", "Back"};
const TcRate kFpsValues[] = {TcRate::R24, TcRate::R25, TcRate::R2997DF, TcRate::R30};
const char* const kFormat[] = {"Format SD-Card", "Back"};

void menuItems(const char* const*& items, int& count) {
    switch (s_menu) {
        case MENU_MODE:      items = kModes;    count = 7; return;
        case MENU_FPS:       items = kFps;      count = 5; return;
        case MENU_FORMAT_SD: items = kFormat;   count = 2; return;
        default:             items = kSettings; count = 5; return;
    }
}

const char* menuTitle() {
    switch (s_menu) {
        case MENU_MODE:      return "  MODE";
        case MENU_FPS:       return "  FRAMERATE";
        case MENU_FORMAT_SD: return "  FORMAT SD";
        default:             return "  SETTINGS";
    }
}

void drawMenuItem(int idx, bool selected) {
    const char* const* items; int count;
    menuItems(items, count);
    if (idx < 0 || idx >= count) return;
    const bool isBack = idx == count - 1 && s_menu != MENU_SETTINGS;
    const uint16_t bg = selected ? ILI9341_RED : ILI9341_BLACK;
    const uint16_t fg = (isBack && !selected) ? ILI9341_DARKGREY : ILI9341_WHITE;
    s_tft.fillRect(0, 30 + idx * 40, 240, 40, bg);
    s_tft.drawFastHLine(0, 30 + idx * 40 + 39, 240, ILI9341_DARKGREY);
    s_tft.setTextColor(fg, bg);
    s_tft.setTextSize(2);
    s_tft.setCursor(12, 30 + idx * 40 + 12);
    s_tft.print(items[idx]);
}

void drawMenuScreen() {
    const char* const* items; int count;
    menuItems(items, count);
    s_tft.fillScreen(ILI9341_BLACK);
    s_tft.drawFastHLine(0, 29, 240, ILI9341_RED);
    s_tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    s_tft.setTextSize(2);
    s_tft.setCursor(8, 8);
    s_tft.print(menuTitle());
    for (int i = 0; i < count; i++) drawMenuItem(i, i == s_sel);
}

// ── status screen ───────────────────────────────────────────────────────
struct Cache {
    arc::Timecode tc;
    bool tcValid = false;
    bool running = false;
    int mode = -1;
    int rate = -1;
    uint32_t ub = 0xFFFFFFFF;
    uint32_t rev = 0;
    int rtcMin = -1;
    int vol = -1;
    int jam = -1;
    bool countIn = false;
    bool browse = false;
    bool upload = false;
} s_o;

bool s_full = true;
uint32_t s_volShowUntil = 0;

void drawFrame() {
    s_tft.fillScreen(ILI9341_BLACK);
    s_tft.setTextColor(ILI9341_WHITE);
    s_tft.setTextSize(2);
    s_tft.setCursor(70, 4);
    s_tft.print("ARC PLAY");
    s_tft.drawFastHLine(0, 24, 240, ILI9341_DARKGREY);
    s_tft.drawFastHLine(0, 70, 240, ILI9341_DARKGREY);
    s_tft.drawFastHLine(0, 160, 240, ILI9341_DARKGREY);
    s_tft.drawFastHLine(0, 250, 240, ILI9341_DARKGREY);
    s_o = Cache();
}

bool isRunning() {
    return arc::engine::mode() == Mode::Playback ? audio::playing() : arc::engine::running();
}

const char* modeLabel(Mode m) {
    switch (m) {
        case Mode::FreeRun:   return "GEN: FREE RUN";
        case Mode::RecordRun: return "GEN: REC RUN";
        case Mode::Playback:  return "PLAYBACK";
        case Mode::Slave:     return "SLAVE DECODER";
        case Mode::JamFree:   return "JAM SYNC FREE";
        case Mode::JamRec:    return "JAM SYNC REC";
    }
    return "";
}

void drawStatus() {
    char buf[64];
    const Mode mode = arc::engine::mode();
    const bool running = isRunning();

    if (audio::uploading()) {
        if (!s_o.upload) {
            s_tft.fillScreen(ILI9341_BLACK);
            s_tft.setTextColor(ILI9341_WHITE);
            s_tft.setTextSize(3);
            s_tft.setCursor(20, 100); s_tft.print("RECEIVING");
            s_tft.setCursor(20, 140); s_tft.print("WAV FILE...");
            s_o.upload = true;
        }
        return;
    }
    if (s_o.upload || s_full) { drawFrame(); s_full = false; }

    // 1. Timecode + play/pause icon
    const arc::Timecode tc = arc::engine::current();
    if (!s_o.tcValid || tc != s_o.tc || running != s_o.running || (int)mode != s_o.mode) {
        s_tft.fillRect(0, 0, 28, 24, ILI9341_BLACK);
        if (running) s_tft.fillTriangle(2, 2, 2, 22, 22, 12, ILI9341_GREEN);
        else { s_tft.fillRect(2, 3, 6, 18, 0x7BEF); s_tft.fillRect(12, 3, 6, 18, 0x7BEF); }
        s_tft.setTextSize(3);
        s_tft.setTextColor((running || mode == Mode::Slave) ? ILI9341_GREEN : 0x7BEF, ILI9341_BLACK);
        snprintf(buf, sizeof(buf), "%02u:%02u:%02u:%02u", tc.h, tc.m, tc.s, tc.f);
        s_tft.setCursor(20, 36); s_tft.print(buf);
        s_o.tc = tc; s_o.tcValid = true; s_o.running = running;
    }

    // 2. RTC time (top right)
    const DateTime now = arc::clock::now();
    if (now.minute() != s_o.rtcMin) {
        s_tft.setTextSize(2);
        s_tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
        snprintf(buf, sizeof(buf), "%02d:%02d", now.hour(), now.minute());
        s_tft.setCursor(176, 4); s_tft.print(buf);
        s_o.rtcMin = now.minute();
    }

    // 3. Mode line
    const arc::engine::JamStatus jam = arc::engine::jam();
    const bool countIn = audio::countInActive();
    const bool browse = s_menu == MENU_SHOT_BROWSE;
    if ((int)mode != s_o.mode || countIn != s_o.countIn || browse != s_o.browse) {
        s_tft.setTextSize(2);
        s_tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
        const char* label = countIn ? "COUNT IN..." : browse ? "BROWSE SHOT" : modeLabel(mode);
        snprintf(buf, sizeof(buf), "%-16s", label);
        s_tft.setCursor(10, 80); s_tft.print(buf);
        s_o.countIn = countIn; s_o.browse = browse;
    }

    // 4. Frame rate + user bits
    const bool decode = mode == Mode::Slave || mode == Mode::JamFree || mode == Mode::JamRec;
    const uint32_t ub = decode ? arc::engine::rxUserBits() : arc::engine::txUserBits();
    if ((int)mode != s_o.mode || (int)arc::engine::rate() != s_o.rate || ub != s_o.ub) {
        s_tft.setTextSize(2);
        s_tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        snprintf(buf, sizeof(buf), "%-7s U:%08lX ", arc::rateKey(arc::engine::rate()), (unsigned long)ub);
        s_tft.setCursor(10, 106); s_tft.print(buf);
        s_o.rate = (int)arc::engine::rate(); s_o.ub = ub;
    }

    // 5. Volume bar (auto hide) or shot data
    const bool volVisible = millis() < s_volShowUntil;
    const int vol = audio::volume();
    if (volVisible) {
        if (vol != s_o.vol) {
            s_tft.fillRect(0, 126, 240, 32, ILI9341_BLACK);
            s_tft.setTextSize(2);
            s_tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
            snprintf(buf, sizeof(buf), "%d%%", vol);
            s_tft.setCursor(100, 128); s_tft.print(buf);
            s_tft.drawRect(20, 146, 200, 12, ILI9341_WHITE);
            const int w = vol * 196 / 100;
            if (w > 0) s_tft.fillRect(22, 148, w, 8, vol <= 30 ? ILI9341_GREEN : vol <= 70 ? ILI9341_YELLOW : ILI9341_RED);
            s_o.vol = vol;
        }
    } else if (s_o.vol >= 0) {
        s_tft.fillRect(0, 126, 240, 32, ILI9341_BLACK);
        s_o.vol = -1;
    }

    const uint32_t rev = play::revision();
    if (rev != s_o.rev || (int)mode != s_o.mode) {
        play::lock();
        const PlayShot sh = play::shot();
        const PlaySong song = play::song();
        play::unlock();
        s_tft.fillRect(0, 166, 240, 84, ILI9341_BLACK);
        if (sh.loaded && sh.shot != "0" && sh.shot.length()) {
            const uint16_t bg = browse ? ILI9341_RED : ILI9341_BLACK;
            s_tft.fillRect(0, 166, 240, 24, bg);
            s_tft.setTextColor(ILI9341_WHITE, bg);
            s_tft.setTextSize(3);
            snprintf(buf, sizeof(buf), "%s %d %s %d", sh.roll.c_str(), sh.scene, sh.shot.c_str(), sh.take);
            s_tft.setCursor(10, 167); s_tft.print(buf);
            s_tft.setTextSize(2);
            s_tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
            if (mode == Mode::Playback && song.bpm > 0) {
                snprintf(buf, sizeof(buf), "%.0f BPM | %d/4", song.bpm, song.timeSignature);
                s_tft.setCursor(10, 194); s_tft.print(buf);
            }
            snprintf(buf, sizeof(buf), "Start:%s", sh.starttime.c_str());
            s_tft.setCursor(10, 213); s_tft.print(buf);
            snprintf(buf, sizeof(buf), "End  :%s", sh.endtime.c_str());
            s_tft.setCursor(10, 231); s_tft.print(buf);
        } else {
            s_tft.setTextSize(2);
            s_tft.setTextColor(ILI9341_DARKGREY, ILI9341_BLACK);
            s_tft.setCursor(10, 180); s_tft.print(arc::net::ssid());
            s_tft.setCursor(10, 210); s_tft.print(arc::net::ipString());
        }
        s_o.rev = rev;
    }

    // 6. Bottom: song progress / jam progress
    if (mode == Mode::Playback) {
        play::lock();
        const PlayShot sh = play::shot();
        const PlaySong song = play::song();
        play::unlock();
        if ((int)mode != s_o.mode) {
            s_tft.fillRect(0, 254, 240, 66, ILI9341_BLACK);
            s_tft.setTextSize(2);
            s_tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
            snprintf(buf, sizeof(buf), "Song: %-13.13s", song.title.length() ? song.title.c_str() : "-");
            s_tft.setCursor(10, 256); s_tft.print(buf);
            s_tft.drawRect(10, 276, 220, 18, ILI9341_WHITE);
        }
        const TcRate r = arc::engine::rate();
        const double pos = (double)arc::framesDurationNs(arc::tcToFrames(tc, r), r) / 1e9;
        const double total = song.durationSec > 0 ? song.durationSec : 1;
        double p = pos / total;
        if (p > 1) p = 1;
        const int w = (int)(p * 216);
        s_tft.fillRect(12, 278, w, 14, ILI9341_RED);
        if (w < 216) s_tft.fillRect(12 + w, 278, 216 - w, 14, ILI9341_BLACK);
        arc::Timecode st, en;
        if (arc::parseTc(sh.starttime.c_str(), st) && arc::parseTc(sh.endtime.c_str(), en)) {
            const double a = (double)arc::framesDurationNs(arc::tcToFrames(st, r), r) / 1e9 / total;
            const double b = (double)arc::framesDurationNs(arc::tcToFrames(en, r), r) / 1e9 / total;
            const int x0 = (int)(a * 216), x1 = (int)(b > 1 ? 216 : b * 216);
            if (x1 > x0) s_tft.drawRect(10 + x0, 276, x1 - x0, 18, ILI9341_YELLOW);
        }
        s_tft.setTextSize(2);
        s_tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        snprintf(buf, sizeof(buf), "%02d:%02d", (int)pos / 60, (int)pos % 60);
        s_tft.setCursor(10, 298); s_tft.print(buf);
        snprintf(buf, sizeof(buf), "%02d:%02d", (int)total / 60, (int)total % 60);
        s_tft.setCursor(170, 298); s_tft.print(buf);
    } else if (mode == Mode::JamFree || mode == Mode::JamRec || jam.muteHeld) {
        const int key = (int)jam.state * 1000 + (int)jam.okFrames + (jam.muteHeld ? 100000 : 0) + (int)jam.verifyRemainingS * 7;
        if (key != s_o.jam || (int)mode != s_o.mode) {
            s_tft.fillRect(0, 254, 240, 66, ILI9341_BLACK);
            s_tft.setTextSize(2);
            s_tft.setCursor(10, 258);
            using S = arc::engine::JamStatus::State;
            if (jam.muteHeld) {
                s_tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
                s_tft.print("KABEL ABZIEHEN,");
                s_tft.setCursor(10, 280); s_tft.print("DANN DRUECKEN");
                snprintf(buf, sizeof(buf), "Rest %ldus %.2fppm", (long)jam.residualUs, jam.ppm);
                s_tft.setTextSize(1);
                s_tft.setCursor(10, 304); s_tft.print(buf);
            } else if (jam.state == S::Verifying) {
                s_tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
                snprintf(buf, sizeof(buf), "JAM: PRUEFE %lus", (unsigned long)jam.verifyRemainingS);
                s_tft.print(buf);
            } else {
                s_tft.setTextColor(jam.okFrames ? ILI9341_GREEN : ILI9341_RED, ILI9341_BLACK);
                s_tft.print(jam.okFrames ? "JAM: SIGNAL OK " : "JAM: NO SIGNAL ");
                s_tft.drawRect(10, 286, 220, 20, ILI9341_WHITE);
                int w = (int)(jam.okFrames * 216 / 75);
                if (w > 216) w = 216;
                s_tft.fillRect(12, 288, w, 16, ILI9341_GREEN);
            }
            s_o.jam = key;
        }
    } else if ((int)mode != s_o.mode) {
        s_tft.fillRect(0, 254, 240, 66, ILI9341_BLACK);
    }
    s_o.mode = (int)mode;
}

// ── actions ─────────────────────────────────────────────────────────────
void togglePlay() {
    if (arc::engine::jam().muteHeld) { arc::engine::confirmUnplug(); return; }
    const Mode mode = arc::engine::mode();
    if (mode == Mode::Playback) {
        if (audio::playing()) audio::stop(0); else audio::start(0);
    } else if (mode == Mode::RecordRun) {
        if (arc::engine::running()) arc::engine::stop(); else arc::engine::start();
    }
    arc::net::broadcastStatus();
}

void formatSd() {
    File root = SD_MMC.open("/");
    for (File e = root.openNextFile(); e; e = root.openNextFile()) {
        const String name = String("/") + e.name();
        e.close();
        SD_MMC.remove(name.c_str());
    }
    root.close();
    Serial.println("[PLAY] SD-Karte geleert");
}

void menuPress() {
    switch (s_menu) {
        case MENU_SETTINGS:
            switch (s_sel) {
                case 0: s_menu = MENU_MODE; s_sel = 0; drawMenuScreen(); return;
                case 1: s_menu = MENU_FPS; s_sel = 0; drawMenuScreen(); return;
                case 2: arc::net::requestNtpSync(); return;
                case 3: s_menu = MENU_FORMAT_SD; s_sel = 0; drawMenuScreen(); return;
                default: s_menu = MENU_CLOSED; return;
            }
        case MENU_MODE:
            if (s_sel < 6) { arc::engine::requestMode(kModeValues[s_sel]); s_menu = MENU_CLOSED; }
            else { s_menu = MENU_SETTINGS; s_sel = 0; drawMenuScreen(); }
            return;
        case MENU_FPS:
            if (s_sel < 4) { arc::engine::setRate(kFpsValues[s_sel]); s_menu = MENU_CLOSED; }
            else { s_menu = MENU_SETTINGS; s_sel = 0; drawMenuScreen(); }
            return;
        case MENU_FORMAT_SD:
            if (s_sel == 0) formatSd();
            s_menu = s_sel == 0 ? MENU_CLOSED : MENU_SETTINGS;
            s_sel = 0;
            if (s_menu == MENU_SETTINGS) drawMenuScreen();
            return;
        default: return;
    }
}

void uiTask(void*) {
    bool lastBtn = true;
    uint32_t pressMs = 0, lastReleaseMs = 0;
    bool longDone = false, clickPending = false;
    for (;;) {
        int32_t steps;
        portENTER_CRITICAL(&s_encMux);
        steps = s_encSteps;
        s_encSteps = 0;
        portEXIT_CRITICAL(&s_encMux);
        if (steps) {
            if (s_menu == MENU_CLOSED) {
                audio::setVolume(audio::volume() + steps * 2);
                s_volShowUntil = millis() + 1500;
            } else if (s_menu == MENU_SHOT_BROWSE) {
                playlink::requestShotStep(steps > 0 ? 1 : -1);
            } else {
                const char* const* items; int count;
                menuItems(items, count);
                const int old = s_sel;
                s_sel = ((s_sel + steps) % count + count) % count;
                drawMenuItem(old, false);
                drawMenuItem(s_sel, true);
            }
        }

        const bool btn = digitalRead(PIN_ENC_SW);   // LOW = pressed
        const uint32_t now = millis();
        if (lastBtn && !btn && now - pressMs > 80) { pressMs = now; longDone = false; }
        if (!btn && !longDone && s_menu == MENU_CLOSED && now - pressMs >= 1500) {
            longDone = true;
            clickPending = false;
            s_menu = MENU_SETTINGS;
            s_sel = 0;
            drawMenuScreen();
        }
        if (!lastBtn && btn && !longDone) {
            if (s_menu == MENU_CLOSED) {
                if (now - lastReleaseMs < 400) {   // double click: browse shots
                    clickPending = false;
                    s_menu = MENU_SHOT_BROWSE;
                    playlink::requestShotStep(1);
                } else {
                    clickPending = true;
                }
                lastReleaseMs = now;
            } else if (s_menu == MENU_SHOT_BROWSE) {
                s_menu = MENU_CLOSED;
                s_full = true;
            } else {
                menuPress();
                if (s_menu == MENU_CLOSED) s_full = true;
            }
        }
        lastBtn = btn;
        if (clickPending && now - lastReleaseMs >= 400) {
            clickPending = false;
            if (s_menu == MENU_CLOSED) togglePlay();
        }

        if (s_menu == MENU_CLOSED || s_menu == MENU_SHOT_BROWSE) drawStatus();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(15));
    }
}

}  // namespace

namespace playui {

void begin() {
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, HIGH);
    pinMode(PIN_TFT_CS, OUTPUT);
    digitalWrite(PIN_TFT_CS, HIGH);
    SPI.begin(PIN_TFT_SCLK, PIN_TFT_MISO, PIN_TFT_MOSI);
    s_tft.begin(40000000UL);
    s_tft.setRotation(2);
    s_tft.invertDisplay(false);
    drawFrame();

    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT, INPUT_PULLUP);
    pinMode(PIN_ENC_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), isrEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT), isrEncoder, CHANGE);
    xTaskCreatePinnedToCore(uiTask, "PLAYUI", 8192, nullptr, 1, &s_task, 0);
}

void requestRedraw() { if (s_task) xTaskNotifyGive(s_task); }

}  // namespace playui
