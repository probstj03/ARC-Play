/*
 * ARC-Play – playback device on the ARC_AP board (ESP32-S3).
 *
 * ARC-Core (shared with ARC-Slate): RTC wall clock, phase-locked LTC generator,
 * decoder, Jam Sync, WiFi/ARC-Net, protocol v2, event log.
 *
 * Play modules
 *   play_audio  song playback with count-in, sample-exact scheduled start/stop
 *   play_ui     TFT + rotary encoder
 *   play_link   Play protocol (shot, song, upload, take events, user bits)
 *   play_state  shot + song state
 *
 * Pins: pins.h · Wiring: hardware/wiring/ARC-Play_Verdrahtung.svg
 * Serial console: 115200 baud, "help" lists the commands.
 */
#include <Arduino.h>
#include <ArcCore.h>
#include <Preferences.h>
#include <Wire.h>
#include "pins.h"
#include "play_audio.h"
#include "play_link.h"
#include "play_state.h"
#include "play_ui.h"

using arc::engine::Mode;
using arc::TcRate;

namespace {

volatile bool s_statusChanged = false;
void onEngineStatus() { s_statusChanged = true; }

void onJam(const arc::engine::JamStatus& j) {
    JsonDocument ev;
    ev["type"] = "event";
    ev["kind"] = "jam";
    ev["device"] = "play";
    ev["deviceId"] = arc::net::deviceId();
    ev["id"] = arc::events::nextId();
    ev["rtcUs"] = arc::clock::nowUs();
    ev["ok"] = j.state == arc::engine::JamStatus::State::Done;
    ev["residualUs"] = j.residualUs;
    ev["ppm"] = j.ppm;
    ev["aging"] = j.aging;
    arc::net::send(ev);
    arc::events::append(ev);
}

// SQW edge + frame rate are kept in NVS (namespace "play").
bool loadSqwEdge(bool& falling) {
    Preferences p;
    p.begin("play", true);
    const bool known = p.getBool("sqwKnown", false);
    falling = p.getBool("sqwFall", true);
    p.end();
    return known;
}

void saveSqwEdge(bool falling) {
    Preferences p;
    p.begin("play", false);
    p.putBool("sqwKnown", true);
    p.putBool("sqwFall", falling);
    p.end();
}

void detectSqwEdge() {
    bool falling = true;
    int32_t offsetMs = 0;
    Serial.println("[CLK] Messe, welche SQW-Flanke den Sekundenwechsel markiert ...");
    if (!arc::clock::detectSqwEdge(falling, offsetMs)) {
        Serial.println("[CLK] Messung fehlgeschlagen (RTC/SQW prüfen)");
        return;
    }
    Serial.printf("[CLK] Sekundenwechsel = %s Flanke (Abstand %ld ms)\n", falling ? "fallende" : "steigende", (long)offsetMs);
    saveSqwEdge(falling);
    arc::clock::useEdge(falling);
}

void printHelp() {
    Serial.println(
        "\n=== ARC-Play " ARC_FW_VERSION " – serielle Befehle ===\n"
        "  ?                Status\n"
        "  1 2 3 4 5 6      Modus: Free Run, Record Run, Playback, Slave, Jam->Free, Jam->Rec\n"
        "  q w e r          Bildrate: 30, 25, 29.97 DF, 24\n"
        "  s / x            Start-Stop (Playback mit Vorzähler / Record Run) / Stop\n"
        "  t | tc HH:MM:SS:FF   TC auf 0 / setzen (Record Run)\n"
        "  u                Jam: Kabel abgezogen bestaetigen\n"
        "  n                NTP-Sync erzwingen\n"
        "  time YYYY-MM-DD HH:MM:SS   RTC stellen\n"
        "  sqw              SQW-Flanke neu messen\n"
        "  vol 0-100        Lautstaerke\n"
        "  wifi | wifi add SSID PASS [PRIO] | wifi clear\n"
        "  log | log clear  Ereignis-Log\n");
}

void printStatus() {
    const arc::Timecode tc = arc::engine::current();
    char tcs[12];
    arc::formatTc(tc, tcs);
    const int64_t now = arc::clock::nowNs();
    const DateTime d((uint32_t)(now / 1000000000LL));
    const arc::engine::JamStatus j = arc::engine::jam();
    play::lock();
    const PlayShot sh = play::shot();
    const PlaySong song = play::song();
    play::unlock();
    Serial.printf("TC %s  Modus %s  %s fps  %s\n", tcs, arc::engine::modeKey(arc::engine::mode()),
                  arc::rateKey(arc::engine::rate()), arc::engine::running() ? "läuft" : "steht");
    Serial.printf("RTC %04d-%02d-%02d %02d:%02d:%02d.%06lu  SQW %s  Trim %lld us  %.2f °C  Aging %d\n", d.year(),
                  d.month(), d.day(), d.hour(), d.minute(), d.second(), (unsigned long)((now / 1000) % 1000000),
                  arc::clock::sqwActive() ? "ok" : "FEHLT", (long long)(arc::clock::trimNs() / 1000),
                  arc::clock::temperature(), (int)arc::clock::agingOffset());
    Serial.printf("Ausgang: Latenz %.0f us %s, aktuell %.0f us, Phasenfehler %.1f us, Slips %lu\n",
                  arc::engine::outputLatencyUs(), arc::engine::latencyCalibrated() ? "(kalibriert)" : "(Standard)",
                  arc::engine::generator().latencyUs(), arc::engine::phaseErrorUs(),
                  (unsigned long)arc::engine::generator().slips());
    static const char* const js[] = {"idle", "hoert zu", "prueft", "fertig", "fehlgeschlagen"};
    Serial.printf("Jam: %s, ok %lu, Rest %ld us, Drift %.2f ppm%s\n", js[(int)j.state], (unsigned long)j.okFrames,
                  (long)j.residualUs, j.ppm, j.muteHeld ? "  [Ausgang stumm bis Kabel ab]" : "");
    Serial.printf("Shot: %s Szene %d Shot %s Take %d  %s – %s  Tempo %.2f\n", sh.roll.c_str(), sh.scene, sh.shot.c_str(),
                  sh.take, sh.starttime.c_str(), sh.endtime.c_str(), sh.speed);
    Serial.printf("Song: %s  %.1f s  %.1f BPM  Lautstärke %d%%  %s\n", song.title.c_str(), song.durationSec, song.bpm,
                  audio::volume(), audio::playing() ? "spielt" : "steht");
    Serial.printf("Netz: %s  Heap %lu\n", arc::net::statusLine().c_str(), (unsigned long)ESP.getFreeHeap());
}

void handleSerial() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;
    const int sp = line.indexOf(' ');
    const String cmd = sp < 0 ? line : line.substring(0, sp);
    const String arg = sp < 0 ? String() : line.substring(sp + 1);

    if (cmd == "?") printStatus();
    else if (cmd == "help") printHelp();
    else if (cmd == "1") arc::engine::requestMode(Mode::FreeRun);
    else if (cmd == "2") arc::engine::requestMode(Mode::RecordRun);
    else if (cmd == "3") arc::engine::requestMode(Mode::Playback);
    else if (cmd == "4") arc::engine::requestMode(Mode::Slave);
    else if (cmd == "5") arc::engine::requestMode(Mode::JamFree);
    else if (cmd == "6") arc::engine::requestMode(Mode::JamRec);
    else if (cmd == "q") arc::engine::setRate(TcRate::R30);
    else if (cmd == "w") arc::engine::setRate(TcRate::R25);
    else if (cmd == "e") arc::engine::setRate(TcRate::R2997DF);
    else if (cmd == "r") arc::engine::setRate(TcRate::R24);
    else if (cmd == "s") {
        if (arc::engine::mode() == Mode::Playback) { if (audio::playing()) audio::stop(0); else audio::start(0); }
        else if (arc::engine::running()) arc::engine::stop();
        else arc::engine::start();
    }
    else if (cmd == "x") { if (arc::engine::mode() == Mode::Playback) audio::stop(0); else arc::engine::stop(); }
    else if (cmd == "t") arc::engine::setTimecode(arc::Timecode());
    else if (cmd == "tc") { arc::Timecode tc; if (arc::parseTc(arg.c_str(), tc)) arc::engine::setTimecode(tc); }
    else if (cmd == "u") arc::engine::confirmUnplug();
    else if (cmd == "n") arc::net::requestNtpSync();
    else if (cmd == "time") {
        int y, mo, da, h, mi, se;
        if (sscanf(arg.c_str(), "%d-%d-%d %d:%d:%d", &y, &mo, &da, &h, &mi, &se) == 6) arc::clock::setTime(DateTime(y, mo, da, h, mi, se));
    }
    else if (cmd == "sqw") detectSqwEdge();
    else if (cmd == "vol") audio::setVolume(arg.toInt());
    else if (cmd == "wifi") {
        if (arg.startsWith("add ")) {
            const String rest = arg.substring(4);
            const int a = rest.indexOf(' ');
            String pass = rest.substring(a + 1);
            int prio = 10;
            const int b = pass.lastIndexOf(' ');
            if (b > 0 && pass.substring(b + 1).toInt() > 0) { prio = pass.substring(b + 1).toInt(); pass = pass.substring(0, b); }
            arc::net::addNetwork(rest.substring(0, a), pass, prio);
        } else if (arg == "clear") arc::net::clearNetworks();
        arc::net::printNetworks(Serial);
    }
    else if (cmd == "log") { if (arg == "clear") arc::events::clear(); else arc::events::print(Serial); }
    else Serial.printf("Unbekannter Befehl '%s' – 'help' zeigt alle Befehle\n", line.c_str());
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\nARC-Play " ARC_FW_VERSION " startet ...");

    play::begin();
    arc::events::begin();
    playui::begin();

    bool falling = true;
    const bool known = loadSqwEdge(falling);
    arc::clock::begin({PIN_RTC_SDA, PIN_RTC_SCL, PIN_RTC_SQW}, falling);
    if (arc::clock::rtcPresent() && !known) detectSqwEdge();

    arc::engine::Config ec;
    ec.dacBck = PIN_DAC_BCK;
    ec.dacLrck = PIN_DAC_LRCK;
    ec.dacDin = PIN_DAC_DIN;
    ec.dacMute = PIN_PCM_XSMT;
    ec.ltcIn = PIN_LTC_IN;
    ec.ltcChannels = 2;   // LTC only on the right channel (J2); left = song (J3)
    arc::engine::onStatusChange(onEngineStatus);
    arc::engine::onJamEvent(onJam);
    arc::engine::begin(ec, TcRate::R25);

    audio::begin();
    playlink::begin();
    printHelp();
}

void loop() {
    arc::engine::loop();
    arc::net::loop();
    playlink::loop();
    if (s_statusChanged) {
        s_statusChanged = false;
        playlink::onModeChanged();
        arc::net::broadcastStatus();
        playui::requestRedraw();
    }
    handleSerial();
    vTaskDelay(pdMS_TO_TICKS(5));
}
