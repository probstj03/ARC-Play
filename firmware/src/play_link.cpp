#include "play_link.h"
#include <ArcCore.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SD_MMC.h>
#include "play_audio.h"
#include "play_state.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

#ifndef ARC_FW_VERSION
#define ARC_FW_VERSION "dev"
#endif

using arc::engine::Mode;

namespace {

const char* const kCaps[] = {"ltc_gen", "ltc_jam", "playback", "countin", "upload", "tft", "encoder", "schedule"};
constexpr const char* SONGINFO = "/songinfo.json";

int64_t atNs(JsonDocument& doc) {
    if (doc["at"].isNull()) return 0;
    return (int64_t)doc["at"].as<double>() * 1000LL;
}

int toInt(JsonVariantConst v, int fallback) {
    if (v.is<int>()) return v.as<int>();
    if (v.is<const char*>()) return String(v.as<const char*>()).toInt();
    return fallback;
}

float tcStringToSeconds(const String& s, float fps) {
    arc::Timecode tc;
    if (!arc::parseTc(s.c_str(), tc)) return 0;
    return tc.h * 3600.0f + tc.m * 60.0f + tc.s + tc.f / (fps > 0 ? fps : 25.0f);
}

void saveSongInfo() {
    play::lock();
    const PlaySong s = play::song();
    play::unlock();
    JsonDocument doc;
    doc["songtitle"] = s.title;
    doc["songDurationSec"] = s.durationSec;
    doc["songBpm"] = s.bpm;
    doc["songTimeSignature"] = s.timeSignature;
    if (SD_MMC.exists(SONGINFO)) SD_MMC.remove(SONGINFO);
    File f = SD_MMC.open(SONGINFO, FILE_WRITE);
    if (f) { serializeJson(doc, f); f.close(); }
}

void loadSongInfo() {
    File f = SD_MMC.open(SONGINFO, FILE_READ);
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        play::lock();
        PlaySong& s = play::song();
        s.title = doc["songtitle"] | "";
        s.durationSec = doc["songDurationSec"] | 0.0f;
        s.bpm = doc["songBpm"] | 0.0f;
        s.timeSignature = doc["songTimeSignature"] | 4;
        play::touch();
        play::unlock();
    }
    f.close();
}

void handleShot(JsonDocument& doc) {
    play::lock();
    PlayShot& s = play::shot();
    PlaySong& song = play::song();
    if (!doc["scene"].isNull()) s.scene = toInt(doc["scene"], s.scene);
    if (!doc["shot"].isNull()) s.shot = doc["shot"].as<String>();
    if (!doc["take"].isNull()) s.take = toInt(doc["take"], s.take);
    if (!doc["roll"].isNull()) s.roll = doc["roll"].as<String>();
    if (!doc["camera"].isNull()) s.camera = doc["camera"].as<String>();
    if (!doc["starttime"].isNull()) s.starttime = doc["starttime"].as<String>();
    if (!doc["endtime"].isNull()) s.endtime = doc["endtime"].as<String>();
    if (!doc["duration"].isNull()) song.durationSec = doc["duration"].as<float>();
    if (!doc["songtitle"].isNull()) song.title = doc["songtitle"].as<String>();
    float speed = doc["speed"].isNull() ? s.speed : doc["speed"].as<float>();
    if (speed <= 0.0f || speed > 4.0f) speed = 1.0f;
    s.speed = speed;
    double fps = 0;
    if (doc["framerate"].is<const char*>()) fps = String(doc["framerate"].as<const char*>()).toFloat();
    else if (!doc["framerate"].isNull()) fps = doc["framerate"].as<double>();
    if (fps > 0) s.fps = (float)fps;
    s.loaded = true;
    play::touch();
    play::unlock();
    arc::TcRate r;
    if (arc::rateFromFps(fps, r)) arc::engine::setRate(r);
    playlink::updateUserBits();
    audio::prepareShot();
}

void handleSongInfo(JsonDocument& doc) {
    play::lock();
    PlaySong& s = play::song();
    const float fps = play::shot().fps;
    if (!doc["songtitle"].isNull()) s.title = doc["songtitle"].as<String>();
    if (!doc["songduration"].isNull()) s.durationSec = tcStringToSeconds(doc["songduration"].as<String>(), fps);
    if (!doc["songBpm"].isNull()) s.bpm = doc["songBpm"].as<float>();
    if (!doc["songTimeSignature"].isNull() && doc["songTimeSignature"].as<int>() > 0) s.timeSignature = doc["songTimeSignature"].as<int>();
    play::touch();
    play::unlock();
    saveSongInfo();
}

bool onMessage(const String& type, JsonDocument& doc, AsyncWebSocketClient* client) {
    const bool playback = arc::engine::mode() == Mode::Playback;
    if (type == "shot") { handleShot(doc); return true; }
    if (type == "song_info") { handleSongInfo(doc); return true; }
    if (type == "prepare_upload") {
        if (audio::uploadBegin()) client->text("{\"type\":\"upload_prepared\"}");
        return true;
    }
    if (type == "finish_upload") {
        audio::uploadFinish();
        play::lock();
        if (!doc["duration"].isNull()) play::song().durationSec = doc["duration"].as<float>();
        if (!doc["songtitle"].isNull()) play::song().title = doc["songtitle"].as<String>();
        play::touch();
        play::unlock();
        saveSongInfo();
        client->text("{\"type\":\"upload_ack\"}");
        return true;
    }
    if (type == "command") {
        const String cmd = doc["cmd"] | "";
        if (cmd == "set_volume") { audio::setVolume((int)doc["val"].as<float>()); arc::net::broadcastStatus(); return true; }
        if (!playback) {
            if (cmd == "seek") {
                const double secs = doc["time"].as<double>();
                const arc::TcRate r = arc::engine::rate();
                arc::engine::setTimecode(arc::framesToTc((uint32_t)arc::framesIn((int64_t)(secs * 1e9), r, nullptr), r));
                return true;
            }
            return false;   // generic transport in the other modes: ARC-Core
        }
        if (cmd == "start" || cmd == "play") { audio::start(atNs(doc)); return true; }
        if (cmd == "stop" || cmd == "pause") { audio::stop(atNs(doc)); return true; }
        if (cmd == "seek") { audio::seekSongSeconds(doc["time"].as<double>()); return true; }
        if (cmd == "reset") { audio::seekSongSeconds(0); return true; }
        return false;
    }
    return type == "status";   // other devices' status: ignored
}

void onBinary(AsyncWebSocketClient*, const uint8_t* data, size_t len) { audio::uploadData(data, len); }

void fillStatus(JsonDocument& doc) {
    play::lock();
    const PlayShot s = play::shot();
    const PlaySong song = play::song();
    play::unlock();
    doc["volume"] = audio::volume();
    doc["scene"] = s.scene;
    doc["shot"] = s.shot;
    doc["take"] = s.take;
    doc["roll"] = s.roll;
    doc["songtitle"] = song.title;
    doc["countIn"] = audio::countInActive();
    if (arc::engine::mode() == Mode::Playback) doc["running"] = audio::playing();
}

// Audio task: the take reached the shot end.
void onTakeEnd(const arc::Timecode& endTc, int64_t endWallNs) {
    play::lock();
    PlayShot& s = play::shot();
    const int take = s.take;
    const String roll = s.roll;
    const int scene = s.scene;
    const String shot = s.shot;
    s.take++;
    play::touch();
    play::unlock();
    char tcs[12], ub[9], date[12];
    arc::formatTc(endTc, tcs);
    snprintf(ub, sizeof(ub), "%08lX", (unsigned long)arc::engine::txUserBits());
    const DateTime d((uint32_t)(endWallNs / 1000000000LL));
    snprintf(date, sizeof(date), "%02d.%02d.%04d", d.day(), d.month(), d.year());

    JsonDocument ev;
    ev["type"] = "event";
    ev["kind"] = "take_end";
    ev["device"] = "play";
    ev["deviceId"] = arc::net::deviceId();
    ev["id"] = arc::events::nextId();
    ev["rtcUs"] = endWallNs / 1000;
    ev["tc"] = tcs;
    ev["fps"] = arc::rateKey(arc::engine::rate());
    ev["scene"] = scene;
    ev["shot"] = shot;
    ev["take"] = take;
    ev["roll"] = roll;
    ev["userbit"] = ub;
    arc::net::send(ev);
    arc::events::append(ev);

    // Legacy message understood by the current ARC-Studio.
    JsonDocument legacy;
    legacy["type"] = "newTake";
    legacy["take"] = take;
    legacy["timecode"] = tcs;
    legacy["roll"] = roll;
    legacy["userbit"] = ub;
    legacy["date"] = date;
    arc::net::send(legacy);

    Serial.printf("[PLAY] Take %d fertig bei %s\n", take, tcs);
    playlink::updateUserBits();
    audio::prepareShot();
    arc::net::broadcastStatus();
}

}  // namespace

namespace playlink {

void begin() {
    loadSongInfo();
    audio::onTakeEnd(onTakeEnd);
    arc::net::Config cfg;
    cfg.hostname = "arc-play";
    cfg.deviceType = "playback";
    cfg.device = "play";
    cfg.firmware = ARC_FW_VERSION;
    cfg.caps = kCaps;
    cfg.capCount = sizeof(kCaps) / sizeof(kCaps[0]);
    cfg.defaults = kDefaultWifi;
    cfg.defaultCount = sizeof(kDefaultWifi) / sizeof(kDefaultWifi[0]);
    cfg.arcNetPassword = kArcNetPassword;
    cfg.onMessage = onMessage;
    cfg.fillStatus = fillStatus;
    cfg.onBinary = onBinary;
    arc::net::begin(cfg);
    updateUserBits();
}

void loop() {
    static uint32_t lastMs = 0;
    if (millis() - lastMs > 60000) {   // date user bits change at midnight
        lastMs = millis();
        updateUserBits();
    }
}

namespace {
struct TakeScan { String scene, shot; int maxTake; };

bool scanTakes(JsonDocument& ev, void* ctx) {
    TakeScan& t = *(TakeScan*)ctx;
    if (String(ev["kind"] | "") != "take_end") return true;
    if (String(ev["scene"] | 0) == t.scene && String(ev["shot"] | "") == t.shot) {
        const int take = ev["take"] | 0;
        if (take > t.maxTake) t.maxTake = take;
    }
    return true;
}

// Without ARC-Studio: step through the offline project stored on the device.
void stepShotOffline(int dir) {
    const int count = arc::project::shotCount();
    if (count == 0) return;
    play::lock();
    const String scene = String(play::shot().scene);
    const String shot = play::shot().shot;
    play::unlock();
    int idx = arc::project::indexOf(scene, shot);
    idx = idx < 0 ? 0 : ((idx + dir) % count + count) % count;
    JsonDocument msg;
    if (!arc::project::shotMessage(idx, msg)) return;
    TakeScan scan{String(msg["scene"].is<const char*>() ? String(msg["scene"].as<const char*>()).toInt() : msg["scene"].as<int>()),
                  msg["shot"].as<String>(), 0};
    arc::events::forEachAfter(0, scanTakes, &scan);
    msg["take"] = scan.maxTake + 1;
    handleShot(msg);
}
}  // namespace

void requestShotStep(int dir) {
    if (!arc::net::studioConnected()) { stepShotOffline(dir); return; }
    JsonDocument doc;
    doc["type"] = dir > 0 ? "nextShot" : "previousShot";
    doc["device"] = "play";
    arc::net::send(doc);
}

void updateUserBits() {
    play::lock();
    const PlayShot s = play::shot();
    play::unlock();
    uint32_t bits;
    if (s.loaded) {
        char cam = 'A';
        int rollNum = 1;
        for (unsigned i = 0; i < s.roll.length(); i++) {
            const char c = (char)toupper((unsigned char)s.roll[i]);
            if (c >= 'A' && c <= 'Z') { cam = c; break; }
        }
        for (unsigned i = 0; i < s.roll.length(); i++) {
            if (isdigit((unsigned char)s.roll[i])) { rollNum = s.roll.substring(i).toInt(); break; }
        }
        bits = arc::shotUserBits(cam, rollNum, s.scene, s.shot.toInt(), s.take);
    } else {
        const DateTime d = arc::clock::now();
        bits = arc::dateUserBits(d.day(), d.month(), d.year(), arc::isEuSummerTime(d.year(), d.month(), d.day(), d.hour()));
    }
    arc::engine::setUserBits(bits);
}

void onModeChanged() {
    const bool wantPlayback = arc::engine::mode() == Mode::Playback;
    if (wantPlayback && !audio::inPlayback()) audio::enterPlayback();
    if (!wantPlayback && audio::inPlayback()) audio::leavePlayback();
}

}  // namespace playlink
