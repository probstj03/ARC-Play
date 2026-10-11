#include "play_audio.h"
#include <SD_MMC.h>
#include <esp_timer.h>
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecWAV.h"
#include "AudioTools/Disk/AudioSourceSDMMC.h"
#include "pins.h"
#include "play_state.h"

namespace {

using arc::TcRate;
using arc::Timeline;

enum class St : uint8_t { Off, Warmup, Idle, Armed, Playing };
enum class CmdType : uint8_t { Enter, Leave, Start, Stop, Seek, Prepare };
struct Cmd {
    CmdType type;
    int64_t atNs;
    double sec;
};

constexpr uint64_t NO_INDEX = UINT64_MAX;
constexpr int BLOCK = 256;
constexpr const char* TRACK = "/track.wav";

volatile St s_state = St::Off;
QueueHandle_t s_cmdQ = nullptr;
SemaphoreHandle_t s_sdMtx = nullptr;

// Output session
uint64_t s_written = 0;            // frames written to I2S since the session started
double s_fsOut = 44100.0;
int s_fsWav = 44100;
int s_wavChannels = 1;
float s_speed = 1.0f;
uint32_t s_wavDataOffset = 44;
bool s_haveSong = false;

// Scheduled transport
uint64_t s_startIndex = NO_INDEX;
uint64_t s_stopIndex = NO_INDEX;
bool s_stopIsShotEnd = false;
uint32_t s_endFrame = 0;
uint32_t s_shotStartFrame = 0;
TcRate s_rate = TcRate::R25;

// Count-in (8 beats, accent on 1 and 5)
volatile bool s_countIn = false;
uint32_t s_ciElapsed = 0, s_ciTotal = 0, s_ciPerBeat = 0;
float s_ciPhase = 0;

volatile int s_volume = 50;
volatile bool s_uploading = false;
File s_uploadFile;
void (*s_takeEndCb)(const arc::Timecode&, int64_t) = nullptr;

// ── helpers ─────────────────────────────────────────────────────────────
double frameSongSec(uint32_t frame, TcRate r) { return (double)arc::framesDurationNs(frame, r) / 1e9; }

int64_t wallOfSample(uint64_t idx) {
    const double esp = arc::engine::outputClock().espUsOfSample(idx) + arc::engine::generator().latencyUs();
    return arc::clock::wallNsAtEsp((int64_t)esp);
}

bool parseTcFrames(const String& s, TcRate r, uint32_t& out) {
    arc::Timecode tc;
    if (!arc::parseTc(s.c_str(), tc)) return false;
    out = arc::tcToFrames(tc, r);
    return true;
}

void scanWavDataOffset() {
    s_wavDataOffset = 44;
    File f = SD_MMC.open(TRACK, FILE_READ);
    if (!f) return;
    uint8_t hdr[12];
    if (f.read(hdr, 12) == 12 && !memcmp(hdr, "RIFF", 4) && !memcmp(hdr + 8, "WAVE", 4)) {
        uint32_t pos = 12;
        for (int guard = 0; guard < 64; guard++) {   // BWF headers can be long (bext, iXML)
            uint8_t ch[8];
            f.seek(pos, SeekSet);
            if (f.read(ch, 8) != 8) break;
            const uint32_t size = (uint32_t)ch[4] | ((uint32_t)ch[5] << 8) | ((uint32_t)ch[6] << 16) | ((uint32_t)ch[7] << 24);
            if (!memcmp(ch, "data", 4)) { s_wavDataOffset = pos + 8; break; }
            pos += 8 + size + (size & 1);
        }
    }
    f.close();
}

// ── count-in click track ────────────────────────────────────────────────
int16_t nextCountInSample() {
    if (!s_countIn) return 0;
    const uint32_t el = s_ciElapsed++;
    if (el >= s_ciTotal) { s_countIn = false; return 0; }
    const uint32_t posInBeat = el % s_ciPerBeat;
    if (posInBeat == 0) s_ciPhase = 0.0f;
    if (posInBeat >= s_ciPerBeat * 55u / 100u) return 0;          // tone in the first 55 % of a beat
    const uint32_t beat = el / s_ciPerBeat;
    const float freq = (beat == 0 || beat == 4) ? 1760.0f : 880.0f;  // accent on 1 and 5
    s_ciPhase += 2.0f * (float)M_PI * freq / (float)s_fsWav;
    if (s_ciPhase >= 2.0f * (float)M_PI) s_ciPhase -= 2.0f * (float)M_PI;
    return (int16_t)(sinf(s_ciPhase) * 23000.0f);
}

// ── output ──────────────────────────────────────────────────────────────
audio_tools::I2SStream s_i2s;

void writeFrames(const int16_t* stereo, int frames) {
    size_t total = (size_t)frames * 4, done = 0;
    while (done < total) {
        const size_t w = s_i2s.write((const uint8_t*)stereo + done, total - done);
        if (w > 0) done += w; else vTaskDelay(1);
    }
    s_written += frames;
    arc::engine::outputClock().onWrite(s_written, esp_timer_get_time());
}

// Silence on the song channel, LTC on the right.
void pumpSilence(int frames) {
    static int16_t mono[BLOCK], stereo[2 * BLOCK];
    arc::engine::generator().render(mono, frames, s_written, arc::engine::outputClock());
    for (int i = 0; i < frames; i++) { stereo[2 * i] = 0; stereo[2 * i + 1] = mono[i]; }
    writeFrames(stereo, frames);
}

void finishTake() {
    const uint32_t endFrame = s_endFrame;
    const int64_t endWall = wallOfSample(s_stopIndex);
    s_state = St::Idle;
    s_stopIndex = NO_INDEX;
    s_countIn = false;
    if (s_stopIsShotEnd && s_takeEndCb) s_takeEndCb(arc::framesToTc(endFrame, s_rate), endWall);
    s_stopIsShotEnd = false;
}

// AudioPlayer output: mixes song + count-in (left) with LTC (right).
class Mixer : public audio_tools::AudioOutput {
public:
    void setAudioInfo(audio_tools::AudioInfo info) override {
        _info = info;
        s_wavChannels = info.channels > 0 ? info.channels : 1;
        if (info.sample_rate > 0) s_fsWav = info.sample_rate;
    }
    size_t write(const uint8_t* data, size_t len) override {
        if (s_state != St::Playing || _info.bits_per_sample != 16) return len;   // warm-up: discard
        const int ch = s_wavChannels;
        const int16_t* in = (const int16_t*)data;
        int frames = (int)(len / (2 * ch));
        int pos = 0;
        static int16_t mono[BLOCK], stereo[2 * BLOCK];
        const float vol = s_volume / 100.0f;
        while (pos < frames) {
            int n = frames - pos;
            if (n > BLOCK) n = BLOCK;
            if (s_stopIndex != NO_INDEX && s_written + n > s_stopIndex) n = (int)(s_stopIndex - s_written);
            if (n <= 0) break;
            arc::engine::generator().render(mono, n, s_written, arc::engine::outputClock());
            for (int i = 0; i < n; i++) {
                const int16_t song = (int16_t)(in[(pos + i) * ch] * vol);
                stereo[2 * i] = s_countIn ? (int16_t)(((int32_t)(nextCountInSample() * vol) + song) / 2) : song;
                stereo[2 * i + 1] = mono[i];
            }
            writeFrames(stereo, n);
            pos += n;
        }
        if (s_stopIndex != NO_INDEX && s_written >= s_stopIndex) finishTake();
        return len;
    }

private:
    audio_tools::AudioInfo _info{44100, 1, 16};
};

audio_tools::AudioSourceSDMMC s_source("/", ".wav", false);
audio_tools::WAVDecoder s_decoder;
Mixer s_mixer;
audio_tools::AudioPlayer s_player(s_source, s_mixer, s_decoder);

bool seekWavSeconds(double songSec) {
    Stream* st = s_player.getStream();
    if (!st) return false;
    fs::File* f = (fs::File*)st;
    const uint32_t frame = (uint32_t)(songSec * s_fsWav + 0.5);
    const uint32_t pos = s_wavDataOffset + frame * (uint32_t)(2 * s_wavChannels);
    return f->seek(pos, SeekSet);
}

void startI2S() {
    s_fsOut = (double)lround(s_fsWav * s_speed);
    if (s_fsOut < 8000) s_fsOut = 8000;
    auto cfg = s_i2s.defaultConfig(audio_tools::TX_MODE);
    cfg.pin_bck = PIN_DAC_BCK;
    cfg.pin_ws = PIN_DAC_LRCK;
    cfg.pin_data = PIN_DAC_DIN;
    cfg.sample_rate = (int)s_fsOut;
    cfg.channels = 2;
    cfg.bits_per_sample = 16;
    cfg.buffer_count = 4;
    cfg.buffer_size = 256;
    s_i2s.begin(cfg);
    s_written = 0;
    arc::engine::outputClock().reset(s_fsOut);
    arc::engine::generator().begin(s_fsOut);
    // Same DMA depth as the internal 48 kHz path, so convert the calibrated latency.
    const double queueInt = 1024.0 * 1e6 / 48000.0, queueOut = 1024.0 * 1e6 / s_fsOut;
    arc::engine::generator().setLatencyUs(arc::engine::outputLatencyUs() - queueInt + queueOut);
}

void prepareStopped() {
    play::lock();
    const PlayShot sh = play::shot();
    play::unlock();
    TcRate r = arc::engine::rate();
    arc::rateFromFps(sh.fps, r);
    s_rate = r;
    uint32_t f = 0;
    if (!(sh.loaded && parseTcFrames(sh.starttime, r, f))) f = 0;
    s_shotStartFrame = f;
    Timeline tl = Timeline::stopped(r, f);
    tl.speed = s_speed;
    arc::engine::setTimeline(tl);
}

void doEnter() {
    if (s_state != St::Off) return;
    arc::engine::setExternalOutput(true);
    for (int i = 0; i < 50 && !arc::engine::internalOutputStopped(); i++) vTaskDelay(pdMS_TO_TICKS(10));
    play::lock();
    s_speed = play::shot().speed > 0 ? play::shot().speed : 1.0f;
    play::unlock();
    xSemaphoreTake(s_sdMtx, portMAX_DELAY);
    s_haveSong = SD_MMC.exists(TRACK);
    s_state = St::Warmup;
    if (s_haveSong) {
        s_source.begin();
        s_player.setIndex(0);
        s_player.begin();
        // Parse the WAV header before the first real start (lazy header parsing
        // would otherwise rewind the file on the first copy()).
        for (int i = 0; i < 40 && !s_player.isActive(); i++) { s_player.copy(); vTaskDelay(2); }
        for (int i = 0; i < 4; i++) s_player.copy();
        scanWavDataOffset();
        s_player.setVolume(1.0f);   // volume is applied in the mixer
    }
    xSemaphoreGive(s_sdMtx);
    startI2S();
    prepareStopped();
    s_state = St::Idle;
    Serial.printf("[PLAY] Playback bereit: %s, %d Hz, Tempo %.2f\n", s_haveSong ? "Song" : "kein Song", s_fsWav, s_speed);
}

void doLeave() {
    if (s_state == St::Off) return;
    s_state = St::Off;
    xSemaphoreTake(s_sdMtx, portMAX_DELAY);
    if (s_haveSong) s_player.end();
    xSemaphoreGive(s_sdMtx);
    s_i2s.end();
    arc::engine::setExternalOutput(false);
}

void doStart(int64_t atNs, bool useShotStart, double fromSec) {
    if (s_state != St::Idle) return;
    play::lock();
    const PlayShot sh = play::shot();
    const PlaySong song = play::song();
    play::unlock();
    const TcRate r = s_rate;

    // Position: shot start (with count-in) or an explicit song position.
    uint32_t startFrame = 0, endFrame = 0;
    const bool haveStart = useShotStart && sh.loaded && parseTcFrames(sh.starttime, r, startFrame);
    const bool haveEnd = useShotStart && sh.loaded && parseTcFrames(sh.endtime, r, endFrame);
    if (!useShotStart) startFrame = (uint32_t)arc::framesIn((int64_t)(fromSec * 1e9), r, nullptr);
    else if (!haveStart) {
        uint32_t f; int64_t into;
        arc::engine::timeline().frameAt(arc::clock::nowNs(), f, into);
        startFrame = f;
    }

    uint32_t prerollFrame = startFrame;
    double prerollSongSec = 0;
    if (haveStart && song.bpm > 0) {
        const int64_t shotNs = arc::framesDurationNs(startFrame, r);
        int64_t startNs = shotNs - (int64_t)(8.0 * 60.0 / song.bpm * 1e9);
        if (startNs < 0) startNs = 0;
        prerollFrame = (uint32_t)arc::framesIn(startNs, r, nullptr);
        prerollSongSec = (double)(shotNs - arc::framesDurationNs(prerollFrame, r)) / 1e9;
    }

    if (s_haveSong) {
        xSemaphoreTake(s_sdMtx, portMAX_DELAY);
        seekWavSeconds(frameSongSec(prerollFrame, r));
        xSemaphoreGive(s_sdMtx);
    }

    // Start sample: the output sample that reaches the DAC at `atNs`.
    if (!atNs) atNs = arc::clock::nowNs() + 150000000LL;
    double idx = arc::engine::sampleIndexAtWall(atNs);
    uint64_t start = idx > 0 ? (uint64_t)ceil(idx) : 0;
    if (start < s_written + 2 * BLOCK) {
        Serial.println("[PLAY] Startzeit zu knapp – starte sofort");
        start = s_written + 2 * BLOCK;
    }
    const int64_t anchorNs = wallOfSample(start);
    Timeline tl = Timeline::anchored(r, prerollFrame, anchorNs, s_speed);

    s_stopIndex = NO_INDEX;
    s_stopIsShotEnd = false;
    if (haveEnd && endFrame > prerollFrame) {
        const double songLen = frameSongSec(endFrame, r) - frameSongSec(prerollFrame, r);
        s_stopIndex = start + (uint64_t)llround(songLen * s_fsWav);
        s_stopIsShotEnd = true;
        s_endFrame = endFrame;
        tl = tl.stoppingAt(wallOfSample(s_stopIndex));
    }
    s_countIn = false;
    if (prerollSongSec > 0) {
        s_ciPerBeat = (uint32_t)(60.0 / song.bpm * s_fsWav);
        s_ciTotal = (uint32_t)(prerollSongSec * s_fsWav);
        if (s_ciTotal > s_ciPerBeat * 8) s_ciTotal = s_ciPerBeat * 8;
        s_ciElapsed = 0;
        s_ciPhase = 0;
    }
    arc::engine::setTimeline(tl);
    s_startIndex = start;
    s_state = St::Armed;
    s_countIn = prerollSongSec > 0;
}

void doStop(int64_t atNs) {
    if (s_state == St::Armed) {
        s_state = St::Idle;
        s_countIn = false;
        prepareStopped();
        return;
    }
    if (s_state != St::Playing) return;
    const double idx = atNs ? arc::engine::sampleIndexAtWall(atNs) : (double)(s_written + BLOCK);
    uint64_t stopAt = idx > (double)s_written ? (uint64_t)ceil(idx) : s_written + BLOCK;
    if (s_stopIndex != NO_INDEX && stopAt >= s_stopIndex) return;   // shot end comes first
    s_stopIndex = stopAt;
    s_stopIsShotEnd = false;
    arc::engine::setTimeline(arc::engine::timeline().stoppingAt(wallOfSample(stopAt)));
}

void processCommands() {
    Cmd c;
    while (xQueueReceive(s_cmdQ, &c, 0)) {
        switch (c.type) {
            case CmdType::Enter: doEnter(); break;
            case CmdType::Leave: doLeave(); break;
            case CmdType::Start: doStart(c.atNs, true, 0); break;
            case CmdType::Stop: doStop(c.atNs); break;
            case CmdType::Seek:
                if (s_state == St::Playing || s_state == St::Armed) {
                    doStop(0);
                    // Restart at the new position as soon as the stop is through.
                    while (s_state == St::Playing) {
                        if (s_haveSong) { xSemaphoreTake(s_sdMtx, portMAX_DELAY); s_player.copy(); xSemaphoreGive(s_sdMtx); }
                        else pumpSilence(BLOCK);
                    }
                    doStart(0, false, c.sec);
                } else if (s_state == St::Idle) {
                    Timeline tl = Timeline::stopped(s_rate, (uint32_t)arc::framesIn((int64_t)(c.sec * 1e9), s_rate, nullptr));
                    tl.speed = s_speed;
                    arc::engine::setTimeline(tl);
                }
                break;
            case CmdType::Prepare: {
                play::lock();
                const float sp = play::shot().speed > 0 ? play::shot().speed : 1.0f;
                play::unlock();
                if (s_state == St::Idle && fabsf(sp - s_speed) > 0.001f) {   // new tempo: new output rate
                    doLeave();
                    doEnter();
                } else if (s_state == St::Idle) {
                    prepareStopped();
                }
                break;
            }
        }
    }
}

void audioTask(void*) {
    for (;;) {
        processCommands();
        switch (s_state) {
            case St::Off:
            case St::Warmup:
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            case St::Idle:
                if (s_uploading) { vTaskDelay(pdMS_TO_TICKS(10)); break; }
                pumpSilence(BLOCK);
                break;
            case St::Armed: {
                const uint64_t left = s_startIndex - s_written;
                if (left == 0) { s_state = St::Playing; break; }
                pumpSilence(left < (uint64_t)BLOCK ? (int)left : BLOCK);
                if (s_written >= s_startIndex) s_state = St::Playing;
                break;
            }
            case St::Playing: {
                bool songData = false;
                if (s_haveSong) {
                    xSemaphoreTake(s_sdMtx, portMAX_DELAY);
                    songData = s_player.copy() > 0 || s_player.isActive();
                    xSemaphoreGive(s_sdMtx);
                }
                if (s_state != St::Playing) break;   // stop index reached inside the mixer
                if (!songData) {
                    // Song ended (or none): keep LTC running until the stop index.
                    if (s_stopIndex == NO_INDEX) {
                        s_stopIndex = s_written + BLOCK;
                        arc::engine::setTimeline(arc::engine::timeline().stoppingAt(wallOfSample(s_stopIndex)));
                    }
                    int n = BLOCK;
                    if (s_written + n > s_stopIndex) n = (int)(s_stopIndex - s_written);
                    if (n > 0) pumpSilence(n);
                    if (s_written >= s_stopIndex) finishTake();
                }
                break;
            }
        }
    }
}

void post(CmdType t, int64_t atNs = 0, double sec = 0) {
    Cmd c{t, atNs, sec};
    xQueueSend(s_cmdQ, &c, pdMS_TO_TICKS(100));
}

}  // namespace

namespace audio {

void begin() {
    s_sdMtx = xSemaphoreCreateMutex();
    s_cmdQ = xQueueCreate(8, sizeof(Cmd));
    SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
    if (!SD_MMC.begin("/sdcard", true)) Serial.println("[PLAY] SD-Karte nicht gefunden");
    xTaskCreatePinnedToCore(audioTask, "PLAYAUDIO", 8192, nullptr, configMAX_PRIORITIES - 3, nullptr, 1);
}

void enterPlayback() { post(CmdType::Enter); }
void leavePlayback() { post(CmdType::Leave); }
bool inPlayback() { return s_state != St::Off; }
void start(int64_t atNs) { post(CmdType::Start, atNs); }
void stop(int64_t atNs) { post(CmdType::Stop, atNs); }
void seekSongSeconds(double sec) { post(CmdType::Seek, 0, sec); }
void prepareShot() { post(CmdType::Prepare); }
bool playing() { return s_state == St::Playing || s_state == St::Armed; }
bool countInActive() { return s_countIn && s_state == St::Playing; }

void setVolume(int percent) { s_volume = constrain(percent, 0, 100); }
int volume() { return s_volume; }

bool uploadBegin() {
    s_uploading = true;
    if (playing()) stop(0);
    vTaskDelay(pdMS_TO_TICKS(50));
    xSemaphoreTake(s_sdMtx, portMAX_DELAY);
    if (s_haveSong && s_state != St::Off) s_player.end();
    s_haveSong = false;
    File root = SD_MMC.open("/");
    if (root) {
        for (File f = root.openNextFile(); f; f = root.openNextFile()) {
            const String name = f.name();
            const bool wav = !f.isDirectory() && (name.endsWith(".wav") || name.endsWith(".WAV"));
            f.close();
            if (wav) SD_MMC.remove("/" + name);
        }
        root.close();
    }
    s_uploadFile = SD_MMC.open(TRACK, FILE_WRITE);
    xSemaphoreGive(s_sdMtx);
    if (!s_uploadFile) { s_uploading = false; return false; }
    return true;
}

void uploadData(const uint8_t* data, size_t len) {
    if (!s_uploading || !s_uploadFile) return;
    xSemaphoreTake(s_sdMtx, portMAX_DELAY);
    s_uploadFile.write(data, len);
    xSemaphoreGive(s_sdMtx);
}

bool uploadFinish() {
    xSemaphoreTake(s_sdMtx, portMAX_DELAY);
    const bool ok = (bool)s_uploadFile;
    if (s_uploadFile) s_uploadFile.close();
    xSemaphoreGive(s_sdMtx);
    s_uploading = false;
    if (s_state != St::Off) { post(CmdType::Leave); post(CmdType::Enter); }   // reload the new song
    return ok;
}

bool uploading() { return s_uploading; }

void onTakeEnd(void (*cb)(const arc::Timecode&, int64_t)) { s_takeEndCb = cb; }

}  // namespace audio
