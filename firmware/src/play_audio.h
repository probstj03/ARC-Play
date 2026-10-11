#pragma once
// Playback of the song (SD /track.wav) with count-in and phase-locked LTC.
//
// In playback mode this module owns the I2S output (ARC-Core's internal
// output is switched off). Its audio task always writes to I2S – silence
// while idle – so the output clock keeps running and starts can be scheduled
// to the exact output sample: song, count-in and LTC (timeline anchored on
// the same sample) start together. The shot end is stopped on the sample too.
//
// Channels: left = song + count-in (J3), right = LTC (J2).
#include <Arduino.h>
#include <ArcCore.h>

namespace audio {

void begin();                              // SD + player; call once
void enterPlayback();                      // take over the I2S output
void leavePlayback();
bool inPlayback();

// Transport (wall-clock ns, 0 = now + small lead). start() plays the current
// shot with count-in (8 beats before the shot start, if BPM is known).
void start(int64_t atNs = 0);
void stop(int64_t atNs = 0);
void seekSongSeconds(double sec);          // stopped: new position; playing: restart there
void prepareShot();                        // shot / speed changed: reposition (stopped)
bool playing();
bool countInActive();

void setVolume(int percent);
int volume();

// Upload of a new /track.wav over the WebSocket (binary frames).
bool uploadBegin();
void uploadData(const uint8_t* data, size_t len);
bool uploadFinish();
bool uploading();

// Called on the audio task when a take reached the shot end.
void onTakeEnd(void (*cb)(const arc::Timecode& endTc, int64_t endWallNs));

}  // namespace audio
