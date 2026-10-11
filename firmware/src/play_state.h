#pragma once
// Shot + song state of ARC-Play. Access between lock()/unlock().
#include <Arduino.h>

struct PlayShot {
    int    scene = 0;
    String shot = "0";
    int    take = 0;
    String roll = "A001";
    String camera;           // camera operator / DOP (display only)
    String starttime;        // shot start TC (song position)
    String endtime;          // shot end TC
    float  speed = 1.0f;     // playback speed (pitch changes accordingly)
    float  fps = 25.0f;
    bool   loaded = false;   // shot data received
};

struct PlaySong {
    String title;
    float  durationSec = 0;
    float  bpm = 0;
    int    timeSignature = 4;
};

namespace play {

void begin();
void lock();
void unlock();
PlayShot& shot();
PlaySong& song();
void touch();
uint32_t revision();

}  // namespace play
