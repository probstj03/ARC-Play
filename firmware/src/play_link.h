#pragma once
// ARC-Play specific protocol on top of arc::net (ARC-Core): shot and song data,
// playback transport (scheduled with "at"), volume, WAV upload, take events.
#include <Arduino.h>

namespace playlink {

void begin();
void loop();

void requestShotStep(int dir);   // ask ARC-Studio for the next/previous shot
void updateUserBits();
void onModeChanged();            // engine mode changed (enter/leave playback)

}  // namespace playlink
