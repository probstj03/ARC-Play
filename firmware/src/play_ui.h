#pragma once
// TFT (ILI9341 240x320) and rotary encoder of ARC-Play.
//
// Idle screen: turn = volume · click = play/pause (after a jam: confirm that
// the cable is unplugged) · double click = browse shots (ARC-Studio) ·
// long press (1.5 s) = settings (mode, frame rate, NTP, SD card).
#include <Arduino.h>

namespace playui {

void begin();
void requestRedraw();

}  // namespace playui
