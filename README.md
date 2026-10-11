# ARC-Play

ARC-Play ist das Zuspielgerät des ARC-Systems. Es spielt den Song eines Shots ab, auf Wunsch mit Vorzähler und
veränderter Geschwindigkeit. Dazu gibt es einen sample-genauen **LTC-Timecode** mit Shot-Userbits aus
(Kamera, Roll, Scene, Shot, Take). Gesteuert wird ARC-Play aus **ARC-Studio** ([ARC-Server](https://github.com/probstj03/ARC-Server))
oder direkt am Gerät über das TFT-Display und den Drehencoder.

## Hardware

- Platine **ARC_AP** (`hardware/ARC_Playback.1`, KiCad 9) mit ESP32-S3-DevKitC-1 N16R8
- PCM5102A-DAC für LTC und Audio, LTC-Eingangsstufe (LM358 + 74HC14), DS3231-RTC, Micro-SD, 2,4"-TFT (ILI9341), Drehencoder KY-040
- **Verdrahtung:** [`hardware/wiring/ARC-Play_Verdrahtung.svg`](hardware/wiring/ARC-Play_Verdrahtung.svg)
- **Bauteile und Pins:** [`hardware/components.txt`](hardware/components.txt)
- **Gehäuse:** `hardware/3D-Druck` (FreeCAD + STL)

Dieselbe Platine wird auch für ARC-Slate verwendet.

## Firmware

PlatformIO-Projekt in `firmware/` (Arduino-Framework, ESP32-S3).

```bash
cd firmware
pio run -t upload
pio device monitor -b 115200
```

| Modus | Funktion |
|---|---|
| Free Run | TC = Tageszeit aus der RTC |
| Record Run | TC läuft nur während der Aufnahme |
| Playback | Song + LTC ab Shot-Start-TC, mit Vorzähler und Geschwindigkeit |
| Slave / Jam Sync | TC von einer externen Quelle über J2 übernehmen |

Bildraten: 24, 25, 29.97 DF, 30.

- **Kommunikation:** WebSocket-Server auf `ws://<ip>/ws`. Nachrichten: `shot`, `command`, `song_info`,
  WAV-Upload, `status`, `newTake`.
- **Serielle Befehle** (115200 Baud, je ein Zeichen):

  | Zeichen | Wirkung |
  |---|---|
  | `1`–`6` | Modus: Free Run, Record Run, Playback, Slave, Jam Free, Jam Rec |
  | `q` / `w` / `e` / `r` | 30 / 25 / 29.97 DF / 24 fps |
  | `s` | Start/Stop |
  | `x` | Stop |
  | `n` | NTP-Sync |

## Roadmap

Diese Punkte sind geplant:
- Umstieg auf die gemeinsame Library **ARC-Core**:
  - RTC-disziplinierte Zeitbasis
  - subframe-genauer Jam Sync mit ARC-Slate
  - geplante Befehle
  - Protokoll v2
- WLAN-Liste vom Server statt fest einprogrammiert; eigenes Netz „ARC-Net" als Rückfall
- Offline-Projekte auf SD
