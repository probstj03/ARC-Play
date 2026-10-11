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

## Firmware (v2.0.0-dev)

PlatformIO-Projekt in `firmware/` (Arduino-Framework, ESP32-S3). Zeitbasis, Timecode, Jam Sync und
Netzwerk kommen aus **[ARC-Core](https://github.com/probstj03/ARC-Core)**. Das Repo muss neben
`ARC-Play` liegen, weil es per `symlink://../../ARC-Core` eingebunden wird.

| Modul | Aufgabe |
|---|---|
| ARC-Core | RTC-Uhr, phasengeregelter LTC-Generator, Decoder, Jam Sync mit Prüfphase, Latenz-Loopback, WLAN/ARC-Net, Protokoll v2, Ereignis-Log |
| `play_audio` | Song von SD (`/track.wav`) mit 8-Schläge-Vorzähler, Start und Shot-Ende sample-genau geplant (`at`) |
| `play_ui` | TFT + Drehencoder |
| `play_link` | Play-Protokoll: Shot, Song-Info, WAV-Upload, Lautstärke, Take-Ereignis am Shot-Ende |
| `play_state` | Shot- und Song-Daten |

```bash
cd firmware
cp src/secrets.example.h src/secrets.h   # einmalig: eigene WLANs eintragen
pio run -t upload
pio device monitor -b 115200             # "help" zeigt alle Befehle
```

| Modus | Funktion |
|---|---|
| Free Run | TC = Tageszeit aus der RTC, phasengeregelt |
| Record Run | TC läuft nur während der Aufnahme. Start und Stopp auch geplant (`at`). |
| Playback | Song + LTC ab Shot-Start mit Vorzähler und Tempo, sample-genauer Start und Stopp am Shot-Ende |
| Slave / Jam Sync | TC von J2 übernehmen. Jam → Free Run stellt die RTC µs-genau und prüft 30 s lang. |

Bildraten: 24, 25, 29.97 DF, 30. Kanäle: links = Song (J3), rechts = LTC (J2).

**Bedienung:**
- Drehen = Lautstärke
- Klick = Play/Pause (nach einem Jam: „Kabel abgezogen“ bestätigen)
- Doppelklick = Shot-Auswahl über ARC-Studio
- Lang drücken = Einstellungen: Modus, Bildrate, NTP, SD-Karte leeren

**Protokoll:** WebSocket `ws://<ip>/ws`, Protokoll v2 aus ARC-Core.
- Neu sind `hello`, `event` (`take_end` mit µs-Zeit), `pong` und geplante Befehle mit `at`.
- Für das aktuelle ARC-Studio bleiben `newTake`, `status`, `upload_prepared` und `upload_ack` erhalten.

## Stand

- Die Firmware ist neu aufgebaut und noch nicht auf der Hardware getestet.
- Wichtig beim ersten Test:
  - TFT-Anzeige
  - WAV-Wiedergabe mit Vorzähler
  - Stopp am Shot-Ende
  - Jam Sync mit ARC-Slate
- Die Ausgangslatenz der Playback-Ausgabe wird live über J2 nachgeführt. `?` zeigt Latenz und Phasenfehler.
- Offen sind Offline-Projekte auf SD und die Leader-Rolle ohne ARC-Studio (Plan, Teil C3).
