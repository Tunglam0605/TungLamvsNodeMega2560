<p align="center">
  <img src="docs/hero.svg" alt="Stack light hero" width="1000">
</p>
<p align="center">
  <img src="docs/intro-icons.svg" alt="Feature icons" width="1000">
</p>

# TungLamvsNodeMega2560
Stack Light + Siren Controller for Arduino Mega 2560

> Non-blocking, timer-driven stack light controller with a simple serial protocol (Serial0/USB).

## ✨ <span style="color:#ff6b6b; text-shadow:0 0 10px #ffd1d1;">Overview</span>
This project drives a 3-color stack light and a siren from an Arduino Mega 2560. It listens to
line-based ASCII commands over Serial0 (USB), applies a robust state machine, and stays responsive
thanks to a 100 Hz Timer3 ISR (no `delay()` in the main loop).

## ⚡ <span style="color:#f59e0b; text-shadow:0 0 10px #fde68a;">Highlights</span>
- Timer3 @ 100 Hz handles blinking and beep timing (stable, non-blocking).
- Line-based serial parser with ACK/ERR responses and a timeout failsafe.
- READY/RUN/CANCELED/SUCCEEDED/FAIL states mapped to lamp patterns.
- Manual control commands for testing (SIREN, BEEP, SET).

## 🔧 <span style="color:#22c55e; text-shadow:0 0 10px #bbf7d0;">Hardware</span>
### 📌 <span style="color:#06b6d4; text-shadow:0 0 8px #a5f3fc;">Pin map</span>
| Function | Arduino Mega pin | External device |
| --- | --- | --- |
| Green lamp | D22 | Stack light green channel via relay/MOSFET |
| Yellow lamp | D24 | Stack light yellow channel via relay/MOSFET |
| Red lamp | D26 | Stack light red channel via relay/MOSFET |
| Siren | D28 | Siren via relay/MOSFET |
| Heartbeat | D13 | On-board LED (built-in) |
| GND | GND | Common ground with external supply |

### 🧷 <span style="color:#3b82f6; text-shadow:0 0 8px #bfdbfe;">Wiring diagram</span>
![Wiring diagram](docs/wiring.svg)

### 📝 <span style="color:#f97316; text-shadow:0 0 8px #fed7aa;">Wiring notes</span>
- Stack light and siren are typically 12-24 V loads. Use a relay or MOSFET driver and a flyback diode.
- Do NOT connect 12-24 V directly to Arduino pins.
- All grounds must be common: Arduino GND <-> driver GND <-> power supply GND.
- `ACTIVE_LEVEL` is set to `HIGH`. If your driver is active-low, change it to `LOW`.

## 🌈 <span style="color:#ec4899; text-shadow:0 0 10px #fbcfe8;">State mapping</span>
| State | Green | Yellow | Red | Siren |
| --- | --- | --- | --- | --- |
| IDLE | OFF | OFF | OFF | OFF |
| READY | ON | OFF | OFF | OFF |
| RUN | BLINK SLOW (anti-phase) | BLINK SLOW (anti-phase) | OFF | OFF |
| CANCELED | OFF | ON | OFF | OFF |
| SUCCEEDED | BLINK FAST | OFF | OFF | OFF |
| FAIL | OFF | OFF | BLINK FAST | ON |

## 📡 <span style="color:#0ea5e9; text-shadow:0 0 10px #bae6fd;">Serial protocol</span>
### 🔹 <span style="color:#14b8a6; text-shadow:0 0 8px #99f6e4;">Basics</span>
- Baud rate: `115200`
- Line endings: `\n` or `\r`
- Case-insensitive (input is normalized to uppercase)

### 💬 <span style="color:#ef4444; text-shadow:0 0 8px #fecaca;">Commands</span>
```
IDLE | READY | RUN | CANCELED | SUCCEEDED | FAIL
STATE:<state>                     (same as above)
SIREN ON | SIREN OFF
BEEP <ms>
SET <COLOR> <ON|OFF|BLINK> [period_ms]   (COLOR: GREEN|YELLOW|RED)
```

### ✅ <span style="color:#10b981; text-shadow:0 0 8px #bbf7d0;">Responses</span>
- `ACK:STATE=...`, `ACK:SIREN=...`, `ACK:BEEP=...`, `ACK:SET ...`
- `ACK:BEEP_DONE` when a timed beep finishes
- `ERR:...` for invalid commands

## ⏱️ <span style="color:#84cc16; text-shadow:0 0 10px #ecfccb;">Defaults and timing</span>
| Parameter | Value |
| --- | --- |
| `SERIAL_BAUD` | 115200 |
| `LINK_TIMEOUT_MS` | 5000 (no commands => IDLE) |
| `HEARTBEAT_MS` | 1000 |
| `BLINK_SLOW_MS` | 600 |
| `BLINK_FAST_MS` | 200 |

## ⬆️ <span style="color:#2563eb; text-shadow:0 0 10px #bfdbfe;">Build and upload</span>
1. Open `NODE_MEGA.ino` in Arduino IDE.
2. Board: `Arduino Mega 2560` (ATmega2560).
3. Port: the USB serial port of the board.
4. Upload.

## 🧪 <span style="color:#f43f5e; text-shadow:0 0 10px #fecdd3;">Quick test (Serial Monitor)</span>
```
READY
RUN
BEEP 300
SIREN OFF
STATE:FAIL
IDLE
```

## 🛡️ <span style="color:#dc2626; text-shadow:0 0 10px #fecaca;">Safety</span>
- Use proper drivers for inductive loads and add flyback diodes.
- Do not exceed the current limits of the Arduino pins.
- Keep wiring short, secure, and labeled.

## 👤 <span style="color:#64748b; text-shadow:0 0 10px #cbd5e1;">Author</span>
Tung Lam

## 📄 <span style="color:#0f766e; text-shadow:0 0 10px #99f6e4;">License</span>
MIT (see `LICENSE`).
