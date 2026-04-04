# La Voiture Remote — EFM8 Controller & Desktop GUI

Remote controller for an autonomous wire-tracking robot car. Features a joystick interface, 16x2 LCD menu system, bidirectional IR communication with the car, and a PyQt5 desktop application for path planning and real-time telemetry visualization.

Built for the **EFM8LB1** microcontroller (Silicon Labs 8051, 72 MHz) with **SDCC/CrossIDE** toolchain.

## System Overview

```
  Laptop (GUI)                Remote Controller (this repo)    Robot Car
 ┌────────────┐   BLE/UART   ┌──────────────────────┐  IR TX/RX  ┌──────────┐
 │ PyQt5 App  │◄────────────►│ EFM8LB1 @ 72 MHz     │◄──────────►│ STM32    │
 │  - Path    │  JDY-23 BLE  │  - 16x2 LCD (4-bit)  │  38 kHz    │  - Motors│
 │    editor  │  9600 baud   │  - Joystick (2-axis)  │  28-bit    │  - IMU   │
 │  - IMU viz │              │  - 4 push buttons     │  SIRC      │  - Mag   │
 │  - Console │              │  - IR LED + TSOP33x   │  protocol  │    sensor│
 └────────────┘              └──────────────────────┘             └──────────┘
```

## Features

### Remote Controller (EFM8 Firmware)
- **Three operating modes**: Field tracking (inductive), Remote (joystick), Pathfinding (autonomous waypoints)
- **Path selection**: 3 preset intersection sequences + manual configuration (up to 8 intersections with 4 directions each)
- **18-state LCD menu**: Joystick and button navigation through mode selection, path configuration, and run monitoring
- **Bidirectional IR**: SIRC protocol (28-bit frames: 8-bit cmd + 16-bit data + 4-bit address) at 38 kHz carrier
- **Bluetooth bridge**: Streams real-time IMU data and motor power from car to GUI; receives waypoint paths from GUI
- **Real-time joystick**: ADC-sampled X/Y axes mapped to 0–255 bytes, transmitted via IR at update rate

### Desktop GUI (PyQt5)
- **BLE connection manager**: Scan and connect to JDY-23 module via Bleak library
- **Interactive path editor**: Grid canvas for drawing waypoint routes (up to 32 waypoints, 8 cm grid resolution)
- **Real-time telemetry**: IMU register display (accel + gyro), motor power levels, robot trajectory trail (last 2000 points)
- **Command console**: Send/receive BLE text commands for debugging

## Communication Protocols

### IR Protocol (Controller ↔ Car)

28-bit pulse-distance frames with NEC-style timing:

| Field | Bits | Description |
|-------|------|-------------|
| Command | [27:20] | 8-bit operation code |
| Data | [19:4] | 16-bit payload |
| Address | [3:0] | 4-bit device ID (0x6 = controller, 0x7 = car) |

**Timing**: T = 263 µs (10 cycles @ 38 kHz). Start: 4T burst + 2T space. Bit 0: 3T falling-to-falling. Bit 1: 4T falling-to-falling. Threshold: 3.5T (~921 µs).

**Key challenge solved**: High duty cycles caused the TSOP receiver's AGC to drop gain mid-packet. Re-engineered frames with silence areas to keep duty cycle < 50%.

| Cmd ID | Name | Direction | Description |
|--------|------|-----------|-------------|
| 0–2 | START / PAUSE / RESET | TX → Car | Car control |
| 3 | MODE | TX → Car | 0=field, 1=remote, 2=pathfind |
| 4 | PATH | TX → Car | 1–3=preset, 4=manual |
| 5–6 | JOYSTICK_X / Y | TX → Car | 0–255 axis values |
| 7–24 | IMU_REG[0–17] | Car → RX | 16-bit sensor readings |
| 25 | DATA_RECEIVED | Car → RX | Mode confirmation |
| 26 | CROSSING_ACTION | Car → RX | Intersection direction |
| 27–28 | LEFT / RIGHT_POWER | Car → RX | Motor power feedback |
| 39 | ZERO_YAW | TX → Car | IMU calibration trigger |
| 40 | MANUAL_PATH | TX → Car | Packed 2-bit intersection directions |

### Bluetooth Protocol (Controller ↔ GUI via JDY-23)

Text-based CRLF-terminated commands over UART1 at 9600 baud:

| Command | Direction | Description |
|---------|-----------|-------------|
| `STREAM_ON` / `STREAM_OFF` | GUI → Controller | Enable/disable telemetry streaming |
| `STATUS` | GUI → Controller | Query streaming state |
| `imu,<reg>,<value>` | Controller → GUI | IMU register update (~500 ms interval) |
| `pwr,<0\|1>,<-180..127>` | Controller → GUI | Left/right motor power |
| `PATH_BEGIN,<n>` | GUI → Controller | Start n-waypoint upload |
| `WPT,<i>,<x_cm>,<y_cm>` | GUI → Controller | Waypoint at index |
| `PATH_END` | GUI → Controller | Commit and validate path |

## Hardware

| Component | Part / Pin | Purpose |
|-----------|-----------|---------|
| MCU | EFM8LB1 @ 72 MHz | 8051-based, 64 KB Flash |
| LCD | 16x2 character (4-bit) | Menu display |
| Joystick | 2-axis analog, P1.4 (Y) / P1.5 (X) | Navigation + motor control |
| Buttons | PB_START (P2.5), PB_PAUSE (P2.6), PB_RESET (P2.4), PB_TXCMD (P3.0) | State transitions + IR transmit |
| Joystick SW | P2.3 | Alternative select button |
| IR TX | LED on P2.1, Timer0 carrier + Timer2 envelope | 38 kHz SIRC transmission |
| IR RX | TSOP33x on P0.7, Port Match ISR + Timer3 | Pulse-distance decoding |
| BLE | JDY-23 on UART1 (P0.2 TX / P0.3 RX) | 9600 baud wireless link to GUI |
| Debug | UART0 @ 115200 baud | Serial console logging |

## LCD State Machine (18 States)

```
S0  Welcome ──► S1  Choose Mode ─┬─► S13 Ready TX (Field) ──► S3  Ready (Auto) ──► S5  Running
                                 ├─► S14 Ready TX (Remote) ─► S4  Ready (Remote) ► S6  Running
                                 └─► S2  Choose Path ───────┬─► S15 Ready TX (BLE Path) ► S8 ► S9/S11
                                                            └─► S17 Manual Intersections ► S18 Ready TX
                                                    S7  Pause (from any running state)
```

Navigation: Joystick X-axis cycles options, buttons confirm/transmit. Latching prevents repeat triggers until joystick returns to center.

## Project Structure

```
src/                Firmware source
  main.c              Entry point, button/joystick handlers, main loop
  bootloader.c        Clock (72 MHz), pin, ADC initialization
  ir_tx.c             IR transmitter: Timer0 (38 kHz carrier) + Timer2 (envelope FSM)
  ir_rx.c             IR receiver: Port Match ISR + Timer3 timing, frame decoder
  uart.c              Dual UART: UART0 debug (115200) + UART1 BLE (9600)
  bluetooth.c         BLE command parser, IMU/power streaming relay
  lcd.c               16x2 LCD 4-bit driver
  lcd_fsm.c           18-state menu system
  data_buffers.c      Path staging/active buffers, 32-sample IMU ring buffer
  timer.c             Software delay utilities
inc/                Headers and configuration
  config.h            System constants, pin map, IR commands, timing
  ir_tx.h / ir_rx.h   Protocol structures and APIs
  data_buffers.h      Waypoint_t, IMU_Sample_t, buffer interfaces
  lcd_fsm.h           State enum and LCD variables
gui.py              PyQt5 desktop application
ble_receiver.py     Async BLE client (Bleak library)
pathfinder.py       Interactive waypoint grid editor
jdy23_monitor.py    Standalone BLE debugging monitor
Makefile            CrossIDE/SDCC build configuration
build/              Compiled output (main.hex)
```

## Build

### Firmware
```bash
make CROSSIDE_DIR=D:/CrossIDE
```
Output: `build/main.hex` — flash to EFM8LB1 via programmer.

### GUI
```bash
pip install PyQt5 bleak
python gui.py
```

### BLE Debugging
```bash
python jdy23_monitor.py
```

## Timer Architecture

| Timer | Rate | Purpose |
|-------|------|---------|
| Timer 0 | 38 kHz | IR carrier frequency generation |
| Timer 1 | Baud clock | UART0 bit timing (SYSCLK/12) |
| Timer 2 | 3,817 Hz (263 µs) | SIRC envelope — one T unit per tick |
| Timer 3 | Free-running (6 MHz) | IR RX pulse-width measurement |

## Buffer Architecture

- **UART ring buffers**: 64-byte TX + 64-byte RX for each UART (interrupt-driven, non-blocking)
- **IR RX frame buffer**: 8-frame circular queue
- **Path buffers**: Staging (BLE upload) → Active (validated, ready for IR TX), 32 waypoints max
- **IMU history**: 32-sample ring buffer for trajectory visualization
