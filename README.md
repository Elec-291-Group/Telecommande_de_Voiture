# Magnetic Field Path Tracking Remote Controller

Remote controller for an autonomous robot that follows magnetic field paths, supports joystick manual control, and tracks the robot's trajectory on a desktop GUI using IMU data.

Built for the **EFM8LB1** microcontroller (72 MHz, 8051-based) with a **PyQt5** desktop application.

## System Overview

```
  Laptop (GUI)                Remote Controller              Robot Car
 ┌────────────┐   BLE/UART   ┌──────────────────┐   IR TX/RX  ┌──────────┐
 │ PyQt5 App  │◄────────────►│ EFM8LB1 MCU      │◄───────────►│ STM32    │
 │  - Path    │  JDY-23 BLE  │  - 16x2 LCD      │  38 kHz     │  - Motors│
 │    editor  │  9600 baud   │  - Joystick (2-ax)│  SIRC proto │  - IMU   │
 │  - IMU viz │              │  - 4 push buttons │             │  - Mag   │
 │  - Console │              │  - IR LED + TSOP  │             │    sensor│
 └────────────┘              └──────────────────────┘          └──────────┘
```

## Features

### Remote Controller (EFM8LB1 Firmware)
- **Three operating modes**: Field/Auto tracking, Remote (joystick), Pathfinding
- **Path selection**: 3 preset paths + manual intersection configuration (up to 8 intersections)
- **LCD state machine**: 18-state menu system navigated by joystick and buttons
- **IR communication**: Bidirectional with robot via SIRC protocol (28-bit frames: 8-bit cmd + 16-bit data + 4-bit addr)
- **Bluetooth bridge**: Relays IMU data and motor power from car to GUI; receives drawn paths from GUI

### Desktop GUI (Python/PyQt5)
- **BLE connection manager**: Scan, connect, and communicate with JDY-23 module
- **Path editor**: Interactive grid canvas for drawing waypoint paths (up to 32 waypoints)
- **Real-time telemetry**: IMU register display, motor power levels, robot trajectory visualization
- **Command console**: Send/receive text commands over BLE

## Communication Protocols

| Link | Medium | Baud/Freq | Direction | Data |
|------|--------|-----------|-----------|------|
| GUI ↔ Controller | BLE (JDY-23) | 9600 baud | Bidirectional | IMU stream, motor power, path waypoints, commands |
| Controller ↔ Car | IR (38 kHz) | SIRC encoding | Bidirectional | Mode/path selection, joystick, waypoints, crossing actions |

### Key IR Commands
| Cmd | Purpose |
|-----|---------|
| START/PAUSE/RESET | Car control |
| MODE | Set field/remote/pathfind |
| PATH | Select preset path or manual |
| JOYSTICK_X/Y | Manual motor control |
| PATH_WPT[0-31] | Waypoint transmission |
| MANUAL_PATH | Packed intersection directions |

### BLE Commands
- `STREAM_ON/OFF` — Enable/disable real-time IMU and motor power streaming
- `PATH_BEGIN,<n>` / `WPT,<i>,<x>,<y>` / `PATH_END` — Upload drawn path

## Hardware

- **MCU**: Silicon Labs EFM8LB1 @ 72 MHz
- **Display**: 16x2 character LCD (4-bit mode)
- **Input**: 2-axis analog joystick (ADC) + 4 push buttons (Start, Pause, Reset, TX Cmd)
- **IR TX**: LED on P2.1, 38 kHz carrier via Timer0
- **IR RX**: TSOP33x demodulator on P0.7, Port Match interrupt
- **BLE**: JDY-23 module on UART1 (9600 baud)

## Project Structure

```
src/           EFM8 firmware source
  main.c         Entry point, button/joystick handlers, main loop
  bootloader.c   Clock, pin, ADC, and peripheral initialization
  ir_tx.c        IR transmitter (SIRC protocol, Timer0/Timer2)
  ir_rx.c        IR receiver (Port Match ISR, frame decoder)
  bluetooth.c    BLE command parser, IMU/power streaming
  lcd_fsm.c      18-state LCD menu system
  lcd.c          LCD 4-bit driver
  uart.c         UART0 (debug, 115200) and UART1 (BLE, 9600) drivers
  data_buffers.c Path staging/active buffers, IMU history ring buffer
  timer.c        Software delay utilities
inc/           Header files and configuration constants
build/         Compiled output (main.hex)
gui.py         PyQt5 desktop application
ble_receiver.py   Async BLE client (Bleak library)
pathfinder.py     Interactive waypoint path editor widget
jdy23_monitor.py  Standalone BLE monitor for debugging
Makefile       Build with CrossIDE/SDCC toolchain
```

## Build

### Firmware
```bash
make CROSSIDE_DIR=D:/CrossIDE
```
Flash `build/main.hex` to EFM8LB1.

### GUI
```bash
pip install PyQt5 bleak
python gui.py
```
