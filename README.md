# MERCOM-1: Satellite-on-the-Move (SoTM) Terminal Prototype

MERCOM-1 is a three-axis pointing system that keeps an antenna locked onto a fixed target while mounted on a moving platform. As the base tilts or rotates, an IMU on the antenna head measures its orientation. The firmware drives the pan, tilt and CE (cross-elevation) motors accordingly and keeps a boresight laser on the target point. The system is controlled from a desktop application over USB or a LoRa wireless link.

This repository contains the board firmware, the desktop control application and the test tools written during development.

![Control and data tab](docs/arayuz_kontrol.png)
*Desktop application, control and data tab (values shown are sample data; the user interface is in Turkish).*

## Features

**Motion and tracking**
- Three axes: **pan** (0–360°, continuous, always takes the shortest path), **tilt** (−90…90°) and **CE** (0–4°, small lateral corrections).
- **Automatic tracking:** the target direction is locked once; as the base moves, the deviation measured by the IMU is corrected by a PI controller.
- **Gyro feed-forward:** the base angular rate is compensated before an error builds up, with separate gains for pan and tilt.
- **Parallax correction:** the laser offset from the rotation axes and the target distance are taken into account, so the target point is held rather than just the direction.
- Dead band and integral limit; axes hold position if IMU data becomes stale.
- **Manual mode:** go to angle, ±0.1/1/10° steps, press-and-hold jogging, oscillation between two angles.

**Reliability**
- **Closed loop:** magnetic encoders on the motor shafts detect missed steps and correct the position.
- **CE position hold:** in automatic mode the CE axis is held at the lock angle using its encoder.
- **IMU recovery:** if the IMU stream stops for 300 ms, the I2C bus is reset first and then the sensor if needed.
- **Watchdog:** the board restarts itself if the main loop hangs and reports the reset cause at boot.
- **Persistent parameters:** 78 parameters are stored in flash and restored after a power cycle.
- **STOP and EMERGENCY STOP:** STOP ramps down; EMERGENCY STOP disables the drivers immediately.

**Communication**
- The same protocol over USB serial and LoRa (E22-900T22D, 9.6 kbps, listen-before-talk).
- Framed packets with CRC16; commands are acknowledged (ACK).
- 84-byte telemetry packet.

## Hardware

| Component | Model |
|---|---|
| Controller | Teensy 4.1 (main board); first version on STM32 Nucleo-H753ZI |
| Motors | 3 × NEMA17 stepper motors; pan 1:10 and tilt 1:5 geared, CE direct drive |
| Drivers | 3 × TMC2209 (STEP/DIR/EN, 8 microsteps) |
| Encoders | 3 × AS5048A magnetic encoders (SPI) |
| IMU | BNO085 (I2C, Game Rotation Vector, 100 Hz) |
| Wireless | 2 × E22-900T22D LoRa (terminal + ground station bridge) |
| Laser | 5 mW, switched by a MOSFET |

Pin map and wiring rules: [MERCOM1_OZELLIKLER.md](MERCOM1_OZELLIKLER.md) (in Turkish).

## Software architecture

| Layer | Files | Role |
|---|---|---|
| Hardware | `src/hw.cpp` (Nucleo), `src/hw_teensy.cpp` | Timers, pins, software SPI, watchdog, persistent storage |
| Drivers | `src/drv/` | `stepgen` 20 kHz step generator (DDA), `as5048a`, `bno085`, `tmc2209` |
| Control | `src/ctrl/axis.c`, `src/ctrl/pointing.c` | Trapezoidal velocity profile, limits, closed loop; tracking, feed-forward, parallax |
| Application | `src/app/` | Protocol, frame parser, parameter table, state machine, telemetry |
| Entry | `src/main.cpp` | Setup, main loop, command handling |

Step generation runs in a 20 kHz timer interrupt and the control loop in a 1 kHz timer interrupt. The algorithm code (`app`, `ctrl`, `drv/stepgen`) is plain C and board-independent; only the `hw` layer is rewritten when the board changes.

## Build and run

1. Install the PlatformIO extension in VS Code and open this folder.
2. Build and upload:
   ```
   pio run -e teensy41 -t upload        # Teensy 4.1
   pio run -e nucleo_h753zi -t upload   # Nucleo-H753ZI
   ```
3. The desktop application needs Python 3 and `pip install pyserial`:
   ```
   python tools/mercom_gui.py
   ```
   Select the serial port and press "Bağlan" (Connect). To connect over LoRa, select the port of the ground station bridge (`tools/lora_koprusu/`).

## Desktop application

**Control and data tab:**
- Pointing error, color-coded against Ø10 and Ø20 target circles.
- Mode buttons (manual, lock on target, automatic tracking).
- Status indicators, IMU angles, encoder table.
- A motion panel for each axis.
- Board messages and CSV data logging.

**Settings tab:** axis and tracking parameters can be read from the board, written to it and saved on the computer (`tools/params.json`).

![Settings tab](docs/arayuz_ayarlar.png)
*Settings tab.*

## Protocol

Frame: `A5 | LEN | ID | PAYLOAD | CRC16 lo | CRC16 hi`.
- CRC16-CCITT, computed from LEN to the end of PAYLOAD.
- Numbers are little-endian.

| ID | Direction | Payload | Meaning |
|---|---|---|---|
| 0x01 | PC→MCU | — | ping |
| 0x02 | PC→MCU | u8 mode | 1 MANUAL, 2 AUTO |
| 0x03 | PC→MCU | u8 axis, f32 dps | jog at a rate, 0 stops |
| 0x04 | PC→MCU | u8 axis, f32 deg | go to angle |
| 0x05 / 0x06 | PC→MCU | — | lock / release |
| 0x07 | PC→MCU | u8 | laser |
| 0x08 / 0x09 | PC→MCU | u16 id (+ f32) | read / write parameter |
| 0x0A | PC→MCU | — | save parameters to flash |
| 0x0B | PC→MCU | u8 | motor driver enable |
| 0x0C | PC→MCU | u16 Hz | telemetry rate |
| 0x0D | PC→MCU | — | stop all axes |
| 0x0E | PC→MCU | — | clear fault |
| 0x81 | MCU→PC | u32 ms, u32 ver | pong |
| 0x82 | MCU→PC | telem_t 84 B | telemetry |
| 0x83 | MCU→PC | u16 id, f32, u8 ok | parameter value |
| 0x84 | MCU→PC | text | log line |
| 0x85 | MCU→PC | u8 id, u8 status | acknowledge |

Full command list: [MERCOM1_KOD_VE_PARAMETRELER.md](MERCOM1_KOD_VE_PARAMETRELER.md) (in Turkish).

## Repository layout

```
src/                 firmware (hw, drv, ctrl, app)
lib/Adafruit_BNO08x  BNO085 library (patched for this project, see MERCOM_YAMA.md)
tools/               desktop application, LoRa bridge and configuration scripts, hardware test programs and test scripts
docs/                README images
```

## Documentation (in Turkish)

- [MERCOM1_SISTEM_TAM_ACIKLAMA.md](MERCOM1_SISTEM_TAM_ACIKLAMA.md): full system description
- [MERCOM1_YAZILIM_ACIKLAMA.md](MERCOM1_YAZILIM_ACIKLAMA.md): how the software works
- [MERCOM1_EKSENLER_VE_KOD.md](MERCOM1_EKSENLER_VE_KOD.md): axes and code
- [MERCOM1_KOD_VE_PARAMETRELER.md](MERCOM1_KOD_VE_PARAMETRELER.md): parameters, protocol, usage
- [MERCOM1_OZELLIKLER.md](MERCOM1_OZELLIKLER.md): hardware, pin map, design decisions and test history
