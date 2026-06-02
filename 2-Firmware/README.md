# Kia Arm Controller — Firmware

ESP32-S3 firmware for the 4-axis + gripper robotic arm controller (PCB `Kia-Arm-Ctrl v1`).

## Stack

- **SDK**: ESP-IDF v5.3+
- **OS**: FreeRTOS (1 kHz tick, dual-core)
- **Language**: C11 + selective C++17 (no exceptions / no RTTI)
- **Build**: CMake via `idf.py`
- **Toolchain**: xtensa-esp32s3-elf-gcc 13+

## Architecture

```
app/        ModeFSM, Calibration
domain/     Kinematics (FK/IK 4DOF), JointModel
services/   Motion, Safety, CommRouter, Storage, Telemetry
hal/drv/    Servo (MCPWM), Pot (ADC continuous DMA), INA219, LED (RMT WS2812),
            Buttons (debounce), Buzzer (LEDC)
common/     err, log, ringbuf (lock-free SPSC), COBS, CRC32
```

### Real-time partitioning

| Core | Tasks                                                                 |
|------|-----------------------------------------------------------------------|
| 1 (APP_CPU) | `motion` (1 kHz), `safety` (200 Hz), `pot` (ADC DMA)            |
| 0 (PRO_CPU) | `usb_rx`, `mode_fsm`, `telemetry`, `btn`, `event_loop`          |

Task priorities are defined in `main/app_config.h` (`KIA_PRIO_*`).
Safety has the highest priority and never blocks on I/O.

### Event bus

All cross-component coordination goes through `esp_event_loop` on `KIA_EVENT_BASE`
(see `main/app_events.h`). No globals, no implicit coupling.

## Build / Flash / Monitor

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py menuconfig          # adjust if needed
idf.py -p COMx flash monitor
```

To clean:
```bash
idf.py fullclean
```

## Wire protocols

### Text (debug / CLI)

Line-terminated ASCII. Commands :

```
PING                                  → PONG fw=1.0.0
STATUS                                → STATUS J=[...] XYZP=[...] I=... FAULT=... SRC=...
J <id> <deg> [duration]               → OK | ERR ...
XYZ <x> <y> <z> <pitch> [duration]    → OK | ERR ...
MODE MANUAL|APP|HOLD                  → OK
HOME                                  → OK
HOLD                                  → OK
ARM                                   → OK | ERR
CLEAR                                 → OK | ERR
SAVE                                  → OK
HELP
```

### Binary (app, COBS + CRC32)

Frames terminated by `0x00`. Body (after COBS decode):

```
struct frame {
  u8   op;
  u8   flags;
  u16  seq;
  u32  crc32;       /* CRC of op|flags|seq|payload */
  u8   payload[];
}
```

Opcodes TBD — currently the dispatcher is a stub. Extension point in
`components/svc_comm/proto_binary.c`.

## Safety model

1. **HW E-STOP** : `GPIO40` drives the P-MOSFET that gates `+VSERVO`.
   Independent of firmware liveness.
2. **HW E-STOP IRQ** : `GPIO18` falling edge → kill rail + raise `FAULT`.
3. **INA219 supervisor** : 200 Hz. Trips on instantaneous I > 8 A or
   sustained average > 5 A.
4. **Brownout detector** : hardware level 7.
5. **Task WDT** : 3 s, registered on safety task.
6. **Soft joint limits** : enforced in `dom_joint_clamp` and in motion controller.

To clear a soft fault: `CLEAR`. To clear E-STOP: release the physical button.

## Calibration

Stored in NVS namespace `kia_cal`. Per-servo: offset, pulse min/max,
inversion, angle window. Per-pot: raw min/max.

Capture from CLI :
```
J 0 0          # drive joint 0 to where you want zero
# then via dedicated calibration cmd (TODO: CAL J0 ZERO)
SAVE           # persist to NVS
```

## File layout

```
firmware/
├── CMakeLists.txt
├── sdkconfig.defaults
├── partitions.csv
├── main/
│   ├── main.c
│   ├── app_config.h         ← single source of truth for pins, limits, rates
│   └── app_events.h
├── components/
│   ├── common/              ← err, log, ringbuf, COBS, CRC32
│   ├── drv_servo/           ← MCPWM, 5 channels
│   ├── drv_pot/             ← ADC continuous + EMA + adaptive hysteresis
│   ├── drv_ina219/          ← I²C current monitor
│   ├── drv_led/             ← WS2812 (RMT) + status/fault GPIOs
│   ├── drv_buttons/         ← E-STOP + mode (ISR + debounce)
│   ├── drv_buzzer/          ← LEDC PWM tones
│   ├── dom_joint/           ← per-joint spec + state + limit clamp
│   ├── dom_kinematics/      ← FK + IK 4DOF closed-form
│   ├── svc_safety/          ← supervisor + fault FSM
│   ├── svc_motion/          ← 1 kHz controller + quintic trajectory
│   ├── svc_comm/            ← router + USB CDC + text + binary protocols
│   ├── svc_storage/         ← NVS wrappers, calibration persistence
│   ├── svc_telemetry/       ← periodic broadcast on all transports
│   ├── app_mode_fsm/        ← Boot → Idle → Manual/App/Calib/Fault/E-STOP
│   └── app_calibration/     ← capture routines
└── tests/                   ← unit tests (Unity, TODO)
```

## Coding conventions

- snake_case, header guards via `#pragma once`
- static for everything not in the public header
- no globals; module state held in a single static struct
- error type : `esp_err_t` augmented by `kia_err_t` codes
- macros : `KIA_CHK(expr)` aborts on error (init time), `KIA_RET(expr)` returns
- clang-format provided

## Status (firmware roadmap)

- [x] BLE NimBLE GATT transport (custom service, RX write + TX notify)
- [x] Binary opcode dispatch (system / motion / mode / storage / cal / OTA)
- [x] CAL CLI subcommands (`CAL J<i> ZERO|MIN|MAX`, `CAL P<i> MIN|MAX`, `CAL GEOM`)
- [x] OTA over USB CDC / BLE (`svc_ota` + opcodes 0x50–0x53)
- [x] Trajectory queue (`move_queue`, FreeRTOS-safe ring)
- [x] Jerk-limited S-curve profile (`traj_scurve_*`, switchable at runtime)
- [x] Cartesian straight-line path (`cart_path_plan_line`, subdivision + per-segment IK)
- [x] Unit tests Unity (kinematics, COBS, CRC32, ringbuf, trajectory) — `tests/`
- [x] Host CLI Python (`tools/kia_cli.py` — serial + BLE, text + binary + OTA push)
- [ ] PID closed-loop (waiting for joint encoders hardware)
- [ ] BLE pairing / bonding for production
- [ ] Web-based telemetry dashboard

## Wire protocol — binary opcodes

| Op   | Direction | Body (LE) |
|------|-----------|-----------|
| 0x01 PING        | req | — |
| 0x81 PONG        | rsp | u8 maj, u8 min, u8 patch |
| 0x02 GET_STATUS  | req | — |
| 0x82 STATUS      | rsp | 4×f32 joints, f32 grip, 3×f32 xyz, f32 pitch, u32 cur_ma, u8 fault, u8 src, u8 idle, u8 _ |
| 0x10 SET_JOINT   | req | u8 idx, f32 deg, f32 dur |
| 0x11 SET_JOINTS  | req | 4×f32 joints, f32 grip, f32 dur |
| 0x12 SET_XYZ     | req | 4×f32 pose, f32 dur |
| 0x13 HOME / 0x14 HOLD | req | — |
| 0x20 MODE_SET    | req | u8 mode |
| 0x21 ARM / 0x22 CLEAR | req | — |
| 0x30 SAVE / 0x31 LOAD / 0x3F FACTORY | req | — |
| 0x50 OTA_BEGIN   | req | u32 total, u16 chunk, u32 crc32 |
| 0x51 OTA_CHUNK   | req | u32 offset, bytes data |
| 0x52 OTA_END / 0x53 OTA_ABORT | req | — |
| 0xF0 TELEMETRY   | async | same payload as STATUS |
| 0xFE ACK / 0xFF NACK | rsp | (NACK) u8 reason |

NACK reasons: 1=BAD_OP 2=BAD_LEN 3=BAD_ARG 4=LIMIT 5=UNREACHABLE 6=FAULT 7=BUSY 8=NOMEM 9=BAD_STATE 0xFE=NOT_IMPL 0xFF=INTERNAL

## BLE service

| Item | Value |
|------|-------|
| Service UUID  | custom 128-bit (see `transport_ble.c`) |
| RX charac     | write / write-no-rsp — host → device frames |
| TX charac     | notify — device → host frames |
| Device name   | `KiaArm` |
