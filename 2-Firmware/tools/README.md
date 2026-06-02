# Kia Arm — Host tools

## kia_cli.py

Interactive CLI for the Kia Arm controller, supports both **USB CDC** (text + binary)
and **BLE** transports.

### Install

```bash
pip install -r requirements.txt
```

### Usage

```bash
# USB serial
python kia_cli.py serial COM5
python kia_cli.py serial /dev/ttyACM0 -b 115200

# BLE (auto-scan for "KiaArm")
python kia_cli.py ble
python kia_cli.py ble --name MyKia
```

### Commands

Text protocol (line-based, parsed by `proto_text` on device):

```
PING
STATUS
J 0 30 1.5
XYZ 150 0 120 -30 2.0
MODE MANUAL|APP|HOLD
HOME / HOLD / ARM / CLEAR / SAVE
CAL J<i> ZERO|MIN|MAX
CAL P<i> MIN|MAX
CAL GEOM <base_h> <upper> <fore> <tool>
CAL LOAD | FACTORY
HELP
```

Binary helpers (encoded COBS+CRC32 frames):

```
bping
bjoint <i> <deg>
bxyz <x y z pitch>
bhome / bsave / barm / bclear
ota <firmware.bin>          # push OTA
tlm                         # print last decoded telemetry
```

Type `q` or `exit` to quit.

### Architecture

```
KiaClient
   ├─ Transport (Serial | BLE)  — bytes in/out
   ├─ Text frames   — newline terminated
   └─ Binary frames — COBS(0x00 delim) + [op|flags|seq|crc32|body]
```

Telemetry frames (opcode 0xF0) are decoded into a `Status` dataclass and
exposed via `client.last_telemetry`.
