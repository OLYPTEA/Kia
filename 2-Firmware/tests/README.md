# Kia Arm — Unit tests

ESP-IDF Unity test app. Runs on ESP32-S3 hardware or QEMU.

## Build & run

```bash
cd firmware/tests
idf.py set-target esp32s3
idf.py build flash monitor
```

Filter by tag (e.g. only kinematics):

```
[kin]
[cobs]
[crc32]
[ringbuf]
[traj]
```

Type the filter (with brackets) in the monitor prompt after Unity menu.
