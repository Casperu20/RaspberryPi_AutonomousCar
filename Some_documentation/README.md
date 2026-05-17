# AutonomousCAR Documentation Index (Current)

This folder was cleaned to match the current repository state.

## What Is Current

The active codebase is centered on:
- `bt_control.py` (Bluetooth remote drive with mecanum motions)
- `pc_control.py` (local keyboard/console drive)
- `Mapping/autonomous_grid_mapper.py` (autonomous mapping with ultrasonic + MPU6050)
- `Test_sensors/test_system.py` (full hardware validation)
- `Test_sensors/test_ultra.py` and `Test_sensors/test.py` (focused sensor checks)
- `camera_stream.py` and `camera_model.py` (camera-only and YOLO stream)

## Read In This Order

1. `HARDWARE_WIRING_GUIDE.md`
   - Final pin map and wiring consistency notes.
2. `ADVANCED_DOCUMENTATION.md`
   - Real architecture, what is implemented, and what is still missing.

## Quick Run Commands

From `AutonomousCAR`:

```bash
sudo python3 Test_sensors/test_system.py
```

```bash
sudo python3 bt_control.py
```

```bash
sudo python3 pc_control.py
```

```bash
cd Mapping
sudo python3 autonomous_grid_mapper.py
```

## Important Notes

- The documented ultrasonic wiring is **3 independent trigger pins**:
  - Front: TRIG 16, ECHO 26
  - Left: TRIG 4, ECHO 14
  - Right: TRIG 15, ECHO 8
- Mapping code follows this 3-trigger layout.
- Camera YOLO script currently still reads ultrasonic with a single trigger logic; this is a known integration mismatch to fix before merging camera + mapping behavior.
- MPU6050 usage in project code intentionally avoids unreliable accel Z in some scripts and keeps gyro Z for yaw.

## Scope Clarification

This project currently has:
- Reliable motor control
- Reliable ultrasonic and MPU6050 testing
- A working autonomous grid mapper
- Camera streaming and YOLO visualization

This project does **not** yet have:
- Unified navigation + YOLO + map fusion in one runtime
- Encoder-based odometry
- True SLAM-quality localization
- Air-quality sensor acquisition pipeline
