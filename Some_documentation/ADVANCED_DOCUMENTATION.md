# Advanced Documentation (Current System)

## 1. Overview

This repository currently implements a mecanum robot stack with:
- 2x TB6612FNG motor drivers
- 4x TT motors
- 3x HC-SR04 ultrasonic sensors
- MPU6050 (I2C)
- Camera stream and YOLO-based vision stream

The current autonomous behavior is implemented in:
- `Mapping/autonomous_grid_mapper.py`

Manual drive behavior is implemented in:
- `bt_control.py`
- `pc_control.py`

## 2. Real Project Components

### 2.1 Motor Control

Implemented in both `bt_control.py` and `pc_control.py` using:
- STBY: GPIO20
- FL: GPIO27, GPIO17, PWM GPIO18
- FR: GPIO22, GPIO23, PWM GPIO13
- RL: GPIO25, GPIO24, PWM GPIO12
- RR: GPIO6, GPIO5, PWM GPIO19

Direction multipliers are used for calibration and differ between scripts.

### 2.2 Ultrasonic Sensors

Current stable layout is 3 independent triggers:
- Front: TRIG GPIO16, ECHO GPIO26
- Left: TRIG GPIO4, ECHO GPIO14
- Right: TRIG GPIO15, ECHO GPIO8

This independent-trigger layout is implemented in:
- `Mapping/autonomous_grid_mapper.py`
- `Test_sensors/test_ultra.py`
- `robot_tests.py/test_ultrasonic_only.py`

### 2.3 MPU6050

I2C pins:
- SDA GPIO2
- SCL GPIO3

Used in:
- `Test_sensors/test.py`
- `robot_tests.py/test_mpu_axes.py`
- `Mapping/autonomous_grid_mapper.py`

Current usage in mapper focuses on gyro-based yaw and calibration offsets.

### 2.4 Mapping Runtime

`Mapping/autonomous_grid_mapper.py` provides:
- Discrete occupancy grid
- Heading-aware cell updates
- One-cell movement strategy
- Gyro-assisted turning
- JSON + ASCII map export

Outputs:
- `Mapping/robot_grid_map.json`
- `Mapping/robot_grid_map.txt`

### 2.5 Vision Runtime

- `camera_stream.py`: MJPEG live stream
- `camera_model.py`: YOLO detections + overlays

Status:
- Vision is operational as a separate stream pipeline.
- It is not yet fused with the autonomous grid mapper.

## 3. What Is Still Needed

To reach a robust final autonomous exploration + mapping + sensing workflow, the remaining high-value tasks are:

1. Unify navigation and vision pipelines.
2. Standardize ultrasonic pin model across all scripts (camera model currently uses single trigger logic).
3. Add robust state machine for mode switching (manual, mapping, vision-assisted).
4. Add repeatable map quality metrics and calibration procedure.
5. Add environmental sensor sampling layer (for heatmap-style projects).
6. Add persistence/versioning for map + sensor datasets.

## 4. Known Mismatches Resolved In Docs

The documentation now reflects that:
- `autonomous_v2.py` and `autonomous.py` are not current runtime files in this repository.
- Active autonomous script is `Mapping/autonomous_grid_mapper.py`.
- Test scripts are in `Test_sensors` and `robot_tests.py`.
- Wiring uses 3 ultrasonic triggers, not a shared trigger for all sensors.

## 5. Recommended Validation Order

Run from `AutonomousCAR`:

```bash
sudo python3 Test_sensors/test_system.py
```

```bash
sudo python3 Test_sensors/test_ultra.py
```

```bash
sudo python3 Test_sensors/test.py
```

```bash
cd Mapping
sudo python3 autonomous_grid_mapper.py
```

Optional control checks:

```bash
sudo python3 pc_control.py
```

```bash
sudo python3 bt_control.py
```

## 6. Reliability Notes

- Keep ultrasonic trigger and echo wires short and separated from motor power wires.
- Keep I2C wires short and stable; verify MPU address `0x68`.
- Use voltage dividers on ultrasonic echo lines to protect 3.3V GPIO input.
- Always run tests before autonomous mapping after wiring changes.
