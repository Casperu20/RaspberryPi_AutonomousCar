# Hardware Wiring Guide - Mecanum Robot (Current)

## 1. Hardware Inventory

- 2x TB6612FNG motor drivers
- 4x TT motors (FL, FR, RL, RR)
- 4x mecanum wheels
- 3x HC-SR04 ultrasonic sensors
- 1x MPU6050 (I2C)
- 1x camera module

## 2. Canonical GPIO Mapping (Authoritative)

### Motor Driver + Motors

- STBY: GPIO20
- Front Left: GPIO27, GPIO17, PWM GPIO18
- Front Right: GPIO22, GPIO23, PWM GPIO13
- Rear Left: GPIO25, GPIO24, PWM GPIO12
- Rear Right: GPIO6, GPIO5, PWM GPIO19

### Ultrasonic Sensors (Independent TRIG per Sensor)

- Front: TRIG GPIO16, ECHO GPIO26
- Left: TRIG GPIO4, ECHO GPIO14
- Right: TRIG GPIO15, ECHO GPIO8

### MPU6050 (I2C)

- SDA: GPIO2
- SCL: GPIO3
- I2C address: 0x68

## 3. Critical Correction

This project now uses **3 separate trigger pins** for ultrasonic sensors.

Do not wire all ultrasonic sensors to a single trigger when following the current mapping and test scripts.

## 4. Power and Signal Safety

- Use voltage dividers on each ultrasonic ECHO line before Raspberry Pi GPIO.
- Keep motor power wiring separated from signal wiring.
- Use common ground between Pi, drivers, and sensors.
- Keep I2C wiring short and stable.
- Keep the STBY pin (GPIO20) controlled by software; motors should be disabled at startup until initialized.

## 5. Software-to-Wiring Consistency

### Files that match this pinout

- `bt_control.py`
- `pc_control.py`
- `Mapping/autonomous_grid_mapper.py`
- `Test_sensors/test_system.py`
- `Test_sensors/test_ultra.py`
- `robot_tests.py/test_ultrasonic_only.py`

### File needing pin-model alignment before full fusion

- `camera_model.py`
  - Uses a single trigger variable for ultrasonic reads.
  - Needs update to independent trigger-per-sensor model if combined with mapper assumptions.

## 6. Verification Steps

1. Run full hardware test:

```bash
sudo python3 Test_sensors/test_system.py
```

2. Run ultrasonic-only test:

```bash
sudo python3 Test_sensors/test_ultra.py
```

3. Run MPU6050 check:

```bash
sudo python3 Test_sensors/test.py
```

4. Run mapper test:

```bash
cd Mapping
sudo python3 autonomous_grid_mapper.py
```

## 7. Current Missing Hardware for Higher-Accuracy Mapping (Optional)

For better localization quality than timed movement:
- Wheel encoders (strongly recommended)
- Or visual markers / external localization

The current configuration can produce functional occupancy mapping, but drift over longer paths is expected without encoder odometry.
