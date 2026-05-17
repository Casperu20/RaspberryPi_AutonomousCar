#!/usr/bin/env python3

import smbus
import time
from time import sleep
''''
========== RESULT ==========
Axis X | Max rate:    26.38 deg/s | Net angle:     4.57 deg | Total motion:     6.54 deg
Axis Y | Max rate:    35.56 deg/s | Net angle:    -0.73 deg | Total motion:     4.75 deg
Axis Z | Max rate:    38.30 deg/s | Net angle:  -108.59 deg | Total motion:   111.31 deg

Recommended yaw axis: Z
Set YAW_AXIS = "z" in the autonomous code.
'''
# =========================================================
# MPU6050 REGISTERS
# =========================================================

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1)

gyro_offsets = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

DEADBAND = 1.2
TEST_DURATION = 8.0


def init_mpu():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x80)
    sleep(0.2)

    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    sleep(0.1)

    # +/- 2g accel, +/- 250 deg/s gyro
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)

    print("MPU6050 initialized.")


def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)

    value = (high << 8) + low

    if value >= 0x8000:
        value = -((65535 - value) + 1)

    return value


def calibrate(samples=300):
    print("Calibrating gyro. Keep robot completely still...")

    sum_gx = 0.0
    sum_gy = 0.0
    sum_gz = 0.0

    for _ in range(samples):
        sum_gx += read_word(GYRO_XOUT_H) / 131.0
        sum_gy += read_word(GYRO_YOUT_H) / 131.0
        sum_gz += read_word(GYRO_ZOUT_H) / 131.0
        sleep(0.005)

    gyro_offsets["x"] = sum_gx / samples
    gyro_offsets["y"] = sum_gy / samples
    gyro_offsets["z"] = sum_gz / samples

    print("Calibration done.")
    print(
        f"Offsets | "
        f"GX:{gyro_offsets['x']:.3f} "
        f"GY:{gyro_offsets['y']:.3f} "
        f"GZ:{gyro_offsets['z']:.3f}"
    )


def read_gyro():
    gx = (read_word(GYRO_XOUT_H) / 131.0) - gyro_offsets["x"]
    gy = (read_word(GYRO_YOUT_H) / 131.0) - gyro_offsets["y"]
    gz = (read_word(GYRO_ZOUT_H) / 131.0) - gyro_offsets["z"]

    return {
        "x": gx,
        "y": gy,
        "z": gz,
    }


def apply_deadband(value):
    if abs(value) < DEADBAND:
        return 0.0

    return value


try:
    init_mpu()
    calibrate(300)

    print("\nIMPORTANT:")
    print("Put the robot flat on the floor or table.")
    print("Do NOT lift it.")
    print("Rotate it left and right while keeping it flat.")
    print("Imagine turning the robot like a car rotating in place.")
    print()
    input("Press Enter, then rotate the robot flat for 8 seconds...")

    stats = {
        "x": {
            "max_rate": 0.0,
            "net_angle": 0.0,
            "total_motion": 0.0,
        },
        "y": {
            "max_rate": 0.0,
            "net_angle": 0.0,
            "total_motion": 0.0,
        },
        "z": {
            "max_rate": 0.0,
            "net_angle": 0.0,
            "total_motion": 0.0,
        },
    }

    start_time = time.monotonic()
    last_time = start_time
    last_print = start_time

    while True:
        now = time.monotonic()

        if now - start_time >= TEST_DURATION:
            break

        dt = now - last_time
        last_time = now

        gyro = read_gyro()

        for axis in ["x", "y", "z"]:
            rate = apply_deadband(gyro[axis])

            stats[axis]["net_angle"] += rate * dt
            stats[axis]["total_motion"] += abs(rate) * dt
            stats[axis]["max_rate"] = max(
                stats[axis]["max_rate"],
                abs(gyro[axis]),
            )

        if now - last_print >= 0.25:
            print(
                f"GX:{gyro['x']:8.2f} | "
                f"GY:{gyro['y']:8.2f} | "
                f"GZ:{gyro['z']:8.2f}"
            )
            last_print = now

        sleep(0.01)

    print("\n========== RESULT ==========")

    for axis in ["x", "y", "z"]:
        print(
            f"Axis {axis.upper()} | "
            f"Max rate: {stats[axis]['max_rate']:8.2f} deg/s | "
            f"Net angle: {stats[axis]['net_angle']:8.2f} deg | "
            f"Total motion: {stats[axis]['total_motion']:8.2f} deg"
        )

    best_axis = max(
        ["x", "y", "z"],
        key=lambda axis: stats[axis]["total_motion"],
    )

    print()
    print(f"Recommended yaw axis: {best_axis.upper()}")
    print(f'Set YAW_AXIS = "{best_axis}" in the autonomous code.')

except KeyboardInterrupt:
    print("\nStopped.")