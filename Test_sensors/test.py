#!/usr/bin/env python3

import smbus
import time

# =========================================================
# MPU6050 REGISTERS
# =========================================================
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
# Skipping 0x3F (Z-Accel) as it is defective

# Skipping ACCEL_ZOUT_H at 0x3F because accelerometer Z is defective.
# Gyro Z at 0x47 is separate and still used.

GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# =========================================================
# GLOBAL OFFSETS
# =========================================================
accel_offsets = {'x': 0, 'y': 0}
gyro_offsets  = {'x': 0, 'y': 0, 'z': 0}

bus = smbus.SMBus(1)

def init_mpu():
    """Hard reset and configuration sequence."""
    try:
        # Full internal reset
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x80)
        time.sleep(0.2)
        
        # Wake up and set stable clock source
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        
        # Set ranges (+/- 2g and +/- 250 deg/s)
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)
        
        print("MPU6050: System Reset & Initialized.")
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")

def read_word(reg):
    """Reads 16-bit word from I2C."""
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def calibrate_sensors(samples=200):
    """Calculates error offsets while stationary."""
    global accel_offsets, gyro_offsets
    print(f"Calibrating over {samples} samples. DO NOT MOVE...")
    
    sum_ax, sum_ay = 0, 0
    sum_gx, sum_gy, sum_gz = 0, 0, 0
    
    for _ in range(samples):
        sum_ax += read_word(ACCEL_XOUT_H) / 16384.0
        sum_ay += read_word(ACCEL_YOUT_H) / 16384.0
        
        sum_gx += read_word(GYRO_XOUT_H) / 131.0
        sum_gy += read_word(GYRO_YOUT_H) / 131.0
        sum_gz += read_word(GYRO_ZOUT_H) / 131.0
        time.sleep(0.005)
        
    accel_offsets['x'] = sum_ax / samples
    accel_offsets['y'] = sum_ay / samples
    
    gyro_offsets['x'] = sum_gx / samples
    gyro_offsets['y'] = sum_gy / samples
    gyro_offsets['z'] = sum_gz / samples
    
    print("Calibration Finished.")

def get_clean_data():
    """Returns X/Y Accel and X/Y/Z Gyro (Skipping broken Z-Accel)."""
    # Read Accel
    ax = (read_word(ACCEL_XOUT_H) / 16384.0) - accel_offsets['x']
    ay = (read_word(ACCEL_YOUT_H) / 16384.0) - accel_offsets['y']
    
    # Read Gyro
    gx = (read_word(GYRO_XOUT_H) / 131.0) - gyro_offsets['x']
    gy = (read_word(GYRO_YOUT_H) / 131.0) - gyro_offsets['y']
    gz = (read_word(GYRO_ZOUT_H) / 131.0) - gyro_offsets['z']
    
    return ax, ay, gx, gy, gz

# =========================================================
# MAIN LOOP
# =========================================================
try:
    init_mpu()
    calibrate_sensors(200)
    
    while True:
        ax, ay, gx, gy, gz = get_clean_data()

        # Visualizing the functional data
        print(f"ACCEL [g]   X: {ax:6.2f}  Y: {ay:6.2f}")
        print(f"GYRO [°/s]  X: {gx:6.2f}  Y: {gy:6.2f}  Z: {gz:6.2f}")
        print("-" * 45)

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopping IMU Stream...")