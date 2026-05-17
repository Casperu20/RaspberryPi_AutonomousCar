#!/usr/bin/env python3
"""
Gyro-assisted turn test for Raspberry Pi mecanum robot.

Purpose:
- Test only motors + MPU6050.
- No ultrasonic.
- No autonomous logic.
- Robot should rotate left/right and stop when gyro yaw reaches target angle.

Hardware:
- Raspberry Pi 4B
- 2x TB6612FNG motor drivers
- 4x TT motors with mecanum wheels
- MPU6050 over I2C

Important:
- Uses your working mecanum rotate_left() and rotate_right() motor patterns.
- Uses Gyro Z as yaw axis, because your MPU axis test confirmed Z is correct.
"""

import time
from time import sleep

import RPi.GPIO as GPIO
import smbus

# ============================================================================
# GPIO CONFIGURATION
# ============================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ============================================================================
# MOTOR PINS
# ============================================================================

STBY_PIN = 20

FL_PINS = (27, 17, 18)  # Front Left
FR_PINS = (22, 23, 13)  # Front Right
RL_PINS = (25, 24, 12)  # Rear Left
RR_PINS = (6, 5, 19)    # Rear Right

MOTOR_FL_DIR = 1
MOTOR_FR_DIR = -1
MOTOR_RL_DIR = 1
MOTOR_RR_DIR = -1

# ============================================================================
# MPU6050 REGISTERS
# ============================================================================

MPU6050_ADDR = 0x68

PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# ============================================================================
# TEST SETTINGS
# ============================================================================

SPEED_ROTATE = 70        # Start safe. Later try 60 or 65.
TURN_ANGLE = 90       # Start safe. Later try 40, 45, 55.
MAX_TURN_TIME = 4.0      # Safety timeout.
GYRO_DEADBAND = 0.8      # Ignore tiny gyro noise.

YAW_AXIS = "z"           # Your test confirmed Z is correct.

# ============================================================================
# GLOBAL STATE
# ============================================================================

bus = smbus.SMBus(1)

gyro_offsets = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

yaw_angle = 0.0
yaw_last_time = None

pwm_fl = None
pwm_fr = None
pwm_rl = None
pwm_rr = None

cleaned_up = False

# ============================================================================
# GPIO / MOTOR SETUP
# ============================================================================

def setup_gpio():
    global pwm_fl, pwm_fr, pwm_rl, pwm_rr

    print("Initializing motor GPIO pins...")

    motor_pins = [
        STBY_PIN,
        *FL_PINS,
        *FR_PINS,
        *RL_PINS,
        *RR_PINS,
    ]

    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    pwm_fl = GPIO.PWM(FL_PINS[2], 1000)
    pwm_fr = GPIO.PWM(FR_PINS[2], 1000)
    pwm_rl = GPIO.PWM(RL_PINS[2], 1000)
    pwm_rr = GPIO.PWM(RR_PINS[2], 1000)

    pwm_fl.start(0)
    pwm_fr.start(0)
    pwm_rl.start(0)
    pwm_rr.start(0)

    GPIO.output(STBY_PIN, GPIO.HIGH)

    print("Motor GPIO initialized.")


def clamp_speed(speed):
    if speed < 0:
        return 0

    if speed > 100:
        return 100

    return speed


def set_motor(pins, pwm, direction, speed, dir_multiplier):
    speed = clamp_speed(speed)

    fwd_pin = pins[0]
    rev_pin = pins[1]

    final_direction = direction * dir_multiplier

    if final_direction == 1:
        GPIO.output(fwd_pin, GPIO.HIGH)
        GPIO.output(rev_pin, GPIO.LOW)
        pwm.ChangeDutyCycle(speed)

    elif final_direction == -1:
        GPIO.output(fwd_pin, GPIO.LOW)
        GPIO.output(rev_pin, GPIO.HIGH)
        pwm.ChangeDutyCycle(speed)

    else:
        GPIO.output(fwd_pin, GPIO.LOW)
        GPIO.output(rev_pin, GPIO.LOW)
        pwm.ChangeDutyCycle(0)


def rotate_right(speed=SPEED_ROTATE):
    """
    Your working mecanum rotate-right pattern.
    Do not change unless your motor test says it is wrong.
    """
    print("ACTION: Rotate Right")

    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)


def rotate_left(speed=SPEED_ROTATE):
    """
    Your working mecanum rotate-left pattern.
    Do not change unless your motor test says it is wrong.
    """
    print("ACTION: Rotate Left")

    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)


def stop():
    print("ACTION: Stop")

    set_motor(FL_PINS, pwm_fl, 0, 0, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 0, 0, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 0, 0, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 0, 0, MOTOR_RR_DIR)

# ============================================================================
# MPU6050 FUNCTIONS
# ============================================================================

def init_mpu():
    print("Initializing MPU6050...")

    # Full reset
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x80)
    sleep(0.2)

    # Wake up, use stable clock source
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


def calibrate_gyro(samples=300):
    print(f"Calibrating gyro over {samples} samples.")
    print("Keep the robot completely still...")

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

    print("Gyro calibration finished.")
    print(
        f"Offsets | "
        f"GX:{gyro_offsets['x']:.3f} "
        f"GY:{gyro_offsets['y']:.3f} "
        f"GZ:{gyro_offsets['z']:.3f}"
    )


def read_gyro_axis(axis):
    if axis == "x":
        return (read_word(GYRO_XOUT_H) / 131.0) - gyro_offsets["x"]

    if axis == "y":
        return (read_word(GYRO_YOUT_H) / 131.0) - gyro_offsets["y"]

    if axis == "z":
        return (read_word(GYRO_ZOUT_H) / 131.0) - gyro_offsets["z"]

    raise ValueError(f"Invalid YAW_AXIS: {axis}")


def reset_yaw():
    global yaw_angle, yaw_last_time

    yaw_angle = 0.0
    yaw_last_time = time.monotonic()


def update_yaw():
    """
    Integrate gyro Z over time.
    Returns:
    - yaw_angle in degrees
    - yaw_rate in deg/s
    """
    global yaw_angle, yaw_last_time

    now = time.monotonic()

    if yaw_last_time is None:
        yaw_last_time = now
        return yaw_angle, 0.0

    dt = now - yaw_last_time
    yaw_last_time = now

    yaw_rate = read_gyro_axis(YAW_AXIS)

    if abs(yaw_rate) < GYRO_DEADBAND:
        yaw_rate = 0.0

    # Protect against weird pauses causing one huge integration jump.
    if dt < 0 or dt > 0.2:
        dt = 0.0

    yaw_angle += yaw_rate * dt

    return yaw_angle, yaw_rate

# ============================================================================
# GYRO-ASSISTED TURN FUNCTIONS
# ============================================================================

def rotate_left_angle(target_angle=TURN_ANGLE, speed=SPEED_ROTATE):
    print()
    print(f"TURN TEST: Left target = {target_angle:.1f} degrees, speed = {speed}")

    reset_yaw()
    sleep(0.05)

    rotate_left(speed)

    start_time = time.monotonic()
    last_print_time = start_time
    timed_out = False

    while True:
        current_yaw, yaw_rate = update_yaw()
        now = time.monotonic()

        if now - last_print_time >= 0.10:
            print(
                f"Yaw Left | "
                f"raw:{current_yaw:8.2f} deg | "
                f"abs:{abs(current_yaw):8.2f} deg | "
                f"rate:{yaw_rate:8.2f} deg/s"
            )
            last_print_time = now

        if abs(current_yaw) >= target_angle:
            break

        if now - start_time >= MAX_TURN_TIME:
            timed_out = True
            break

        sleep(0.01)

    stop()

    if timed_out:
        print(
            f"TURN TIMEOUT: Left did not reach {target_angle:.1f} deg. "
            f"Final yaw = {abs(yaw_angle):.2f} deg"
        )
        return False

    print(f"TURN DONE: Left reached {abs(yaw_angle):.2f} deg")
    return True


def rotate_right_angle(target_angle=TURN_ANGLE, speed=SPEED_ROTATE):
    print()
    print(f"TURN TEST: Right target = {target_angle:.1f} degrees, speed = {speed}")

    reset_yaw()
    sleep(0.05)

    rotate_right(speed)

    start_time = time.monotonic()
    last_print_time = start_time
    timed_out = False

    while True:
        current_yaw, yaw_rate = update_yaw()
        now = time.monotonic()

        if now - last_print_time >= 0.10:
            print(
                f"Yaw Right | "
                f"raw:{current_yaw:8.2f} deg | "
                f"abs:{abs(current_yaw):8.2f} deg | "
                f"rate:{yaw_rate:8.2f} deg/s"
            )
            last_print_time = now

        if abs(current_yaw) >= target_angle:
            break

        if now - start_time >= MAX_TURN_TIME:
            timed_out = True
            break

        sleep(0.01)

    stop()

    if timed_out:
        print(
            f"TURN TIMEOUT: Right did not reach {target_angle:.1f} deg. "
            f"Final yaw = {abs(yaw_angle):.2f} deg"
        )
        return False

    print(f"TURN DONE: Right reached {abs(yaw_angle):.2f} deg")
    return True

# ============================================================================
# CLEANUP
# ============================================================================

def cleanup_and_exit():
    global cleaned_up

    if cleaned_up:
        return

    print("\nCleaning up...")

    try:
        if pwm_fl is not None:
            stop()
            sleep(0.1)

            pwm_fl.stop()
            pwm_fr.stop()
            pwm_rl.stop()
            pwm_rr.stop()

        GPIO.output(STBY_PIN, GPIO.LOW)
        GPIO.cleanup()

    except Exception as e:
        print(f"Cleanup warning: {e}")

    cleaned_up = True
    print("System halted.")

# ============================================================================
# MAIN TEST
# ============================================================================

try:
    setup_gpio()

    init_mpu()
    calibrate_gyro(300)

    print()
    print("Gyro-assisted turn test ready.")
    print("Place the robot flat on the floor with free space around it.")
    print("Do not hold or lift the robot during the turn.")
    print("Keep your hand ready to stop it if needed.")
    print()
    print(f"Settings: TURN_ANGLE={TURN_ANGLE}, SPEED_ROTATE={SPEED_ROTATE}, MAX_TURN_TIME={MAX_TURN_TIME}")
    print()

    input("Press Enter to test LEFT turn...")

    left_ok = rotate_left_angle(TURN_ANGLE, SPEED_ROTATE)

    sleep(1)

    input("\nPress Enter to test RIGHT turn...")

    right_ok = rotate_right_angle(TURN_ANGLE, SPEED_ROTATE)

    print()
    print("========== TEST RESULT ==========")
    print(f"Left turn:  {'PASS' if left_ok else 'FAIL'}")
    print(f"Right turn: {'PASS' if right_ok else 'FAIL'}")

    if left_ok and right_ok:
        print("Gyro-assisted turning works.")
    else:
        print("Gyro-assisted turning needs tuning.")
        print("Check if yaw increases while the robot rotates.")
        print("If yaw increases but timeout happens, lower TURN_ANGLE or raise MAX_TURN_TIME.")
        print("If yaw stays near zero, check YAW_AXIS and MPU wiring/orientation.")

except KeyboardInterrupt:
    print("\nManual stop triggered.")

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    cleanup_and_exit() 