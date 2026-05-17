#!/usr/bin/env python3
"""
Mecanum Wheel Robot Control - Local Console
Raspberry Pi 4B with 2x TB6612FNG Motor Drivers and 4x TT DC Motors

Refined version:
- Forward
- Backward
- Rotate left
- Rotate right
- Stop

Strafing is disabled because on this robot the old strafe commands produced
the correct rotation behavior.
"""

import RPi.GPIO as GPIO
from time import sleep

# ============================================================================
# GPIO CONFIGURATION
# ============================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Standby enable pin shared by both motor drivers
STBY_PIN = 20

# Motor pin assignments: (forward_pin, reverse_pin, pwm_pin)
FL_PINS = (27, 17, 18)  # Front Left
FR_PINS = (22, 23, 13)  # Front Right
RL_PINS = (25, 24, 12)  # Rear Left
RR_PINS = (6, 5, 19)    # Rear Right

# ============================================================================
# MOTOR DIRECTION MULTIPLIERS
# ============================================================================

MOTOR_FL_DIR = 1
MOTOR_FR_DIR = -1
MOTOR_RL_DIR = 1
MOTOR_RR_DIR = -1

# ============================================================================
# SPEED SETTINGS
# ============================================================================

SPEED_FORWARD_BACKWARD = 70
SPEED_ROTATE = 70

# ============================================================================
# GPIO SETUP
# ============================================================================

print("Initializing GPIO pins...")

all_pins = [STBY_PIN] + list(FL_PINS) + list(FR_PINS) + list(RL_PINS) + list(RR_PINS)

for pin in all_pins:
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

print("GPIO initialized and motor drivers enabled.")

# ============================================================================
# MOTOR CONTROL FUNCTION
# ============================================================================

def clamp_speed(speed):
    """Keep PWM speed between 0 and 100."""
    if speed < 0:
        return 0
    if speed > 100:
        return 100
    return speed


def set_motor(pins, pwm, direction, speed, dir_multiplier):
    """
    Control a single motor.

    direction:
         1 = one direction
        -1 = opposite direction
         0 = stop
    """
    speed = clamp_speed(speed)

    fwd_pin, rev_pin = pins[0], pins[1]
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


# ============================================================================
# MOVEMENT FUNCTIONS
# ============================================================================

def forward(speed=SPEED_FORWARD_BACKWARD):
    """Move forward."""
    print("Forward")

    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)


def backward(speed=SPEED_FORWARD_BACKWARD):
    """Move backward."""
    print("Backward")

    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)


def rotate_right(speed=SPEED_ROTATE):
    print("Rotate Right")

    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)


def rotate_left(speed=SPEED_ROTATE):
    print("Rotate Left")

    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)


def stop():
    """Stop all motors."""
    print("Stop!")

    set_motor(FL_PINS, pwm_fl, 0, 0, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 0, 0, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 0, 0, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 0, 0, MOTOR_RR_DIR)


# ============================================================================
# CLEANUP FUNCTION
# ============================================================================

cleaned_up = False

def cleanup_and_exit():
    """Safely shut down the robot and clean up resources."""
    global cleaned_up

    if cleaned_up:
        return

    print("Cleaning up GPIO...")

    try:
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
# MAIN CONTROL LOOP
# ============================================================================

print("\nRobot Control Active!")
print("Controls:")
print("  w = Forward")
print("  s = Backward")
print("  a = Rotate Left")
print("  d = Rotate Right")
print("  x = Stop")
print("  (Exit with Ctrl+C)\n")

try:
    while True:
        command = input("Command: ").lower().strip()

        if command == 'w':
            forward()

        elif command == 's':
            backward()

        elif command == 'a':
            rotate_left()

        elif command == 'd':
            rotate_right()


        elif command == 'x':
            stop()

        elif command == '':
            continue

        else:
            print("Invalid command!")

except KeyboardInterrupt:
    print("\nManual stop triggered.")

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    cleanup_and_exit()