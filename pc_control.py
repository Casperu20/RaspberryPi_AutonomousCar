#!/usr/bin/env python3
"""
Mecanum Wheel Robot Control - Local Console
Raspberry Pi 4B with 2x TB6612FNG Motor Drivers and 4x TT DC Motors
"""

import RPi.GPIO as GPIO
from time import sleep
import sys

# ============================================================================
# GPIO CONFIGURATION
# ============================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Standby (enable) pin shared by both motor drivers
STBY_PIN = 20  # TB6612FNG common STBY

# Motor pin assignments: (forward_pin, reverse_pin, pwm_pin)
FL_PINS = (27, 17, 18)  # Front Left
FR_PINS = (22, 23, 13)  # Front Right
RL_PINS = (25, 24, 12)  # Rear Left
RR_PINS = (6, 5, 19)    # Rear Right

# ============================================================================
# MOTOR DIRECTION MULTIPLIERS (CALIBRATION)
# ============================================================================
# Use these to invert motor directions without rewiring.
# Set to -1 to flip a motor's direction, 1 for normal.
# If strafe left/right moves forward/backward, try flipping one or more
# of these multipliers. Common fix: flip FL and RR, or flip FR and RL.

MOTOR_FL_DIR = 1   # Front Left direction multiplier
MOTOR_FR_DIR = -1  # Front Right direction multiplier
MOTOR_RL_DIR = 1   # Rear Left direction multiplier
MOTOR_RR_DIR = -1  # Rear Right direction multiplier

# ============================================================================
# SPEED SETTINGS
# ============================================================================
SPEED_FORWARD_BACKWARD = 60  # Speed for forward/backward movement
SPEED_STRAFE = 70            # Speed for strafe left/right
SPEED_ROTATE = 60            # Speed for rotation

# ============================================================================
# GPIO SETUP
# ============================================================================

print("Initializing GPIO pins...")

all_pins = [STBY_PIN] + list(FL_PINS) + list(FR_PINS) + list(RL_PINS) + list(RR_PINS)

# Set initial state to LOW to prevent glitches during power-on
for pin in all_pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# Create PWM objects for motor enable pins (frequency = 1000 Hz)
pwm_fl = GPIO.PWM(FL_PINS[2], 1000)
pwm_fr = GPIO.PWM(FR_PINS[2], 1000)
pwm_rl = GPIO.PWM(RL_PINS[2], 1000)
pwm_rr = GPIO.PWM(RR_PINS[2], 1000)

# Start PWM with 0% duty cycle
pwm_fl.start(0)
pwm_fr.start(0)
pwm_rl.start(0)
pwm_rr.start(0)

# Enable the motor drivers
GPIO.output(STBY_PIN, GPIO.HIGH)

print("GPIO initialized and motor drivers enabled.")

# ============================================================================
# MOTOR CONTROL FUNCTION
# ============================================================================

def set_motor(pins, pwm, direction, speed, dir_multiplier):
    """
    Control a single motor.
    
    Args:
        pins: (forward_pin, reverse_pin, pwm_pin) tuple
        pwm: PWM object for the enable pin
        direction: 1 for forward, -1 for backward, 0 for stop
        speed: PWM duty cycle (0-100)
        dir_multiplier: Direction multiplier (1 or -1) for calibration
    """
    fwd_pin, rev_pin = pins[0], pins[1]
    final_direction = direction * dir_multiplier
    
    if final_direction == 1:
        GPIO.output(fwd_pin, GPIO.HIGH)
        GPIO.output(rev_pin, GPIO.LOW)
    elif final_direction == -1:
        GPIO.output(fwd_pin, GPIO.LOW)
        GPIO.output(rev_pin, GPIO.HIGH)
    else:  # direction == 0
        GPIO.output(fwd_pin, GPIO.LOW)
        GPIO.output(rev_pin, GPIO.LOW)
    
    pwm.ChangeDutyCycle(speed)

# ============================================================================
# MOVEMENT FUNCTIONS
# ============================================================================

def forward(speed=SPEED_FORWARD_BACKWARD):
    """Move forward - all wheels forward."""
    print("Forward")
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)

def backward(speed=SPEED_FORWARD_BACKWARD):
    """Move backward - all wheels backward."""
    print("Backward")
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)

def strafe_left(speed=SPEED_STRAFE):
    """
    Strafe left - typical mecanum configuration.
    If this moves forward/backward instead of left, try:
    - Flipping MOTOR_FL_DIR and MOTOR_RR_DIR, OR
    - Flipping MOTOR_FR_DIR and MOTOR_RL_DIR
    """
    print("Strafe Left")
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)

def strafe_right(speed=SPEED_STRAFE):
    """
    Strafe right - typical mecanum configuration.
    If this moves forward/backward instead of right, try:
    - Flipping MOTOR_FL_DIR and MOTOR_RR_DIR, OR
    - Flipping MOTOR_FR_DIR and MOTOR_RL_DIR
    """
    print("Strafe Right")
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)

def rotate_left(speed=SPEED_ROTATE):
    """Rotate counter-clockwise (left)."""
    print("Rotate Left")
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)

def rotate_right(speed=SPEED_ROTATE):
    """Rotate clockwise (right)."""
    print("Rotate Right")
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
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

def cleanup_and_exit():
    """Safely shut down the robot and clean up resources."""
    print("Cleaning up GPIO...")
    stop()
    pwm_fl.stop()
    pwm_fr.stop()
    pwm_rl.stop()
    pwm_rr.stop()
    GPIO.cleanup()
    print("System halted.")

# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

print("\nRobot Control Active!")
print("Controls:")
print("  w = Forward")
print("  s = Backward")
print("  a = Strafe Left")
print("  d = Strafe Right")
print("  q = Rotate Left")
print("  e = Rotate Right")
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
            strafe_left()
        elif command == 'd':
            strafe_right()
        elif command == 'q':
            rotate_left()
        elif command == 'e':
            rotate_right()
        elif command == 'r':
            # 'r' is an alias for rotate_left for backward compatibility
            rotate_left()
        elif command == 'x':
            stop()
        elif command == '':
            continue
        else:
            print("Invalid command!")

except KeyboardInterrupt:
    print("\nManual stop triggered (Ctrl+C).")
    cleanup_and_exit()
except Exception as e:
    print(f"Error occurred: {e}")
    cleanup_and_exit()
finally:
    if 'GPIO' in dir():
        try:
            cleanup_and_exit()
        except:
            pass