#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

STBY_PIN = 20

# Pin layout: (Input 1, Input 2, PWM)
RL_PINS = (5, 6, 19)  # Rear Left Motor Pins

# Ensure the direction matches your forward rotation
MOTOR_RL_DIR = 1

# Setup pins
GPIO.setup(STBY_PIN, GPIO.OUT, initial=GPIO.LOW)
for pin in RL_PINS:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# Start PWM on the Rear Left pin
pwm_rl = GPIO.PWM(RL_PINS[2], 1000)
pwm_rl.start(0)

# Wake up the driver board
GPIO.output(STBY_PIN, GPIO.HIGH)

def set_motor(pins, pwm, direction, speed, dir_multiplier):
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

try:
    print("Starting 30-second break-in for REAR LEFT motor only...")
    print("Running at 50% speed to smooth out internal gear friction.")
    
    # Spin Rear Left forward (direction=1) at 50% duty cycle
    set_motor(RL_PINS, pwm_rl, 1, 50, MOTOR_RL_DIR)
    
    # Let it run for 30 seconds
    sleep(15)
    
    print("Break-in period complete. Stopping motor.")

except KeyboardInterrupt:
    print("\nStopped early by user.")

finally:
    # Safe cleanup
    set_motor(RL_PINS, pwm_rl, 0, 0, MOTOR_RL_DIR)
    pwm_rl.stop()
    GPIO.output(STBY_PIN, GPIO.LOW)
    GPIO.cleanup()