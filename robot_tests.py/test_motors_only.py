#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

STBY_PIN = 20

FL_PINS = (27, 17, 18)
FR_PINS = (22, 23, 13)
RL_PINS = (25, 24, 12)
RR_PINS = (6, 5, 19)

MOTOR_FL_DIR = 1
MOTOR_FR_DIR = -1
MOTOR_RL_DIR = 1
MOTOR_RR_DIR = -1

SPEED_FORWARD = 50
SPEED_BACKWARD = 50
SPEED_ROTATE = 70

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


def clamp_speed(speed):
    if speed < 0:
        return 0
    if speed > 100:
        return 100
    return speed


def set_motor(pins, pwm, direction, speed, dir_multiplier):
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


def forward(speed=SPEED_FORWARD):
    print("ACTION: Forward")
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)


def backward(speed=SPEED_BACKWARD):
    print("ACTION: Backward")
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)


def rotate_right(speed=SPEED_ROTATE):
    print("ACTION: Rotate Right")
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)


def rotate_left(speed=SPEED_ROTATE):
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


def cleanup():
    stop()
    sleep(0.1)

    pwm_fl.stop()
    pwm_fr.stop()
    pwm_rl.stop()
    pwm_rr.stop()

    GPIO.output(STBY_PIN, GPIO.LOW)
    GPIO.cleanup()


try:
    print("Motor test. Keep robot lifted.")
    input("Press Enter for FORWARD...")
    forward()
    sleep(1)
    stop()

    input("Press Enter for BACKWARD...")
    backward()
    sleep(1)
    stop()

    input("Press Enter for ROTATE LEFT...")
    rotate_left()
    sleep(1)
    stop()

    input("Press Enter for ROTATE RIGHT...")
    rotate_right()
    sleep(1)
    stop()

    print("Motor test finished.")

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    cleanup()