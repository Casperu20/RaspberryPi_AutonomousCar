#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG_FRONT = 16
TRIG_LEFT = 4
TRIG_RIGHT = 15

ECHO_FRONT = 26
ECHO_LEFT = 14
ECHO_RIGHT = 8

INVALID_DISTANCE = -1

trig_pins = [TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT]
echo_pins = [ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT]

for pin in trig_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

for pin in echo_pins:
    GPIO.setup(pin, GPIO.IN)

sleep(2)


def read_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, False)
    time.sleep(0.002)

    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    timeout_start = time.monotonic()

    while GPIO.input(echo_pin) == 0:
        if time.monotonic() - timeout_start > 0.03:
            return INVALID_DISTANCE

    pulse_start = time.monotonic()

    while GPIO.input(echo_pin) == 1:
        if time.monotonic() - pulse_start > 0.03:
            return INVALID_DISTANCE

    pulse_end = time.monotonic()

    distance = (pulse_end - pulse_start) * 17150

    if distance < 2 or distance > 400:
        return INVALID_DISTANCE

    return round(distance, 1)


def read_distance_stable(trig_pin, echo_pin, samples=3):
    readings = []

    for _ in range(samples):
        distance = read_distance(trig_pin, echo_pin)

        if distance != INVALID_DISTANCE:
            readings.append(distance)

        sleep(0.04)

    if not readings:
        return INVALID_DISTANCE

    readings.sort()
    return readings[len(readings) // 2]


try:
    while True:
        front = read_distance_stable(TRIG_FRONT, ECHO_FRONT)
        sleep(0.05)

        left = read_distance_stable(TRIG_LEFT, ECHO_LEFT)
        sleep(0.05)

        right = read_distance_stable(TRIG_RIGHT, ECHO_RIGHT)
        sleep(0.05)

        print(
            f"Front: {front:>6} cm | "
            f"Left: {left:>6} cm | "
            f"Right: {right:>6} cm"
        )

        sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    GPIO.cleanup()