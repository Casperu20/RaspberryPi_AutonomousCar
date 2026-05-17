#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# =========================================================
# GPIO SETUP
# =========================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# TRIGGER PINS (Now separate)
TRIG_FRONT = 16
TRIG_LEFT  = 4
TRIG_RIGHT = 15

# ECHO PINS
ECHO_FRONT = 26
ECHO_LEFT  = 14
ECHO_RIGHT = 8

# =========================================================
# PIN CONFIGURATION
# =========================================================

# Setup all triggers as outputs
trig_pins = [TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT]
for pin in trig_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Setup all echos as inputs
echo_pins = [ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT]
for pin in echo_pins:
    GPIO.setup(pin, GPIO.IN)

print("Initializing independent sensors...")
time.sleep(2)

# =========================================================
# UPDATED DISTANCE FUNCTION
# =========================================================

def read_distance(trig_pin, echo_pin):
    """
    Takes specific trig and echo pins to prevent crosstalk.
    """
    # Ensure trigger is low
    GPIO.output(trig_pin, False)
    time.sleep(0.01) # Very short settle time

    # Send 10us trigger pulse
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    pulse_start = time.time()
    timeout = pulse_start

    # Wait for echo HIGH
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if pulse_start - timeout > 0.03: # 30ms timeout
            return -1

    pulse_end = time.time()

    # Wait for echo LOW
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.03:
            return -1

    pulse_duration = pulse_end - pulse_start

    # Speed of sound (34300 cm/s / 2 for round trip)
    distance = pulse_duration * 17150

    return round(distance, 1)

# =========================================================
# MAIN LOOP
# =========================================================

try:
    while True:
        # Read each sensor one by one
        # The 0.05s delay between reads gives echoes time to vanish
        front = read_distance(TRIG_FRONT, ECHO_FRONT)
        time.sleep(0.05)

        left = read_distance(TRIG_LEFT, ECHO_LEFT)
        time.sleep(0.05)

        right = read_distance(TRIG_RIGHT, ECHO_RIGHT)
        time.sleep(0.05)

        print(
            f"Front: {str(front):>5} cm | "
            f"Left: {str(left):>5} cm | "
            f"Right: {str(right):>5} cm"
        )

        # Longer sleep between full cycles
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    GPIO.cleanup()