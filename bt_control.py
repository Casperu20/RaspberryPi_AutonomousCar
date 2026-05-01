#!/usr/bin/env python3
"""
Mecanum Wheel Robot Control via Bluetooth RFCOMM
Raspberry Pi 4B with 2x TB6612FNG Motor Drivers and 4x TT DC Motors
Supports persistent server with automatic disconnect/reconnect
"""

import RPi.GPIO as GPIO
from time import sleep
import socket
import struct
import sys

# ============================================================================
# GPIO CONFIGURATION
# ============================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Standby (enable) pins for the two motor drivers
STBY1_PIN = 20  # TB6612FNG Driver 1 STBY
STBY2_PIN = 21  # TB6612FNG Driver 2 STBY

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
MOTOR_FR_DIR = 1   # Front Right direction multiplier
MOTOR_RL_DIR = 1   # Rear Left direction multiplier
MOTOR_RR_DIR = 1   # Rear Right direction multiplier

# ============================================================================
# SPEED SETTINGS
# ============================================================================
SPEED_FORWARD_BACKWARD = 40  # Speed for forward/backward movement
SPEED_STRAFE = 70            # Speed for strafe left/right
SPEED_ROTATE = 60            # Speed for rotation

# ============================================================================
# GPIO SETUP
# ============================================================================

all_pins = [STBY1_PIN, STBY2_PIN] + list(FL_PINS) + list(FR_PINS) + list(RL_PINS) + list(RR_PINS)

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
GPIO.output(STBY1_PIN, GPIO.HIGH)
GPIO.output(STBY2_PIN, GPIO.HIGH)

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
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)

def backward(speed=SPEED_FORWARD_BACKWARD):
    """Move backward - all wheels backward."""
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)

def strafe_left(speed=SPEED_STRAFE):
    """
    Strafe left - typical mecanum configuration.
    If this moves forward/backward instead of left, try:
    - Flipping MOTOR_FL_DIR and MOTOR_RR_DIR, OR
    - Flipping MOTOR_FR_DIR and MOTOR_RL_DIR
    """
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)

def strafe_right(speed=SPEED_STRAFE):
    """
    Strafe right - typical mecanum configuration.
    If this moves forward/backward instead of right, try:
    - Flipping MOTOR_FL_DIR and MOTOR_RR_DIR, OR
    - Flipping MOTOR_FR_DIR and MOTOR_RL_DIR
    """
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)

def rotate_left(speed=SPEED_ROTATE):
    """Rotate counter-clockwise (left)."""
    set_motor(FL_PINS, pwm_fl, 1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, -1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, -1, speed, MOTOR_RR_DIR)

def rotate_right(speed=SPEED_ROTATE):
    """Rotate clockwise (right)."""
    set_motor(FL_PINS, pwm_fl, -1, speed, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 1, speed, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, -1, speed, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 1, speed, MOTOR_RR_DIR)

def stop():
    """Stop all motors."""
    set_motor(FL_PINS, pwm_fl, 0, 0, MOTOR_FL_DIR)
    set_motor(FR_PINS, pwm_fr, 0, 0, MOTOR_FR_DIR)
    set_motor(RL_PINS, pwm_rl, 0, 0, MOTOR_RL_DIR)
    set_motor(RR_PINS, pwm_rr, 0, 0, MOTOR_RR_DIR)

# ============================================================================
# BLUETOOTH SERVER SETUP
# ============================================================================

server_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
client_sock = None
RFCOMM_CHANNEL = 3

try:
    # Allow reusing the address
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Ensure Serial Port Profile is advertised on the same channel used by this server.
    # For Android Bluetooth Serial Terminal, run:
    # sudo sdptool add SP
    # sudo sdptool add --channel=3 SP
    print("If needed, run: sudo sdptool add SP && sudo sdptool add --channel=3 SP")

    print(f"Binding RFCOMM server on channel {RFCOMM_CHANNEL}...")
    try:
        # Preferred form requested: bind on all local adapters.
        server_sock.bind(("", RFCOMM_CHANNEL))
        print("Bind successful with address ''")
    except OSError as bind_error:
        # Some BlueZ/Python builds reject empty address for AF_BLUETOOTH.
        # Fallback keeps compatibility on Raspberry Pi images where this happens.
        print(f"Primary bind failed ({bind_error}); retrying with wildcard adapter address...")
        server_sock.bind(("00:00:00:00:00:00", RFCOMM_CHANNEL))
        print("Bind successful with address '00:00:00:00:00:00'")

    server_sock.listen(1)
    print(f"Listening started on RFCOMM channel {RFCOMM_CHANNEL}")

    # Timeout keeps accept() from blocking forever silently.
    server_sock.settimeout(10)
    print("Server ready")

except Exception as e:
    print(f"Error starting Bluetooth server: {e}")
    stop()
    GPIO.cleanup()
    sys.exit(1)

# ============================================================================
# MAIN CONTROL LOOP - PERSISTENT SERVER
# ============================================================================

try:
    # Outer loop: server keeps running, accepts multiple connections
    while True:
        try:
            print("Waiting for connection...")
            print("Calling accept()")
            client_sock, address = server_sock.accept()
            print("Client connected")
            print(f"Client address: {address}")
            client_sock.settimeout(2.0)
            print("Commands: w=forward, s=backward, a=strafe_left, d=strafe_right, q=rotate_left, e=rotate_right, x=stop")
            
            # Inner loop: handle current client connection
            while True:
                try:
                    # Receive data from Bluetooth client
                    raw_data = client_sock.recv(1024)
                    if not raw_data:
                        # Client disconnected
                        print(f"Client {address} disconnected.")
                        stop()
                        break
                    
                    # Process incoming data character by character
                    try:
                        data_str = raw_data.decode('utf-8').lower()
                    except UnicodeDecodeError as decode_error:
                        print(f"Decode error: {decode_error}")
                        continue
                    
                    for char in data_str:
                        if not char.isalpha():
                            continue
                        
                        # Process individual command characters
                        if char == 'w':
                            forward()
                        elif char == 's':
                            backward()
                        elif char == 'a':
                            strafe_left()
                        elif char == 'd':
                            strafe_right()
                        elif char == 'q':
                            rotate_left()
                        elif char == 'e':
                            rotate_right()
                        elif char == 'x':
                            stop()

                except socket.timeout:
                    # No data from current client in timeout window; keep waiting.
                    continue
                except (ConnectionResetError, BrokenPipeError, OSError) as e:
                    print(f"Client connection error: {e}")
                    stop()
                    break
                except Exception as e:
                    print(f"Error processing command: {e}")
                    stop()
                    break

        except socket.timeout:
            print("accept() timeout; still waiting for client...")
            continue
        
        except KeyboardInterrupt:
            # Ctrl+C pressed - exit outer loop to shutdown
            print("\nManual stop triggered (Ctrl+C). Shutting down...")
            raise
        
        finally:
            # Close client socket but keep server running
            if client_sock:
                try:
                    client_sock.close()
                except:
                    pass
                client_sock = None

except KeyboardInterrupt:
    print("\nShutting down server...")

finally:
    print("Cleanup in progress...")
    stop()
    if client_sock:
        try:
            client_sock.close()
        except:
            pass
    try:
        server_sock.close()
    except:
        pass
    pwm_fl.stop()
    pwm_fr.stop()
    pwm_rl.stop()
    pwm_rr.stop()
    GPIO.cleanup()
    print("System halted.")
