#!/usr/bin/env python3
"""
Autonomous Grid Mapper
----------------------

Converts the ultrasonic + MPU6050 obstacle-avoidance robot into a simple
grid-mapping robot:

- Tracks (x, y, heading) on a discrete grid.
- Moves exactly one cell at a time using timed forward motion.
- Uses MPU6050 gyro-assisted 90-degree turns.
- Uses ultrasonic sensors to mark neighboring cells as free / obstacle.
- Saves the map to JSON + ASCII text files.
- Prints an ASCII map after each step.
- Provides a placeholder per-cell environment sampler
  (future DHT / air-quality hookup).

Hardware:
- Raspberry Pi 4B
- 2x TB6612FNG motor drivers
- 4x TT motors with mecanum wheels
- 3x HC-SR04 ultrasonic sensors
- MPU6050 over I2C (smbus, address 0x68, bus 1)

No camera.
Uses 2x DHT11 for temperature/humidity mapping.
No true air-quality sensor yet.

robot_grid_map:
? = unknown
. = free
* = visited
# = obstacle
^ > v < = robot heading
"""

import json
import time
from datetime import datetime
from time import sleep

import board
import adafruit_dht
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
# ULTRASONIC PINS
# ============================================================================

TRIG_FRONT = 16
TRIG_LEFT = 4
TRIG_RIGHT = 15

ECHO_FRONT = 26
ECHO_LEFT = 14
ECHO_RIGHT = 8

# ============================================================================
# MPU6050 REGISTERS
# ============================================================================

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
# ACCEL_ZOUT_H (0x3F) intentionally not used (defective)

GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# ============================================================================
# SPEED / DRIVE SETTINGS
# ============================================================================

SPEED_FORWARD = 70
SPEED_BACKWARD = 70
SPEED_ROTATE = 70

FRONT_SAFE_DISTANCE = 30
FRONT_DANGER_DISTANCE = 18
SIDE_CLEAR_DISTANCE = 25

BACKUP_TIME = 0.35
LOOP_DELAY = 0.10

TURN_ANGLE = 30.0
TURN_90_ANGLE = 90.0
GYRO_DEADBAND = 0.8
MAX_TURN_TIME = 4.0

INVALID_DISTANCE = -1

# ============================================================================
# GRID MAPPING SETTINGS
# ============================================================================

MAP_WIDTH = 61
MAP_HEIGHT = 61
CELL_SIZE_CM = 35

CELL_DRIVE_TIME = 0.75
CELL_DRIVE_SPEED = 40

CELL_FREE_DISTANCE = 35
CELL_BLOCKED_DISTANCE = 25

MAX_GRID_STEPS = 20 # scan -> update map -> choose cell -> turn -> move one cell -> save map

MAP_JSON_FILE = "robot_grid_map.json"
MAP_ASCII_FILE = "robot_grid_map.txt"

# ============================================================================
# DHT11 SENSOR SETTINGS
# ============================================================================

DHT1_PIN = board.D21   # BCM GPIO21, physical pin 40
DHT2_PIN = board.D10   # BCM GPIO10, physical pin 19

DHT_SAMPLE_COUNT = 1
DHT_SAMPLE_DELAY = 2.2

# Cell states
UNKNOWN = "unknown"
FREE = "free"
OBSTACLE = "obstacle"

# Headings
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

HEADING_DELTAS = {
    NORTH: (0, -1),
    EAST: (1, 0),
    SOUTH: (0, 1),
    WEST: (-1, 0),
}

HEADING_NAMES = {
    NORTH: "NORTH",
    EAST: "EAST",
    SOUTH: "SOUTH",
    WEST: "WEST",
}

HEADING_ARROW = {
    NORTH: "^",
    EAST: ">",
    SOUTH: "v",
    WEST: "<",
}

# Relative directions
REL_FRONT = 0
REL_RIGHT = 1
REL_BACK = 2
REL_LEFT = 3

# ============================================================================
# GLOBAL STATE
# ============================================================================

accel_offsets = {"x": 0.0, "y": 0.0}
gyro_offsets = {"x": 0.0, "y": 0.0, "z": 0.0}

yaw_angle = 0.0
yaw_last_time = None

cleaned_up = False

# Grid state (initialized in main)
grid = []
robot_x = MAP_WIDTH // 2
robot_y = MAP_HEIGHT // 2
robot_heading = NORTH
position_stack = []
current_step = 0

# ============================================================================
# GPIO SETUP
# ============================================================================

print("Initializing GPIO pins...")

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

trig_pins = [TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT]
echo_pins = [ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT]

for pin in trig_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

for pin in echo_pins:
    GPIO.setup(pin, GPIO.IN)

print("GPIO initialized.")
print("Waiting for ultrasonic sensors to settle...")
time.sleep(2)

# ============================================================================
# MPU SETUP
# ============================================================================

bus = smbus.SMBus(1)
dht1 = adafruit_dht.DHT11(DHT1_PIN, use_pulseio=False)
dht2 = adafruit_dht.DHT11(DHT2_PIN, use_pulseio=False)


def init_mpu():
    """Hard reset and configuration sequence from tested MPU6050 code."""
    try:
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x80)
        time.sleep(0.2)

        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # +/- 2g accel and +/- 250 deg/s gyro
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)

        print("MPU6050: System Reset & Initialized.")
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        raise


def read_word(reg):
    """Read signed 16-bit value from MPU register."""
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low

    if value >= 0x8000:
        value = -((65535 - value) + 1)

    return value


def calibrate_sensors(samples=300):
    """Calculate accel X/Y and gyro X/Y/Z offsets while stationary."""
    global accel_offsets, gyro_offsets

    print(f"Calibrating MPU6050 over {samples} samples. Keep robot still...")

    sum_ax, sum_ay = 0.0, 0.0
    sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0

    for _ in range(samples):
        sum_ax += read_word(ACCEL_XOUT_H) / 16384.0
        sum_ay += read_word(ACCEL_YOUT_H) / 16384.0

        sum_gx += read_word(GYRO_XOUT_H) / 131.0
        sum_gy += read_word(GYRO_YOUT_H) / 131.0
        sum_gz += read_word(GYRO_ZOUT_H) / 131.0

        time.sleep(0.005)

    accel_offsets["x"] = sum_ax / samples
    accel_offsets["y"] = sum_ay / samples

    gyro_offsets["x"] = sum_gx / samples
    gyro_offsets["y"] = sum_gy / samples
    gyro_offsets["z"] = sum_gz / samples

    print(
        "Calibration finished. "
        f"Gyro offsets => X:{gyro_offsets['x']:.3f}, "
        f"Y:{gyro_offsets['y']:.3f}, Z:{gyro_offsets['z']:.3f}"
    )


def get_clean_imu_data():
    """Return accel X/Y and gyro X/Y/Z with offsets applied."""
    ax = (read_word(ACCEL_XOUT_H) / 16384.0) - accel_offsets["x"]
    ay = (read_word(ACCEL_YOUT_H) / 16384.0) - accel_offsets["y"]

    gx = (read_word(GYRO_XOUT_H) / 131.0) - gyro_offsets["x"]
    gy = (read_word(GYRO_YOUT_H) / 131.0) - gyro_offsets["y"]
    gz = (read_word(GYRO_ZOUT_H) / 131.0) - gyro_offsets["z"]

    return ax, ay, gx, gy, gz


def reset_yaw():
    """Reset yaw integrator state before a controlled turn."""
    global yaw_angle, yaw_last_time
    yaw_angle = 0.0
    yaw_last_time = time.monotonic()


def update_yaw():
    """Integrate gyro Z over time and return current yaw angle (degrees)."""
    global yaw_angle, yaw_last_time

    now = time.monotonic()

    if yaw_last_time is None:
        yaw_last_time = now
        return yaw_angle

    dt = now - yaw_last_time
    yaw_last_time = now

    gz = (read_word(GYRO_ZOUT_H) / 131.0) - gyro_offsets["z"]

    if abs(gz) < GYRO_DEADBAND:
        gz = 0.0

    # Prevent one bad delay from creating a huge fake yaw jump.
    if dt < 0 or dt > 0.2:
        dt = 0.0

    yaw_angle += gz * dt
    return yaw_angle


# ============================================================================
# MOTOR CONTROL
# ============================================================================

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


def rotate_left_angle(target_angle=TURN_ANGLE, speed=SPEED_ROTATE):
    """Rotate left until abs(yaw) reaches target angle or timeout.
    Returns True on success, False on timeout."""
    print(f"TURN: Left target={target_angle:.1f} deg")

    reset_yaw()
    sleep(0.05)
    rotate_left(speed)

    start_time = time.monotonic()
    last_print_time = start_time
    timed_out = False

    while True:
        current_yaw = update_yaw()
        now = time.monotonic()

        if now - last_print_time >= 0.10:
            print(f"Yaw | Left turn: {abs(current_yaw):6.2f} deg")
            last_print_time = now

        if abs(current_yaw) >= target_angle:
            break

        if now - start_time >= MAX_TURN_TIME:
            timed_out = True
            break

        time.sleep(0.01)

    stop()

    if timed_out:
        print(
            f"TURN TIMEOUT: Left turn exceeded {MAX_TURN_TIME:.2f}s, "
            f"yaw={abs(yaw_angle):.2f} deg"
        )
        return False

    print(f"TURN DONE: Left reached {abs(yaw_angle):.2f} deg")
    return True


def rotate_right_angle(target_angle=TURN_ANGLE, speed=SPEED_ROTATE):
    """Rotate right until abs(yaw) reaches target angle or timeout.
    Returns True on success, False on timeout."""
    print(f"TURN: Right target={target_angle:.1f} deg")

    reset_yaw()
    sleep(0.05)
    rotate_right(speed)

    start_time = time.monotonic()
    last_print_time = start_time
    timed_out = False

    while True:
        current_yaw = update_yaw()
        now = time.monotonic()

        if now - last_print_time >= 0.10:
            print(f"Yaw | Right turn: {abs(current_yaw):6.2f} deg")
            last_print_time = now

        if abs(current_yaw) >= target_angle:
            break

        if now - start_time >= MAX_TURN_TIME:
            timed_out = True
            break

        time.sleep(0.01)

    stop()

    if timed_out:
        print(
            f"TURN TIMEOUT: Right turn exceeded {MAX_TURN_TIME:.2f}s, "
            f"yaw={abs(yaw_angle):.2f} deg"
        )
        return False

    print(f"TURN DONE: Right reached {abs(yaw_angle):.2f} deg")
    return True


# ============================================================================
# ULTRASONIC FUNCTIONS
# ============================================================================

def read_distance(trig_pin, echo_pin):
    """Read distance from one HC-SR04 sensor (cm). Returns -1 on failure."""
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
    """Median of multiple HC-SR04 readings to reduce spikes."""
    readings = []

    for _ in range(samples):
        distance = read_distance(trig_pin, echo_pin)

        if distance != INVALID_DISTANCE:
            readings.append(distance)

        time.sleep(0.04)

    if not readings:
        return INVALID_DISTANCE

    readings.sort()
    return readings[len(readings) // 2]


def read_all_distances():
    front = read_distance_stable(TRIG_FRONT, ECHO_FRONT)
    time.sleep(0.05)

    left = read_distance_stable(TRIG_LEFT, ECHO_LEFT)
    time.sleep(0.05)

    right = read_distance_stable(TRIG_RIGHT, ECHO_RIGHT)
    time.sleep(0.05)

    return front, left, right


# ============================================================================
# GRID MAP HELPERS
# ============================================================================

def create_cell():
    return {
        "state": UNKNOWN,
        "visited": False,
        "samples": [],
        "temperature": None,
        "humidity": None,
        "air_quality": None,
    }


def init_grid():
    global grid
    grid = [[create_cell() for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]


def in_bounds(x, y):
    return 0 <= x < MAP_WIDTH and 0 <= y < MAP_HEIGHT


def get_cell(x, y):
    if not in_bounds(x, y):
        return None
    return grid[y][x]


def mark_cell_state(x, y, state):
    cell = get_cell(x, y)
    if cell is None:
        return

    # Do not downgrade an obstacle back to free based on a single later reading.
    if cell["state"] == OBSTACLE and state == FREE:
        return

    cell["state"] = state


def mark_current_cell_visited():
    cell = get_cell(robot_x, robot_y)
    if cell is None:
        return
    cell["visited"] = True
    if cell["state"] != OBSTACLE:
        cell["state"] = FREE


def heading_after_relative(relative_direction):
    """Return absolute heading if robot looks toward `relative_direction`."""
    return (robot_heading + relative_direction) % 4


def neighbor_from_heading(x, y, heading):
    dx, dy = HEADING_DELTAS[heading]
    return x + dx, y + dy


def neighbor_from_relative(relative_direction):
    abs_heading = heading_after_relative(relative_direction)
    return neighbor_from_heading(robot_x, robot_y, abs_heading)


def update_neighbor_from_sensor(relative_direction, distance):
    nx, ny = neighbor_from_relative(relative_direction)
    if not in_bounds(nx, ny):
        return

    if distance == INVALID_DISTANCE:
        # Leave as unknown.
        return

    if distance <= CELL_BLOCKED_DISTANCE:
        mark_cell_state(nx, ny, OBSTACLE)
    elif distance >= CELL_FREE_DISTANCE:
        mark_cell_state(nx, ny, FREE)
    # In-between distances: ambiguous, leave as-is.


def update_map_from_ultrasonic(front, left, right):
    print(
        f"Sensors | Front: {front} cm | Left: {left} cm | Right: {right} cm"
    )
    update_neighbor_from_sensor(REL_FRONT, front)
    update_neighbor_from_sensor(REL_LEFT, left)
    update_neighbor_from_sensor(REL_RIGHT, right)


def cell_is_free_and_unvisited(x, y):
    cell = get_cell(x, y)
    if cell is None:
        return False
    if cell["state"] == OBSTACLE:
        return False
    if cell["visited"]:
        return False
    return cell["state"] == FREE


def choose_next_grid_target():
    """
    DFS-style movement.

    Returns:
        (target_x, target_y, mode)

    mode:
        "explore"   -> moving into a new unvisited cell
        "backtrack" -> returning to a previous visited cell
    """

    # First try to explore new free cells.
    for rel in (REL_FRONT, REL_LEFT, REL_RIGHT):
        nx, ny = neighbor_from_relative(rel)

        if cell_is_free_and_unvisited(nx, ny):
            print(
                f"Next target: ({nx}, {ny}) via relative "
                f"{['FRONT','RIGHT','BACK','LEFT'][rel]}"
            )
            return nx, ny, "explore"

    # If no new neighbor exists, backtrack only to an adjacent previous cell.
    while position_stack:
        px, py = position_stack.pop()

        if (px, py) == (robot_x, robot_y):
            continue

        distance = abs(px - robot_x) + abs(py - robot_y)

        if distance == 1:
            print(f"Backtracking to ({px}, {py})")
            return px, py, "backtrack"

        print(f"Skipping non-adjacent backtrack target ({px}, {py})")

    print("No more reachable cells.")
    return None

def heading_toward_cell(target_x, target_y):
    dx = target_x - robot_x
    dy = target_y - robot_y

    if dx == 0 and dy == -1:
        return NORTH

    if dx == 1 and dy == 0:
        return EAST

    if dx == 0 and dy == 1:
        return SOUTH

    if dx == -1 and dy == 0:
        return WEST

    raise ValueError(
        f"Target cell ({target_x}, {target_y}) is not adjacent to "
        f"robot position ({robot_x}, {robot_y})"
    )


def turn_to_heading(target_heading):
    """Rotate so robot_heading == target_heading using 90-degree gyro turns."""
    global robot_heading

    diff = (target_heading - robot_heading) % 4

    if diff == 0:
        print("Turn: already facing target heading.")
        return True

    if diff == 1:
        ok = rotate_right_angle(TURN_90_ANGLE, SPEED_ROTATE)
        if ok:
            robot_heading = (robot_heading + 1) % 4
        return ok

    if diff == 3:
        ok = rotate_left_angle(TURN_90_ANGLE, SPEED_ROTATE)
        if ok:
            robot_heading = (robot_heading - 1) % 4
        return ok

    # diff == 2: two right turns
    ok1 = rotate_right_angle(TURN_90_ANGLE, SPEED_ROTATE)
    if ok1:
        robot_heading = (robot_heading + 1) % 4
    sleep(0.1)
    ok2 = rotate_right_angle(TURN_90_ANGLE, SPEED_ROTATE)
    if ok2:
        robot_heading = (robot_heading + 1) % 4

    return ok1 and ok2


def update_robot_position_forward():
    global robot_x, robot_y
    dx, dy = HEADING_DELTAS[robot_heading]
    robot_x += dx
    robot_y += dy


def mark_front_as_obstacle():
    nx, ny = neighbor_from_relative(REL_FRONT)
    if in_bounds(nx, ny):
        mark_cell_state(nx, ny, OBSTACLE)
        print(f"Marked front cell ({nx}, {ny}) as OBSTACLE.")


def move_forward_one_cell():
    """Drive forward for CELL_DRIVE_TIME while watching front sensor.
    Returns True on success, False if blocked mid-move."""
    print(
        f"Moving one cell forward from ({robot_x}, {robot_y}) "
        f"heading {HEADING_NAMES[robot_heading]}"
    )

    danger_streak = 0
    forward(CELL_DRIVE_SPEED)
    start = time.monotonic()

    try:
        while time.monotonic() - start < CELL_DRIVE_TIME:
            front = read_distance(TRIG_FRONT, ECHO_FRONT)

            if front != INVALID_DISTANCE and front <= FRONT_DANGER_DISTANCE:
                danger_streak += 1
                print(f"Front danger reading: {front} cm (streak={danger_streak})")
                if danger_streak >= 2:
                    stop()
                    print("Blocked mid-cell. Aborting move.")
                    return False
            else:
                danger_streak = 0

            sleep(0.03)
    finally:
        stop()

    sleep(0.1)
    return True


# ============================================================================
# ENVIRONMENT SAMPLER (DHT11)
# ============================================================================

def read_one_dht(sensor, name):
    """
    Read one DHT11 sensor.
    Returns (temperature, humidity) or (None, None).
    """
    try:
        temperature = sensor.temperature
        humidity = sensor.humidity

        if temperature is None or humidity is None:
            print(f"{name}: invalid DHT reading")
            return None, None

        return float(temperature), float(humidity)

    except RuntimeError as e:
        # DHT sensors commonly fail occasional reads.
        print(f"{name}: DHT retry needed: {e}")
        return None, None

    except Exception as e:
        print(f"{name}: serious DHT error: {e}")
        return None, None


def average_valid_values(values):
    valid = [value for value in values if value is not None]

    if not valid:
        return None

    return sum(valid) / len(valid)


def sample_environment():
    """
    Read both DHT11 sensors and save data into the current grid cell.

    DHT11 is not an air-quality sensor.
    For now:
    - temperature = average of valid temperature readings
    - humidity = average of valid humidity readings
    - air_quality = None
    """

    cell = get_cell(robot_x, robot_y)

    if cell is None:
        return

    print("Sampling environment at current cell...")
    stop()
    sleep(0.5)

    temp_values = []
    humidity_values = []

    sensor_details = {
        "dht1": [],
        "dht2": [],
    }

    for sample_index in range(DHT_SAMPLE_COUNT):
        temp1, hum1 = read_one_dht(dht1, "DHT11 #1 GPIO21")
        temp2, hum2 = read_one_dht(dht2, "DHT11 #2 GPIO10")

        sensor_details["dht1"].append({
            "temperature": temp1,
            "humidity": hum1,
        })

        sensor_details["dht2"].append({
            "temperature": temp2,
            "humidity": hum2,
        })

        if temp1 is not None:
            temp_values.append(temp1)

        if hum1 is not None:
            humidity_values.append(hum1)

        if temp2 is not None:
            temp_values.append(temp2)

        if hum2 is not None:
            humidity_values.append(hum2)

        if sample_index < DHT_SAMPLE_COUNT - 1:
            sleep(DHT_SAMPLE_DELAY)

    avg_temperature = average_valid_values(temp_values)
    avg_humidity = average_valid_values(humidity_values)

    sample = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "temperature": avg_temperature,
        "humidity": avg_humidity,
        "air_quality": None,
        "sensor_details": sensor_details,
    }

    cell["samples"].append(sample)

    # Mirror latest values at top level for easy visualization.
    cell["temperature"] = avg_temperature
    cell["humidity"] = avg_humidity
    cell["air_quality"] = None

    print(
        "Environment sample | "
        f"temperature={avg_temperature} C | "
        f"humidity={avg_humidity}% | "
        f"air_quality=None"
    )


# ============================================================================
# MAP I/O AND VISUALIZATION
# ============================================================================

def build_ascii_map():
    lines = []
    for y in range(MAP_HEIGHT):
        row_chars = []
        for x in range(MAP_WIDTH):
            if x == robot_x and y == robot_y:
                row_chars.append(HEADING_ARROW[robot_heading])
                continue

            cell = grid[y][x]
            state = cell["state"]

            if state == OBSTACLE:
                row_chars.append("#")
            elif cell["visited"]:
                row_chars.append("*")
            elif state == FREE:
                row_chars.append(".")
            else:
                row_chars.append("?")

        lines.append("".join(row_chars))
    return "\n".join(lines)


def print_ascii_map():
    print("\n--- MAP ---")
    print(build_ascii_map())
    print(
        f"Robot @ ({robot_x}, {robot_y}) heading {HEADING_NAMES[robot_heading]}"
    )
    print("-----------\n")


def save_map_files():
    try:
        with open(MAP_ASCII_FILE, "w") as f:
            f.write(build_ascii_map())
            f.write(
                f"\n\nRobot @ ({robot_x}, {robot_y}) "
                f"heading {HEADING_NAMES[robot_heading]}\n"
            )

        payload = {
            "width": MAP_WIDTH,
            "height": MAP_HEIGHT,
            "cell_size_cm": CELL_SIZE_CM,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "step": current_step,
            "robot": {
                "x": robot_x,
                "y": robot_y,
                "heading": HEADING_NAMES[robot_heading],
            },
            "grid": grid,
        }
        with open(MAP_JSON_FILE, "w") as f:
            json.dump(payload, f, indent=2)
    except Exception as e:
        print(f"Map save warning: {e}")


# ============================================================================
# GRID MAPPER MAIN LOOP
# ============================================================================

def run_grid_mapper():
    global position_stack, current_step

    print("\nGrid mapper started.")
    print("Press Ctrl+C to stop.\n")

    init_grid()
    position_stack = []

    mark_current_cell_visited()
    sample_environment()
    save_map_files()
    print_ascii_map()

    steps = 0
    while steps < MAX_GRID_STEPS:
        current_step = steps + 1
        print(f"\n=== Step {steps + 1} / {MAX_GRID_STEPS} ===")

        # 1. Sense surroundings and update map.
        front, left, right = read_all_distances()
        update_map_from_ultrasonic(front, left, right)

        # 2. Choose target.
        target = choose_next_grid_target()
        if target is None:
            print("Mapping complete: nowhere left to go.")
            break

        target_x, target_y, move_mode = target

        # 3. Turn toward target.
        target_heading = heading_toward_cell(target_x, target_y)
        if not turn_to_heading(target_heading):
            print("Turn failed. Retrying once after a short pause...")
            sleep(0.5)
            if not turn_to_heading(target_heading):
                print("Turn failed twice. Stopping mapper to avoid corrupting the map.")
                stop()
                break

        # 4. Re-check front before committing to the move.
        front_recheck = read_distance_stable(TRIG_FRONT, ECHO_FRONT)
        if (
            front_recheck != INVALID_DISTANCE
            and front_recheck <= CELL_BLOCKED_DISTANCE
        ):
            print(
                f"Front recheck shows {front_recheck} cm -> mark obstacle, skip."
            )
            mark_front_as_obstacle()
            save_map_files()
            print_ascii_map()
            steps += 1
            continue

        # 5. Push current position only when exploring a new cell.
        if move_mode == "explore":
            position_stack.append((robot_x, robot_y))

        # 6. Try to move one cell.
        ok = move_forward_one_cell()

        if ok:
            update_robot_position_forward()
            if not in_bounds(robot_x, robot_y):
                print("Robot position went outside the virtual map. Stopping.")
                stop()
                break
            mark_current_cell_visited()
            sample_environment()
        else:
            mark_front_as_obstacle()

        save_map_files()
        print_ascii_map()

        steps += 1
        sleep(LOOP_DELAY)

    print("\nGrid mapping session ended.")
    save_map_files()
    print_ascii_map()


# ============================================================================
# CLEANUP
# ============================================================================

def cleanup_and_exit():
    global cleaned_up

    if cleaned_up:
        return

    print("\nCleaning up GPIO...")

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
        print(f"GPIO cleanup warning: {e}")

    try:
        dht1.exit()
        dht2.exit()
    except Exception as e:
        print(f"DHT cleanup warning: {e}")

    cleaned_up = True
    print("System halted.")


# ============================================================================
# MAIN
# ============================================================================

try:
    init_mpu()
    calibrate_sensors(300)

    ax, ay, gx, gy, gz = get_clean_imu_data()
    print(
        "Initial IMU | "
        f"AX:{ax:.2f}g AY:{ay:.2f}g "
        f"GX:{gx:.2f} GY:{gy:.2f} GZ:{gz:.2f} deg/s"
    )

    run_grid_mapper()

except KeyboardInterrupt:
    print("\nManual stop triggered.")

except Exception as e:
    print(f"Error occurred: {e}")

finally:
    cleanup_and_exit()
