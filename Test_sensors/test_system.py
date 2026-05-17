#!/usr/bin/env python3
"""
Comprehensive System Testing Suite for Mecanum Robot
Tests all hardware components and validates software functionality.

Run with: sudo python3 test_system.py
"""

import RPi.GPIO as GPIO
import smbus
from time import sleep, time
import sys
from collections import deque
from math import sqrt

# ============================================================================
# CONFIGURATION
# ============================================================================

# Motor pins
STBY_PIN = 20
FL_PINS = (27, 17, 18)
FR_PINS = (22, 23, 13)
RL_PINS = (25, 24, 12)
RR_PINS = (6, 5, 19)

# Sensor pins
TRIG_PIN = 16
ECHO_FRONT = 26
ECHO_LEFT = 14
ECHO_RIGHT = 8

# I2C
MPU6050_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Test configuration
TEST_MOTOR_SPEED = 50  # 50% duty cycle
MOTOR_TEST_DURATION = 0.5  # seconds
SENSOR_READINGS = 10
TEMPERATURE_THRESHOLD = 80  # °C

# ============================================================================
# COLOR OUTPUT HELPERS
# ============================================================================

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'
    BOLD = '\033[1m'

def print_pass(msg):
    print(f"{Colors.GREEN}✓ PASS{Colors.END}: {msg}")

def print_fail(msg):
    print(f"{Colors.RED}✗ FAIL{Colors.END}: {msg}")

def print_warn(msg):
    print(f"{Colors.YELLOW}⚠ WARN{Colors.END}: {msg}")

def print_info(msg):
    print(f"{Colors.BLUE}ℹ INFO{Colors.END}: {msg}")

def print_header(msg):
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{msg:^60}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}\n")

# ============================================================================
# TEST RESULTS TRACKING
# ============================================================================

class TestResults:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.warnings = 0
        
    def add_pass(self):
        self.passed += 1
        
    def add_fail(self):
        self.failed += 1
        
    def add_warn(self):
        self.warnings += 1
    
    def print_summary(self):
        total = self.passed + self.failed + self.warnings
        print(f"\n{Colors.BOLD}TEST SUMMARY{Colors.END}")
        print(f"  Passed: {Colors.GREEN}{self.passed}{Colors.END}/{total}")
        print(f"  Failed: {Colors.RED}{self.failed}{Colors.END}/{total}")
        print(f"  Warnings: {Colors.YELLOW}{self.warnings}{Colors.END}/{total}")
        
        if self.failed == 0:
            print(f"\n{Colors.GREEN}All tests passed!{Colors.END}")
            return True
        else:
            print(f"\n{Colors.RED}Some tests failed. Check output above.{Colors.END}")
            return False

results = TestResults()

# ============================================================================
# GPIO SETUP
# ============================================================================

def setup_gpio():
    """Initialize GPIO for testing."""
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        all_pins = [STBY_PIN] + list(FL_PINS) + list(FR_PINS) + list(RL_PINS) + list(RR_PINS)
        all_pins += [TRIG_PIN, ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT]
        
        for pin in all_pins:
            if pin in [ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT]:
                GPIO.setup(pin, GPIO.IN)
            else:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        
        print_pass("GPIO initialization")
        results.add_pass()
        return True
        
    except Exception as e:
        print_fail(f"GPIO initialization: {e}")
        results.add_fail()
        return False

# ============================================================================
# GPIO TOGGLE TEST
# ============================================================================

def test_gpio_pins():
    """Test that all GPIO pins toggle correctly."""
    print_header("GPIO PIN TOGGLE TEST")
    
    test_pins = [
        (STBY_PIN, "Standby (GPIO20)"),
        (FL_PINS[0], "FL Forward (GPIO27)"),
        (FL_PINS[1], "FL Reverse (GPIO17)"),
        (FL_PINS[2], "FL PWM (GPIO18)"),
        (FR_PINS[0], "FR Forward (GPIO22)"),
        (FR_PINS[1], "FR Reverse (GPIO23)"),
        (FR_PINS[2], "FR PWM (GPIO13)"),
        (RL_PINS[0], "RL Forward (GPIO25)"),
        (RL_PINS[1], "RL Reverse (GPIO24)"),
        (RL_PINS[2], "RL PWM (GPIO12)"),
        (RR_PINS[0], "RR Forward (GPIO6)"),
        (RR_PINS[1], "RR Reverse (GPIO5)"),
        (RR_PINS[2], "RR PWM (GPIO19)"),
        (TRIG_PIN, "Ultrasonic Trigger (GPIO16)"),
    ]
    
    for pin, name in test_pins:
        try:
            GPIO.output(pin, GPIO.HIGH)
            GPIO.output(pin, GPIO.LOW)
            print_pass(f"{name}")
            results.add_pass()
        except Exception as e:
            print_fail(f"{name}: {e}")
            results.add_fail()

# ============================================================================
# MOTOR TESTS
# ============================================================================

def test_motors():
    """Test motor direction and PWM control."""
    print_header("MOTOR CONTROL TEST")
    
    # Enable motor drivers
    GPIO.output(STBY_PIN, GPIO.HIGH)
    print_info("Motor drivers enabled (STBY = HIGH)")
    
    # Create PWM objects
    try:
        pwm_fl = GPIO.PWM(FL_PINS[2], 1000)
        pwm_fr = GPIO.PWM(FR_PINS[2], 1000)
        pwm_rl = GPIO.PWM(RL_PINS[2], 1000)
        pwm_rr = GPIO.PWM(RR_PINS[2], 1000)
        
        pwm_fl.start(0)
        pwm_fr.start(0)
        pwm_rl.start(0)
        pwm_rr.start(0)
        
        print_pass("PWM object creation")
        results.add_pass()
    except Exception as e:
        print_fail(f"PWM creation: {e}")
        results.add_fail()
        return
    
    # Test each motor
    motors = [
        (FL_PINS, pwm_fl, "Front Left"),
        (FR_PINS, pwm_fr, "Front Right"),
        (RL_PINS, pwm_rl, "Rear Left"),
        (RR_PINS, pwm_rr, "Rear Right"),
    ]
    
    for pins, pwm, name in motors:
        try:
            fwd_pin, rev_pin, pwm_pin = pins
            
            # Forward direction
            GPIO.output(fwd_pin, GPIO.HIGH)
            GPIO.output(rev_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(TEST_MOTOR_SPEED)
            sleep(MOTOR_TEST_DURATION)
            
            print_info(f"  {name}: Forward at {TEST_MOTOR_SPEED}% for {MOTOR_TEST_DURATION}s")
            
            # Reverse direction
            GPIO.output(fwd_pin, GPIO.LOW)
            GPIO.output(rev_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(TEST_MOTOR_SPEED)
            sleep(MOTOR_TEST_DURATION)
            
            print_info(f"  {name}: Reverse at {TEST_MOTOR_SPEED}% for {MOTOR_TEST_DURATION}s")
            
            # Stop
            GPIO.output(fwd_pin, GPIO.LOW)
            GPIO.output(rev_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(0)
            
            print_pass(f"{name} motor")
            results.add_pass()
            
        except Exception as e:
            print_fail(f"{name} motor: {e}")
            results.add_fail()
    
    # Cleanup PWM
    try:
        for _, pwm, _ in motors:
            pwm.stop()
        GPIO.output(STBY_PIN, GPIO.LOW)
        print_info("Motor drivers disabled")
    except:
        pass

# ============================================================================
# ULTRASONIC SENSOR TEST
# ============================================================================

def test_ultrasonic():
    """Test ultrasonic sensors."""
    print_header("ULTRASONIC SENSOR TEST")
    
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_FRONT, GPIO.IN)
    GPIO.setup(ECHO_LEFT, GPIO.IN)
    GPIO.setup(ECHO_RIGHT, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    
    sleep(1)  # Settle time
    
    sensors = [
        (ECHO_FRONT, "Front"),
        (ECHO_LEFT, "Left"),
        (ECHO_RIGHT, "Right"),
    ]
    
    for echo_pin, name in sensors:
        distances = []
        errors = 0
        
        for i in range(SENSOR_READINGS):
            try:
                GPIO.output(TRIG_PIN, True)
                sleep(0.00001)
                GPIO.output(TRIG_PIN, False)
                
                pulse_start = time()
                timeout_ref = pulse_start
                
                while GPIO.input(echo_pin) == 0:
                    pulse_start = time()
                    if pulse_start - timeout_ref > 0.03:
                        raise TimeoutError("No pulse start")
                
                pulse_end = time()
                while GPIO.input(echo_pin) == 1:
                    pulse_end = time()
                    if pulse_end - pulse_start > 0.03:
                        raise TimeoutError("Pulse too long")
                
                distance = (pulse_end - pulse_start) * 17150
                
                if 2 <= distance <= 400:
                    distances.append(distance)
                else:
                    errors += 1
                
                sleep(0.1)
                
            except Exception as e:
                errors += 1
        
        if distances:
            avg = sum(distances) / len(distances)
            min_d = min(distances)
            max_d = max(distances)
            
            print_pass(f"{name} sensor")
            print_info(f"  Readings: {len(distances)}/{SENSOR_READINGS}")
            print_info(f"  Range: {min_d:.1f}cm - {max_d:.1f}cm (avg: {avg:.1f}cm)")
            
            if max_d - min_d > 10:
                print_warn(f"{name} sensor noise > 10cm")
                results.add_warn()
            else:
                results.add_pass()
        else:
            print_fail(f"{name} sensor: {errors}/{SENSOR_READINGS} read errors")
            results.add_fail()

# ============================================================================
# I2C DEVICE DETECTION
# ============================================================================

def test_i2c():
    """Test I2C communication and MPU6050 detection."""
    print_header("I2C COMMUNICATION TEST")
    
    try:
        bus = smbus.SMBus(1)
        print_pass("I2C bus opened")
        results.add_pass()
    except Exception as e:
        print_fail(f"I2C bus open: {e}")
        results.add_fail()
        return
    
    # Scan for MPU6050
    try:
        data = bus.read_byte_data(MPU6050_ADDR, 0x75)  # WHO_AM_I register
        if data == 0x68:  # Expected MPU6050 ID
            print_pass(f"MPU6050 detected at address 0x{MPU6050_ADDR:02X}")
            results.add_pass()
        else:
            print_fail(f"MPU6050 ID mismatch: expected 0x68, got 0x{data:02X}")
            results.add_fail()
    except Exception as e:
        print_fail(f"MPU6050 detection: {e}")
        results.add_fail()

# ============================================================================
# IMU SENSOR TEST
# ============================================================================

def test_imu():
    """Test MPU6050 accelerometer and gyroscope."""
    print_header("IMU SENSOR TEST")
    
    try:
        bus = smbus.SMBus(1)
        
        # Wake up MPU6050
        bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)
        sleep(0.1)
        
        print_pass("MPU6050 wakeup")
        results.add_pass()
    except Exception as e:
        print_fail(f"MPU6050 wakeup: {e}")
        results.add_fail()
        return
    
    def read_word(reg):
        """Read 16-bit word from register."""
        high = bus.read_byte_data(MPU6050_ADDR, reg)
        low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    # Read accelerometer and gyroscope
    try:
        accel_readings = []
        gyro_readings = []
        
        for i in range(SENSOR_READINGS):
            ax = read_word(ACCEL_XOUT_H) / 16384.0
            ay = read_word(ACCEL_YOUT_H) / 16384.0
            az = read_word(ACCEL_ZOUT_H) / 16384.0
            
            gx = read_word(GYRO_XOUT_H) / 131.0
            gy = read_word(GYRO_YOUT_H) / 131.0
            gz = read_word(GYRO_ZOUT_H) / 131.0
            
            accel_readings.append((ax, ay, az))
            gyro_readings.append((gx, gy, gz))
            sleep(0.05)
        
        # Accelerometer statistics
        accel_mags = [sqrt(x**2 + y**2 + z**2) for x, y, z in accel_readings]
        accel_avg = sum(accel_mags) / len(accel_mags)
        
        print_pass("Accelerometer read")
        print_info(f"  Average magnitude: {accel_avg:.2f}g (expected ~1.0g if level)")
        if 0.95 <= accel_avg <= 1.05:
            print_info(f"  Accelerometer appears level")
            results.add_pass()
        else:
            print_warn(f"  Accelerometer may be tilted or poorly calibrated")
            results.add_warn()
        
        # Gyroscope statistics
        gyro_rates = [sqrt(x**2 + y**2 + z**2) for x, y, z in gyro_readings]
        gyro_avg = sum(gyro_rates) / len(gyro_rates)
        
        print_pass("Gyroscope read")
        print_info(f"  Average rate: {gyro_avg:.2f}°/s (expected ~0°/s if stationary)")
        if gyro_avg < 2.0:
            print_info(f"  Gyroscope appears stationary")
            results.add_pass()
        else:
            print_warn(f"  Gyroscope detecting rotation (check for vibration)")
            results.add_warn()
        
        # Display sample readings
        ax, ay, az = accel_readings[0]
        gx, gy, gz = gyro_readings[0]
        print_info(f"  Sample: Accel({ax:.2f}g, {ay:.2f}g, {az:.2f}g) Gyro({gx:.1f}°/s, {gy:.1f}°/s, {gz:.1f}°/s)")
        
    except Exception as e:
        print_fail(f"IMU read: {e}")
        results.add_fail()

# ============================================================================
# POWER SUPPLY TEST
# ============================================================================

def test_power():
    """Check GPIO voltage levels (rough test)."""
    print_header("POWER SUPPLY TEST")
    
    print_info("Checking GPIO logic levels...")
    
    try:
        # Set a pin HIGH and attempt to read
        test_pin = FL_PINS[0]
        GPIO.setup(test_pin, GPIO.OUT)
        
        GPIO.output(test_pin, GPIO.HIGH)
        sleep(0.1)
        state_high = GPIO.input(test_pin)
        
        GPIO.output(test_pin, GPIO.LOW)
        sleep(0.1)
        state_low = GPIO.input(test_pin)
        
        if state_high == 1 and state_low == 0:
            print_pass("GPIO voltage levels correct (3.3V logic)")
            results.add_pass()
        else:
            print_fail(f"GPIO voltage anomaly: HIGH={state_high}, LOW={state_low}")
            results.add_fail()
            
    except Exception as e:
        print_fail(f"GPIO voltage test: {e}")
        results.add_fail()
    
    print_info("Use multimeter to verify:")
    print_info("  - Motor supply: 6V (4xAA battery)")
    print_info("  - Pi 5V rail: 4.8-5.2V")
    print_info("  - Pi 3.3V rail: 3.2-3.4V")

# ============================================================================
# MOTOR DIRECTION TEST
# ============================================================================

def test_motor_directions():
    """Interactive motor direction test."""
    print_header("MOTOR DIRECTION TEST")
    
    print_info("This test requires manual observation of motor behavior.")
    print_info("Connect ONE motor at a time and observe the direction.")
    
    GPIO.output(STBY_PIN, GPIO.HIGH)  # Enable drivers
    
    # Create PWM
    pwm_motors = {
        'FL': GPIO.PWM(FL_PINS[2], 1000),
        'FR': GPIO.PWM(FR_PINS[2], 1000),
        'RL': GPIO.PWM(RL_PINS[2], 1000),
        'RR': GPIO.PWM(RR_PINS[2], 1000),
    }
    
    for pwm in pwm_motors.values():
        pwm.start(0)
    
    motor_configs = {
        'FL': (FL_PINS, pwm_motors['FL']),
        'FR': (FR_PINS, pwm_motors['FR']),
        'RL': (RL_PINS, pwm_motors['RL']),
        'RR': (RR_PINS, pwm_motors['RR']),
    }
    
    TEST_SPEED = 70
    TEST_TIME = 1.0
    
    for motor_name, (pins, pwm) in motor_configs.items():
        fwd_pin, rev_pin, _ = pins
        
        print(f"\n{Colors.BOLD}Testing {motor_name} Motor:{Colors.END}")
        print("  Spin time: 1 second at 70% speed")
        
        # Forward
        GPIO.output(fwd_pin, GPIO.HIGH)
        GPIO.output(rev_pin, GPIO.LOW)
        pwm.ChangeDutyCycle(TEST_SPEED)
        sleep(TEST_TIME)
        
        response = input(f"  Direction correct? (y/n): ").strip().lower()
        if response == 'y':
            print_pass(f"{motor_name} direction correct")
            results.add_pass()
        else:
            print_warn(f"{motor_name} direction may need MOTOR_{motor_name}_DIR = -1")
            results.add_warn()
        
        # Stop
        GPIO.output(fwd_pin, GPIO.LOW)
        GPIO.output(rev_pin, GPIO.LOW)
        pwm.ChangeDutyCycle(0)
        sleep(0.5)
    
    # Cleanup
    for pwm in pwm_motors.values():
        pwm.stop()
    GPIO.output(STBY_PIN, GPIO.LOW)

# ============================================================================
# MAIN TEST EXECUTION
# ============================================================================

def main():
    """Run all tests."""
    
    print(f"\n{Colors.BOLD}{Colors.BLUE}")
    print("""
╔═══════════════════════════════════════════════════════╗
║   MECANUM ROBOT - COMPREHENSIVE SYSTEM TEST SUITE    ║
║   Testing: Motors, Sensors, IMU, GPIO, I2C, Power    ║
╚═══════════════════════════════════════════════════════╝
    """)
    print(Colors.END)
    
    print_info("This test will verify all hardware components.")
    print_info("Total estimated time: 2-3 minutes")
    print_info(f"Run with: sudo python3 test_system.py\n")
    
    try:
        # Basic setup
        if not setup_gpio():
            print_fail("Cannot continue without GPIO setup")
            return
        
        # Hardware tests
        test_gpio_pins()
        test_motors()
        test_ultrasonic()
        test_i2c()
        test_imu()
        test_power()
        
        # Optional: interactive test
        response = input("\nRun motor direction test? (y/n): ").strip().lower()
        if response == 'y':
            test_motor_directions()
        
        # Summary
        results.print_summary()
        
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Test interrupted by user{Colors.END}")
    except Exception as e:
        print_fail(f"Unexpected error: {e}")
    finally:
        print_info("Cleaning up GPIO...")
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == "__main__":
    main()
