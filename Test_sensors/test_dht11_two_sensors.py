#!/usr/bin/env python3
"""
Test 2x DHT11 sensors on Raspberry Pi.

Wiring used:
DHT11 #1 DATA -> GPIO21
DHT11 #2 DATA -> GPIO10

Both sensors:
VCC -> 3.3V
GND -> GND
DATA -> GPIO with 4.7k/10k pull-up resistor to 3.3V
"""

import time

import board
import adafruit_dht


# ============================================================================
# DHT11 GPIO CONFIGURATION
# ============================================================================

DHT1_PIN = board.D21   # BCM GPIO21, physical pin 40
DHT2_PIN = board.D10   # BCM GPIO10, physical pin 19

# use_pulseio=False is usually needed on Raspberry Pi Linux
dht1 = adafruit_dht.DHT11(DHT1_PIN, use_pulseio=False)
dht2 = adafruit_dht.DHT11(DHT2_PIN, use_pulseio=False)


# ============================================================================
# READ FUNCTION
# ============================================================================

def read_dht(sensor, name):
    """
    Read one DHT11 sensor.
    Returns: temperature, humidity
    Returns: None, None if failed.
    """

    try:
        temperature = sensor.temperature
        humidity = sensor.humidity

        if temperature is None or humidity is None:
            print(f"{name}: invalid reading")
            return None, None

        return temperature, humidity

    except RuntimeError as e:
        # DHT sensors often fail occasionally. This is normal.
        print(f"{name}: read retry needed: {e}")
        return None, None

    except Exception as e:
        print(f"{name}: serious error: {e}")
        return None, None


# ============================================================================
# MAIN LOOP
# ============================================================================

try:
    print("Starting 2x DHT11 test.")
    print("Press Ctrl+C to stop.\n")

    while True:
        temp1, hum1 = read_dht(dht1, "DHT11 #1 GPIO21")
        temp2, hum2 = read_dht(dht2, "DHT11 #2 GPIO10")

        print("-" * 50)

        if temp1 is not None and hum1 is not None:
            print(f"DHT11 #1 | Temperature: {temp1:.1f} C | Humidity: {hum1:.1f}%")
        else:
            print("DHT11 #1 | No valid reading")

        if temp2 is not None and hum2 is not None:
            print(f"DHT11 #2 | Temperature: {temp2:.1f} C | Humidity: {hum2:.1f}%")
        else:
            print("DHT11 #2 | No valid reading")

        print("-" * 50)
        print()

        # DHT11 is slow. Do not read faster than about once every 2 seconds.
        time.sleep(2.5)

except KeyboardInterrupt:
    print("\nStopping DHT11 test...")

finally:
    dht1.exit()
    dht2.exit()
    print("DHT11 sensors released.")