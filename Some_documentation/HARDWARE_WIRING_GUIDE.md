# Hardware Wiring Guide - Mecanum Robot

## Complete Schematic

### Power Distribution (CRITICAL for reliability)
```
┌─────────────────────────────────────────────────────────────────┐
│                     POWER DISTRIBUTION ARCHITECTURE             │
└─────────────────────────────────────────────────────────────────┘

POWERBANK (5V)                           BATTERY PACK (4xAA, 6V)
    │                                         │
    ├─[100µF + 10µF]─────┐                   ├─[Ferrite Toroid]─┐
    │ (Capacitor Bank)   │                   │                  │
    │                    ↓                   ↓                  │
    └──→ Pi 5V                         TB6612FNG VDD (both)
                                              │
    Pi GND ←─────────────┬──────────────────→ TB6612FNG GND
                         │                    │
                    [STAR POINT]          Motor GND
                         │
                     [Ferrite]
                         │
                    4xAA Battery GND

* All ground connections meet at a single STAR POINT near battery
* Never loop grounds - always star point configuration
* Use 16AWG wire for motor power (6V supply)
* Use 20AWG for logic signals, 18AWG for motor connections
```

### Raspberry Pi GPIO Pinout
```
Pi 4B Pin Layout (40-pin header):

                 ┌─────────────────┐
            3V3  │1  ·· 2│ 5V
(SDA)     GPIO2  │3  ·· 4│ 5V
(SCL)     GPIO3  │5  ·· 6│ GND
          GPIO4  │7  ·· 8│ GPIO14(ECHO_LEFT)
            GND  │9  ·· 10│ GPIO15
          GPIO17 │11 ·· 12│ GPIO18(PWM_FL)
          GPIO27 │13 ·· 14│ GND
          GPIO22 │15 ·· 16│ GPIO23
            3V3  │17 ·· 18│ GPIO24
          GPIO10 │19 ·· 20│ GND
           GPIO9 │21 ·· 22│ GPIO25
          GPIO11 │23 ·· 24│ GPIO8(ECHO_RIGHT)
            GND  │25 ·· 26│ GPIO7
           GPIO0 │27 ·· 28│ GPIO1
           GPIO5 │29 ·· 30│ GND
           GPIO6 │31 ·· 32│ GPIO12(PWM_RL)
          GPIO13 │33 ·· 34│ GND
          GPIO19 │35 ·· 36│ GPIO16(TRIG)
          GPIO26 │37 ·· 38│ GPIO20(STBY)
            GND  │39 ·· 40│ GPIO21
                 └─────────────────┘

Motor Control Pins:
  FL: IN1=27, IN2=17, PWM=18
  FR: IN1=22, IN2=23, PWM=13
  RL: IN1=25, IN2=24, PWM=12
  RR: IN1=6, IN2=5, PWM=19
  STBY: 20

Sensor Pins:
  Ultrasonic TRIG: 16
  Ultrasonic ECHO_FRONT: 26
  Ultrasonic ECHO_LEFT: 14
  Ultrasonic ECHO_RIGHT: 8
  I2C SDA: 2
  I2C SCL: 3

GND Pins: 6, 9, 14, 20, 25, 30, 34, 39 (use multiple for current distribution)
```

### TB6612FNG Motor Driver Wiring

#### TB6612FNG Pinout (DIP-20)
```
         ┌────────────────────┐
     GND │1                20│ VCC (Motor Supply, 6V)
   OUT1A │2                19│ STBY (Standby)
   OUT1B │3                18│ IN1
   OUT2A │4                17│ IN2
   OUT2B │5                16│ (Not used)
    GND  │6                15│ (Not used)
   IN3   │7                14│ IN4
   IN1   │8                13│ VDD (Logic, 3.3V)
   IN2   │9                12│ GND
   PWM1  │10               11│ PWM2
         └────────────────────┘
```

#### Connection for FL Motor (Front Left)
```
TB6612FNG #1 (Driver 1 - Motors FL & FR)

Inputs from Pi:
  IN1 (pin 8)  ← GPIO27 (FL forward)
  IN2 (pin 9)  ← GPIO17 (FL reverse)
  PWM1 (pin 10) ← GPIO18 (PWM_FL)
  IN3 (pin 7)  ← GPIO22 (FR forward)
  IN4 (pin 14) ← GPIO23 (FR reverse)
  PWM2 (pin 11) ← GPIO13 (PWM_FR)
  STBY (pin 19) ← GPIO20 (shared between drivers)
  VDD (pin 13) ← Pi 3.3V
  GND (pins 1,6,12) ← Pi GND (multiple for current)

Outputs to Motors:
  OUT1A (pin 2) → FL Motor (+)
  OUT1B (pin 3) → FL Motor (-)
  OUT2A (pin 4) → FR Motor (+)
  OUT2B (pin 5) → FR Motor (-)

Power:
  VCC (pin 20) ← 6V from battery (with ferrite toroid)
  GND (pins 1,6,12) ← Battery GND (star point)
```

#### Connection for RL Motor (Rear Left)
```
TB6612FNG #2 (Driver 2 - Motors RL & RR)

Inputs from Pi:
  IN1 (pin 8)  ← GPIO25 (RL forward)
  IN2 (pin 9)  ← GPIO24 (RL reverse)
  PWM1 (pin 10) ← GPIO12 (PWM_RL)
  IN3 (pin 7)  ← GPIO6 (RR forward)
  IN4 (pin 14) ← GPIO5 (RR reverse)
  PWM2 (pin 11) ← GPIO19 (PWM_RR)
  STBY (pin 19) ← GPIO20 (shared between drivers)
  VDD (pin 13) ← Pi 3.3V
  GND (pins 1,6,12) ← Pi GND (multiple for current)

Outputs to Motors:
  OUT1A (pin 2) → RL Motor (+)
  OUT1B (pin 3) → RL Motor (-)
  OUT2A (pin 4) → RR Motor (+)
  OUT2B (pin 5) → RR Motor (-)

Power:
  VCC (pin 20) ← 6V from battery (with ferrite toroid, shared with Driver 1)
  GND (pins 1,6,12) ← Battery GND (star point)
```

### Ultrasonic Sensor HC-SR04 Wiring

```
Each HC-SR04 Sensor:
  VCC ─────→ Pi 5V (or use 3.3V regulated)
  GND ─────→ Pi GND
  TRIG ────→ GPIO16 (SHARED across all 3 sensors)
  ECHO ────→ Voltage Divider Circuit

Voltage Divider (for each ECHO to protect 5V signal):
  
  HC-SR04 ECHO (5V max)
       │
       ├──[2kΩ Resistor]──┬──→ GPIO Input (3.3V max)
       │                  │
       │              [2kΩ to GND]
       │                  │
       └──────────────────┴──→ Pi GND

Signal:  Vout = 5V × (2kΩ/(2kΩ+2kΩ)) = 2.5V ✓ (safe for Pi)

Front Sensor:
  VCC → 5V
  GND → Pi GND
  TRIG → GPIO16
  ECHO → GPIO26 (with divider)

Left Sensor:
  VCC → 5V
  GND → Pi GND
  TRIG → GPIO16 (shared)
  ECHO → GPIO14 (with divider)

Right Sensor:
  VCC → 5V
  GND → Pi GND
  TRIG → GPIO16 (shared)
  ECHO → GPIO8 (with divider)
```

**Important**: The voltage dividers MUST be installed to prevent GPIO damage from 5V signals.

### MPU6050 I2C Wiring

```
MPU6050 to Raspberry Pi I2C1 (Bus 1):

MPU6050 Pin    Connection
─────────────────────────────
VCC      ← Pi 3.3V (with 10µF cap to GND at sensor)
GND      ← Pi GND (shortest wire possible)
SDA      ← GPIO2 (with 4.7kΩ pull-up to 3.3V)
SCL      ← GPIO3 (with 4.7kΩ pull-up to 3.3V)
INT      ← Not connected (optional for later use)

Capacitor placement:
  - 10µF electrolytic: VCC to GND (at sensor)
  - 100nF ceramic: VCC to GND (at sensor)
  - Keep leads short (< 5cm)

I2C Pull-up Resistors (usually on Pi module, verify with i2cdetect):
  SDA pull-up: 4.7kΩ to 3.3V
  SCL pull-up: 4.7kΩ to 3.3V

Address: 0x68 (7-bit) - verify with:
  $ i2cdetect -y 1
  Should show "68" in grid
```

### Cable Routing Best Practices

```
        ┌─────────────────────────────┐
        │   Raspberry Pi 4B           │
        │                             │
        │ (keep I2C wires < 30cm,    │
        │  twist SDA+SCL together)   │
        └─────────────────────────────┘
              ↓ (short twisted pair for I2C)
        ┌─────────────────────────────┐
        │   MPU6050 Breakout          │
        │   (mounted on top frame)    │
        └─────────────────────────────┘

        ┌─────────────────────────────┐
        │   Ultrasonic Sensors        │
        │   (front 15cm, left/right   │
        │    at 30° angles)           │
        └─────────────────────────────┘
              ↓ (shielded cable if noisy)
        ┌─────────────────────────────┐
        │   Pi GPIO (TRIG on pin 16)  │
        │   (separate ECHO lines)     │
        └─────────────────────────────┘

        ┌─────────────────────────────┐
        │   TB6612FNG Drivers         │
        │   (mounted on side frame)   │
        └─────────────────────────────┘
              ↓ (6V motor power)
        ┌─────────────────────────────┐
        │   4 TT DC Motors            │
        │   (short leads to drivers)  │
        └─────────────────────────────┘
```

## Cable Routing Rules (Noise Reduction)

1. **Motor Power**: Ferrite toroids on both ends, separate from logic power
2. **I2C Cables**: Twisted pair, shielded if > 20cm, ground shield at Pi end only
3. **PWM Signals**: Route separately from sensor cables (different sides of chassis)
4. **Ultrasonic Trigger**: Use twisted pair with a dedicated return
5. **Ultrasonic ECHO**: Individual shielded cables with ground at Pi end
6. **Ground**: Multiple connections at star point, never daisy-chain

## Assembly Checklist

- [ ] All components powered off
- [ ] Verify GPIO pinout matches code
- [ ] Check all 4x AA batteries in holder (series connection)
- [ ] Verify TB6612FNG VCC is 6V (use multimeter)
- [ ] Verify Pi GPIO is 3.3V logic
- [ ] Test all voltage dividers (should read ~2.5V at ECHO input with 5V source)
- [ ] i2cdetect shows MPU6050 at 0x68
- [ ] Manual motor test (connect one motor, apply 6V to driver inputs)
- [ ] Run autonomous_v2.py with logging enabled
- [ ] Check /tmp/robot_autonomous.log for errors

## Debugging Connection Issues

### I2C Not Working
```bash
# Check device appears
i2cdetect -y 1

# If 0x68 missing, check:
# 1. Voltage on SDA/SCL (should be 3.3V with pull-ups)
# 2. SDA/SCL not swapped
# 3. I2C not already in use

# Enable I2C if disabled
raspi-config → Interface Options → I2C → Enable
```

### Ultrasonic Always Reading 999cm
```bash
# Check GPIO with test_ultra.py
# Verify TRIG pin goes LOW→HIGH→LOW
# Check ECHO pin responds with pulse

# Common issues:
# - TRIG and ECHO swapped
# - Voltage divider values wrong (use 1.5kΩ + 1.5kΩ if 2kΩ unavailable)
# - ECHO pin impedance interfering with divider
```

### Motors Not Spinning
```bash
# Check pins toggle with python3:
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)
GPIO.output(27, GPIO.HIGH)  # Should apply voltage to FL IN1

# If not working:
# 1. Check Pi to TB6612FNG connections
# 2. Verify TB6612FNG power (6V on VCC pin)
# 3. Check STBY pin is HIGH (GPIO 20 = HIGH)
# 4. Measure voltage at motor outputs
```

## Recommended Testing Order

1. **Power Test** (No Pi)
   - Measure voltage at TB6612FNG VCC (should be 6V)
   - Check no shorts

2. **GPIO Test** (Pi powered, motors disconnected)
   ```bash
   sudo python3 -c "
   import RPi.GPIO as GPIO
   GPIO.setmode(GPIO.BCM)
   GPIO.setup([27,17,18,22,23,13,25,24,12,6,5,19,20], GPIO.OUT)
   GPIO.output(20, GPIO.HIGH)  # STBY
   [GPIO.output(pin, GPIO.HIGH) for pin in [27,17,18,22,23,13]]
   print('GPIO test complete')
   GPIO.cleanup()
   "
   ```

3. **Motor Test** (One motor at a time)
   - Connect motor FL
   - Run original_bt_control.py for manual testing
   - Verify direction matches code

4. **I2C Test** (IMU power check)
   ```bash
   i2cdetect -y 1  # Should show 68
   python3 test.py  # Should print IMU data
   ```

5. **Ultrasonic Test**
   ```bash
   python3 test_ultra.py  # Should print distances
   ```

6. **Full Autonomous Test**
   ```bash
   sudo python3 autonomous_v2.py  # Monitor /tmp/robot_autonomous.log
   ```

---

**Version**: 2.0
**Last Updated**: May 7, 2026
**Status**: Ready for Assembly
