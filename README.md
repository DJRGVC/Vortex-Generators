# Vortex Generator Control System

Arduino-based control system for deployable vortex generators using motor control, IMU sensing, and pressure measurement.

## Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Main Sketches](#main-sketches)
- [Utility Sketches](#utility-sketches)
- [Library Components](#library-components)
- [Quick Start Guide](#quick-start-guide)
- [Workflow](#workflow)

---

## Overview

This system controls vortex generators using a geared DC motor with encoder feedback. The system features:

- **Automatic calibration** at startup to find motor limits and sensor baselines
- **Manual mode** for direct toggle switch control
- **Automatic mode** using IMU roll angle and airspeed (pressure) for dynamic positioning
- **EEPROM storage** for calibration persistence
- **Constant speed motor control** (45 PWM) for predictable movement

---

## Hardware Requirements

### Core Components

- **Arduino Leonardo** (or compatible board with ATmega32u4)
- **Pololu Micro Metal Gearmotor** with encoder (248.98:1 gear ratio, 12 CPR encoder)
- **DRV8833 Motor Driver** (dual H-bridge)
- **BNO-055 IMU** (9-axis absolute orientation sensor)
- **MPRLS Pressure Sensor** (for airspeed calculation)
- **SS-5GL Limit Switch** (mechanical endstop)
- **2x Toggle Switches** (mode selection and deployment control)

### Specifications

- Motor: 6V Pololu Micro Metal Gearmotor (running at 5V)
- Encoder: 12 CPR (48 edges/rev quadrature) × 248.98 gear ratio = 11,951 counts/revolution
- Motor speed: Constant 45 PWM (0-255 scale)

---

## Pin Configuration

| Pin  | Function                  | Connection                          |
|------|---------------------------|-------------------------------------|
| 4    | Deployment Toggle Switch  | One terminal to pin, other to GND   |
| 5    | Limit Switch              | COM to pin, NO to GND               |
| 6    | Motor AIN1 (PWM)          | DRV8833 AIN1                        |
| 7    | Encoder A (interrupt)     | Motor encoder channel A             |
| 8    | Encoder B                 | Motor encoder channel B             |
| 9    | Motor AIN2 (PWM)          | DRV8833 AIN2                        |
| 10   | Mode Toggle Switch        | One terminal to pin, other to GND   |
| A0   | I2C SCL                   | IMU & Pressure Sensor (shared bus)  |
| A1   | I2C SDA                   | IMU & Pressure Sensor (shared bus)  |

**Power:**
- Arduino 5V → DRV8833 VCC, motor power, encoder VCC, sensor VIN
- All grounds must be common

---

## Main Sketches

### 1. `auto_control/` - **Primary Control System**

**Purpose:** Complete vortex generator control with automatic calibration and dual modes

**Features:**
- Automatic calibration runs at every startup
- Manual mode: Pin 4 toggle controls deployment (retracted/extended)
- Automatic mode: IMU roll and airspeed control motor position
- Constant speed (45 PWM) motor control
- EEPROM calibration storage

**Calibration Process:**
1. Auto-moves counter-clockwise to limit switch (MIN = 0)
2. Reads IMU roll offset at MIN position (0° angle of attack reference)
3. Reads baseline pressure (no airflow reference)
4. User manually positions motor to MAX using Pin 4 toggle
5. Pin 10 toggle ON saves MAX position to EEPROM
6. System enters normal operation

**Operation:**
- **Pin 10 OFF (Manual Mode):**
  - Pin 4 OFF = Fully retracted (0°)
  - Pin 4 ON = Fully extended (MAX position)
- **Pin 10 ON (Automatic Mode):**
  - Motor position controlled by IMU roll (-30° to +30°) and airspeed (0-20 m/s)
  - 50/50 weighting between sensors
  - Automatic compensation using calibrated offsets

**Upload this sketch for normal operation**

---

### 2. `calibration_only/` - **Standalone Calibration** (LEGACY)

**Purpose:** Dedicated calibration-only sketch (no longer needed with auto_control)

**Note:** This sketch has been superseded by `auto_control`, which now includes automatic calibration at startup. Use this only if you need to run calibration without the main control loop.

**Process:**
- Same calibration routine as auto_control
- Runs once, saves to EEPROM, then halts
- Intended to be uploaded, run, then replaced with auto_control

**Status:** Legacy - auto_control now handles this automatically

---

## Utility Sketches

### 3. `motor_test/` - **Motor Movement Test**

**Purpose:** Verify basic motor and driver functionality

**Controls:**
- Pin 4 ON (Pin 10 OFF): Motor moves clockwise
- Pin 10 ON (Pin 4 OFF): Motor moves counter-clockwise
- Both OFF or both ON: Motor stopped

**Use when:**
- Testing motor wiring
- Verifying DRV8833 connections
- Checking motor direction
- Troubleshooting motor issues

---

### 4. `Sensor_Control/` - **Multi-Sensor Test**

**Purpose:** Read and display data from all sensors and switches

**Displays:**
- IMU orientation (heading, roll, pitch)
- IMU gyroscope (X, Y, Z)
- IMU accelerometer (X, Y, Z)
- Pressure sensor reading (hPa)
- All switch states (mode, deploy, limit)

**Use when:**
- Testing I2C sensor connections
- Verifying IMU orientation
- Checking pressure sensor readings
- Debugging switch inputs
- Understanding sensor coordinate systems

---

### 5. `I2C_Scanner_Test/` - **I2C Device Scanner**

**Purpose:** Scan I2C bus and identify connected devices

**Identifies:**
- BNO-055 IMU (0x28 or 0x29)
- MPRLS Pressure Sensor (0x18)
- Other I2C devices

**Use when:**
- Verifying I2C wiring (A0=SCL, A1=SDA on Leonardo)
- Troubleshooting I2C communication
- Finding device addresses
- Checking for I2C bus errors

---

### 6. `read_eeprom/` - **EEPROM Calibration Reader**

**Purpose:** Display calibration data stored in EEPROM

**Displays:**
- Magic number (validation)
- MIN position (encoder counts)
- MAX position (encoder counts and degrees)
- IMU roll offset (degrees)
- Pressure baseline (hPa)

**Use when:**
- Verifying calibration was saved correctly
- Checking calibration values without running motor
- Debugging EEPROM issues
- Understanding current calibration state

---

### 7. `Motor_Control/` - **Legacy Motor Control**

**Purpose:** Older motor control sketch (likely deprecated)

**Status:** Legacy - replaced by auto_control

---

## Library Components

All sketches use shared library components (header `.h` and implementation `.cpp` files):

### `IMU_Sensor`
- **Hardware:** Adafruit BNO-055 9-axis IMU
- **Methods:**
  - `begin()`: Initialize sensor
  - `readData()`: Update all sensor readings
  - `getHeading()`, `getRoll()`, `getPitch()`: Orientation
  - `getGyroX/Y/Z()`: Angular velocity
  - `getAccelX/Y/Z()`: Linear acceleration
- **I2C Address:** 0x28 or 0x29

### `Pressure_Sensor`
- **Hardware:** Adafruit MPRLS pressure sensor
- **Methods:**
  - `begin()`: Initialize sensor
  - `readData()`: Update pressure reading
  - `getPressure_hPa()`: Get pressure in hectopascals
- **I2C Address:** 0x18
- **Range:** 0-25 PSI absolute pressure

### `Switch`
- **Purpose:** Debounced switch input
- **Methods:**
  - `begin()`: Initialize with internal pullup
  - `readState()`: Update switch state
  - `isOn()`: Get current state (true = switch closed to GND)
- **Configuration:** Active-low (switch connects pin to GND)

### `LED_Button`
- **Purpose:** LED and button control (used in some test sketches)
- **Methods:**
  - Control LED output
  - Read button input with debouncing

---

## Quick Start Guide

### First Time Setup

1. **Hardware Assembly:**
   - Wire all components according to pin configuration
   - Ensure all grounds are common
   - Double-check I2C connections (A0=SCL, A1=SDA)

2. **Test Hardware:**
   ```
   Upload: I2C_Scanner_Test
   Verify: BNO-055 (0x28/0x29) and MPRLS (0x18) detected
   ```

3. **Test Sensors:**
   ```
   Upload: Sensor_Control
   Verify: IMU and pressure readings appear reasonable
   ```

4. **Test Motor:**
   ```
   Upload: motor_test
   Verify: Motor responds to toggle switches, moves both directions
   ```

5. **Run Calibration & Control:**
   ```
   Upload: auto_control
   Follow calibration prompts:
     - Motor finds MIN at limit switch
     - IMU and pressure baselines recorded
     - Use Pin 4 to position motor to MAX
     - Pin 10 ON to save
   Normal operation begins automatically
   ```

6. **Verify Calibration:**
   ```
   Upload: read_eeprom
   Verify: Calibration values look reasonable
   Upload: auto_control (returns to normal operation)
   ```

---

## Workflow

### Development Workflow

```
I2C_Scanner_Test → Sensor_Control → motor_test → auto_control
     ↓                   ↓              ↓             ↓
  Find I2C         Test sensors    Test motor    Production
  addresses        & switches      movement      operation
```

### Normal Operation

```
Power On → auto_control runs calibration → Normal operation
                                             ↓
                                Pin 10 OFF: Manual Mode
                                Pin 10 ON:  Automatic Mode
```

### Troubleshooting Workflow

```
Problem → I2C_Scanner_Test (I2C issues?)
       → Sensor_Control (sensor data wrong?)
       → motor_test (motor not moving?)
       → read_eeprom (calibration corrupted?)
       → auto_control (full system test)
```

---

## Motor Control Details

### Encoder Calculations
- Encoder CPR: 12 (on motor shaft)
- Quadrature edges: 4× = 48 counts/rev (motor shaft)
- Gear ratio: 248.98:1
- **Total counts/revolution (output):** 11,951
- **Counts/degree:** ~33.2

### Speed Control
- All motor movements use **constant speed of 45 PWM**
- CALIBRATION_SPEED: 45 (finding limits, manual positioning)
- CONTROL_SPEED: 45 (normal operation)
- Deadband: 20 encoder counts (~0.6°)

### Position Control
- Simple bang-bang control (not PID)
- Error > deadband: Move at constant speed
- Error ≤ deadband: Stop motor
- Direction determined by sign of error

---

## Automatic Mode Details

### IMU Mapping
- **Range:** -30° to +30° (angle of attack relative to calibrated 0°)
- **Calibration:** Roll offset measured at MIN position
- **Motor mapping:** -30° → 0 counts, +30° → MAX counts

### Airspeed Mapping
- **Calculation:** Bernoulli equation: v = √(2ΔP/ρ)
- **Air density:** 1.225 kg/m³ (sea level, 15°C)
- **Range:** 0-20 m/s
- **Motor mapping:** 0 m/s → 0 counts, 20 m/s → MAX counts

### Combined Control
- 50/50 weighting between IMU and airspeed
- Target position = (IMU position + airspeed position) / 2
- Adjust weighting in `calculateAutomaticPosition()` as needed

---

## EEPROM Memory Map

| Address | Size | Data                          |
|---------|------|-------------------------------|
| 0       | 4    | Magic number (0xCAFEBABE)     |
| 4       | 4    | MIN position (long)           |
| 8       | 4    | MAX position (long)           |
| 12      | 4    | IMU roll offset (float)       |
| 16      | 4    | Pressure baseline (float)     |

**Total:** 20 bytes

---

## Dependencies

### Required Arduino Libraries
- `Wire.h` (built-in)
- `EEPROM.h` (built-in)
- `Adafruit_Sensor.h`
- `Adafruit_BNO055.h`
- `Adafruit_MPRLS.h`

### Installation
```
Arduino IDE → Tools → Manage Libraries
Search and install:
  - Adafruit BNO055
  - Adafruit MPRLS
  - Adafruit Unified Sensor
```

---

## Troubleshooting

### Motor doesn't move
1. Check DRV8833 wiring (AIN1=6, AIN2=9, power, ground)
2. Verify motor connections (red/black to AOUT1/AOUT2)
3. Upload `motor_test` to verify basic movement
4. Check PWM speed isn't too low (should be 45)

### I2C sensors not found
1. Upload `I2C_Scanner_Test`
2. Verify A0=SCL, A1=SDA (Leonardo I2C pins)
3. Check sensor power (5V or 3.3V depending on sensor)
4. Check for loose wiring
5. Verify correct I2C addresses (BNO055: 0x28/29, MPRLS: 0x18)

### Calibration fails
1. Check limit switch wiring (COM to pin 5, NO to GND)
2. Verify switch triggers when pressed (upload `Sensor_Control`)
3. Increase timeout in code if motor is very slow
4. Manually position to limit before startup

### Motor position drifts
1. Check encoder wiring (A to pin 7, B to pin 8)
2. Verify encoder power (5V, GND)
3. Upload `motor_test` and verify encoder counts change
4. Reduce PWM speed if motor is too fast for encoder

### EEPROM data corrupt
1. Upload `read_eeprom` to check current values
2. Re-run calibration (upload `auto_control`, it will auto-calibrate)
3. Check magic number (should be 0xCAFEBABE)

---

## Notes

- Motor runs at 5V (rated for 6V) - lower speed/torque but adequate
- For higher performance, consider separate 6V motor supply
- Leonardo I2C pins are A0 (SCL) and A1 (SDA), not pins 2/3 like Uno
- All library files are duplicated in each sketch folder for Arduino IDE compatibility
- Encoder counting happens via interrupt - very reliable
- EEPROM has ~100,000 write cycle lifetime - don't calibrate too frequently

---

## License

Educational project for UC Berkeley Introduction to Robotics (EECS 206A) course.

---

## Author

Daniel Grant - Fall 2025
