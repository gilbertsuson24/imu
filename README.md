# IMU Testing Toolkit for Raspberry Pi

A comprehensive Python toolkit for testing IMU (Inertial Measurement Unit) sensors on Raspberry Pi 4/5. Supports automatic sensor detection, hardware and simulation modes, and real-time data visualization.

## Supported Sensors

- **MPU6050/MPU6500** - 6-axis accelerometer + gyroscope
- **MPU9250/MPU9255** - 9-axis accelerometer + gyroscope + magnetometer
- **BNO055** - 9-axis sensor with sensor fusion (quaternion/Euler output)
- **LSM6DS3/LSM6DS33** - 6-axis accelerometer + gyroscope

## Features

- ✅ **Automatic Detection** - Scans I2C bus and auto-detects connected IMU sensors
- ✅ **Hardware & Simulation Modes** - Test with real hardware or simulate without any sensor
- ✅ **Multiple Drivers** - Supports Adafruit CircuitPython drivers and raw SMBus access
- ✅ **Real-time Data** - Display acceleration, gyroscope, magnetometer, temperature, and orientation
- ✅ **Live Plotting** - Optional matplotlib-based real-time data visualization
- ✅ **Configurable** - Adjustable sample rates, duration, and output modes
- ✅ **Error Handling** - Graceful error handling and automatic fallback to simulation
- ✅ **Portable** - Works on Raspberry Pi 4/5, with or without GUI

## Installation

### 1. Enable I2C on Raspberry Pi

```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
```

### 2. Add user to i2c group (if needed)

```bash
sudo usermod -a -G i2c $USER
# Log out and log back in for changes to take effect
```

### 3. Create and activate virtual environment (Recommended)

Modern Raspberry Pi OS uses externally-managed Python environments. Use a virtual environment:

```bash
# Install python3-venv if not already installed
sudo apt update
sudo apt install -y python3-venv python3-full

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Your prompt should now show (venv)
```

### 4. Install dependencies

```bash
# Make sure virtual environment is activated (you should see (venv) in your prompt)
pip install -r requirements.txt

# For hardware support with specific sensors, uncomment the relevant line in requirements.txt:
# adafruit-circuitpython-mpu6050    # For MPU6050
# adafruit-circuitpython-bno055     # For BNO055
# adafruit-circuitpython-lsm6ds     # For LSM6DS3/LSM6DS33
```

**Note:** Always activate the virtual environment before running the script:
```bash
source venv/bin/activate
python3 imu_test.py
```

## Quick Start

### Auto-detect and run with real sensor

```bash
# Make sure virtual environment is activated
source venv/bin/activate
python3 imu_test.py
```

### Run in simulation mode (no hardware required)

```bash
source venv/bin/activate
python3 imu_test.py --simulate
```

### Run with live plotting at 100 Hz for 10 seconds

```bash
source venv/bin/activate
python3 imu_test.py --rate 100 --plot --duration 10
```

### Run in simulation mode with plot

```bash
source venv/bin/activate
python3 imu_test.py --simulate --plot --rate 50
```

## Command Line Options

```
--simulate          Run in simulation mode (default: auto-detect)
--rate RATE         Sample rate in Hz (default: 50)
--plot              Enable live plotting (requires matplotlib)
--duration SECONDS  Run duration in seconds (default: infinite until Ctrl+C)
```

## Wiring

See `pinout.txt` for detailed GPIO pin mappings for each sensor model.

### Quick Reference (MPU6050/MPU9250)

| Signal | BCM Pin | Physical Pin |
|--------|---------|--------------|
| VCC    | 3.3V    | 1 or 17      |
| GND    | GND     | 6, 9, 14, etc|
| SDA    | GPIO 2  | 3            |
| SCL    | GPIO 3  | 5            |

**⚠️ Important:** Use 3.3V (NOT 5V) to avoid damaging the sensor and Raspberry Pi.

## File Structure

- `imu_test.py` - Main testing script with auto-detection and data acquisition
- `pinout.txt` - GPIO pin mappings and wiring diagrams
- `requirements.txt` - Python dependencies
- `README.md` - This file

## Output Data

The toolkit displays:

- **Acceleration** (m/s²) - X, Y, Z axes
- **Gyroscope** (deg/s) - X, Y, Z axes  
- **Magnetometer** (µT) - X, Y, Z axes (if available)
- **Temperature** (°C)
- **Quaternion** (w, x, y, z) - For sensors with fusion (BNO055)
- **Euler Angles** (roll, pitch, yaw) - For sensors with fusion (BNO055)
- **Sample Rate** - Actual sampling rate achieved
- **Timestamp** - Elapsed time since start

## Troubleshooting

### No device detected

- Check I2C is enabled: `sudo raspi-config`
- Verify wiring connections
- Check I2C devices: `i2cdetect -y 1`
- Add user to i2c group: `sudo usermod -a -G i2c $USER`

### Permission denied

```bash
sudo usermod -a -G i2c $USER
# Log out and log back in
```

### Wrong readings

- Verify 3.3V connection (not 5V)
- Check wiring and pull-up resistors
- Ensure sensor is properly powered

### Missing dependencies

Install required packages:
```bash
pip install smbus2 numpy matplotlib
```

For specific sensor support:
```bash
pip install adafruit-circuitpython-mpu6050  # For MPU6050
pip install adafruit-circuitpython-bno055   # For BNO055
```

## License

MIT License - see LICENSE file for details

## Contributing

Contributions are welcome! Feel free to:
- Add support for new IMU models
- Improve simulation accuracy
- Enhance error handling
- Add new features

## References

- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- [I2C Communication](https://www.raspberrypi.org/documentation/hardware/raspberrypi/i2c/README.md)
- [Adafruit CircuitPython](https://circuitpython.readthedocs.io/)

