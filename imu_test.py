#!/usr/bin/env python3
"""
IMU Testing Toolkit for Raspberry Pi
=====================================

A robust Python script for testing IMU sensors (MPU6050, MPU9250, BNO055, LSM6DS3)
on Raspberry Pi with automatic detection, hardware/simulation modes, and real-time
data display.

Features:
- Automatic IMU detection via I2C address scan
- Hardware mode (real sensor) and simulation mode (no hardware needed)
- Supports multiple IMU models with appropriate drivers
- Movement direction detection (not moving, forward, backward, leftward, rightward)
- Realistic synthetic data generation for simulation
- Live data output with timestamps and sample rates
- Optional live plotting with matplotlib
- CLI flags for configuration
- Graceful error handling and shutdown

Usage Examples:
    # Auto-detect and run with real sensor
    python3 imu_test.py

    # Run in simulation mode
    python3 imu_test.py --simulate

    # Run at 100 Hz with live plot for 10 seconds
    python3 imu_test.py --rate 100 --plot --duration 10

    # Run in simulation mode with plot
    python3 imu_test.py --simulate --plot --rate 50

License: MIT
Author: IMU Testing Toolkit
"""

# MIT License
#
# Copyright (c) 2024 IMU Testing Toolkit
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import signal
import sys
import time
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Try importing optional dependencies gracefully
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("Warning: numpy not available. Simulation mode and advanced features disabled.")

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# I2C and sensor libraries
try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False
    print("Warning: smbus2 not available. Hardware mode disabled.")

# Adafruit CircuitPython drivers (optional)
try:
    import board
    import busio
    HAS_ADAFRUIT_BLINKA = True
except ImportError:
    HAS_ADAFRUIT_BLINKA = False

# Try importing Adafruit sensor drivers
HAS_MPU6050 = False
HAS_BNO055 = False
HAS_LSM6DS = False
HAS_MPU9250 = False

if HAS_ADAFRUIT_BLINKA:
    try:
        import adafruit_mpu6050
        HAS_MPU6050 = True
    except ImportError:
        pass

    try:
        import adafruit_bno055
        HAS_BNO055 = True
    except ImportError:
        pass

    try:
        import adafruit_lsm6ds.lsm6ds33
        HAS_LSM6DS = True
    except ImportError:
        pass

    try:
        import adafruit_mpu9250
        HAS_MPU9250 = True
    except ImportError:
        pass


class IMUModel(Enum):
    """Supported IMU models."""
    UNKNOWN = "unknown"
    MPU6050 = "MPU6050"
    MPU9250 = "MPU9250"
    BNO055 = "BNO055"
    LSM6DS3 = "LSM6DS3"
    LSM6DS33 = "LSM6DS33"


@dataclass
class IMUData:
    """Container for IMU sensor data."""
    timestamp: float
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    temperature: float = 25.0
    quaternion_w: float = 1.0
    quaternion_x: float = 0.0
    quaternion_y: float = 0.0
    quaternion_z: float = 0.0
    euler_roll: float = 0.0
    euler_pitch: float = 0.0
    euler_yaw: float = 0.0


class IMUDetector:
    """Detects IMU sensors via I2C address scan."""
    
    # Common I2C addresses for IMU sensors
    I2C_ADDRESSES = {
        0x68: IMUModel.MPU6050,  # MPU6050/MPU6500/MPU9250 (AD0 LOW)
        0x69: IMUModel.MPU6050,  # MPU6050/MPU6500/MPU9250 (AD0 HIGH)
        0x28: IMUModel.BNO055,   # BNO055 (default)
        0x29: IMUModel.BNO055,   # BNO055 (alternative)
        0x6A: IMUModel.LSM6DS3,  # LSM6DS3/LSM6DS33 (default)
        0x6B: IMUModel.LSM6DS3,  # LSM6DS3/LSM6DS33 (alternative)
    }
    
    @staticmethod
    def scan_i2c_bus(bus_number: int = 1) -> List[Tuple[int, IMUModel]]:
        """
        Scan I2C bus for connected IMU sensors.
        
        Args:
            bus_number: I2C bus number (default: 1 for Raspberry Pi)
            
        Returns:
            List of tuples (address, model) for detected IMUs
        """
        if not HAS_SMBUS:
            return []
        
        detected = []
        try:
            with SMBus(bus_number) as bus:
                # Scan addresses 0x08 to 0x77 (valid I2C address range)
                for address in range(0x08, 0x78):
                    try:
                        # Try to read from the address
                        bus.read_byte(address)
                        # If successful, check if it's a known IMU address
                        if address in IMUDetector.I2C_ADDRESSES:
                            model = IMUDetector.I2C_ADDRESSES[address]
                            detected.append((address, model))
                    except (IOError, OSError):
                        # Address not responding, continue scanning
                        continue
        except (PermissionError, FileNotFoundError) as e:
            print(f"Error accessing I2C bus: {e}")
            print("Hint: Enable I2C with: sudo raspi-config")
            print("Hint: Add user to i2c group: sudo usermod -a -G i2c $USER")
        except Exception as e:
            print(f"Unexpected error during I2C scan: {e}")
        
        return detected


class IMUSimulator:
    """Generates realistic synthetic IMU data for testing without hardware."""
    
    def __init__(self, sample_rate: float = 50.0):
        """
        Initialize IMU simulator.
        
        Args:
            sample_rate: Sample rate in Hz
        """
        if not HAS_NUMPY:
            raise RuntimeError("numpy is required for simulation mode")
        
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        self.time = 0.0
        
        # Simulation parameters
        self.gravity = np.array([0.0, 0.0, 9.81])  # Gravity vector (m/s²)
        self.accel_noise_std = 0.05  # Acceleration noise (m/s²)
        self.gyro_noise_std = 0.1    # Gyroscope noise (deg/s)
        self.mag_noise_std = 0.5     # Magnetometer noise (µT)
        self.accel_drift = np.array([0.001, 0.001, 0.001])  # Slow drift (m/s²/s)
        self.gyro_drift = np.array([0.01, 0.01, 0.01])      # Gyro drift (deg/s/s)
        
        # State variables
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w, x, y, z]
        self.angular_velocity = np.zeros(3)  # rad/s
        
        # Temperature simulation
        self.temperature = 25.0
        self.temp_target = 25.0 + np.random.normal(0, 2.0)
    
    def generate_data(self) -> IMUData:
        """
        Generate synthetic IMU data.
        
        Returns:
            IMUData object with synthetic sensor readings
        """
        self.time += self.dt
        
        # Update temperature (slow drift towards target)
        self.temperature += (self.temp_target - self.temperature) * 0.01
        self.temperature += np.random.normal(0, 0.1)
        
        # Simulate slow rotation (sine wave)
        angular_rate = 0.1 * np.sin(2 * np.pi * 0.1 * self.time)  # rad/s
        self.angular_velocity = np.array([
            angular_rate * 0.5,
            angular_rate * 0.3,
            angular_rate * 0.2
        ])
        
        # Update orientation (simplified quaternion integration)
        # In real implementation, use proper quaternion integration
        q_w, q_x, q_y, q_z = self.orientation
        wx, wy, wz = self.angular_velocity * self.dt
        
        # Simple rotation update (approximate)
        self.orientation = np.array([
            q_w - 0.5 * (wx*q_x + wy*q_y + wz*q_z),
            q_x + 0.5 * (wx*q_w + wy*q_z - wz*q_y),
            q_y + 0.5 * (wy*q_w + wz*q_x - wx*q_z),
            q_z + 0.5 * (wz*q_w + wx*q_y - wy*q_x)
        ])
        # Normalize quaternion
        norm = np.linalg.norm(self.orientation)
        if norm > 0:
            self.orientation /= norm
        
        # Calculate acceleration (gravity rotated by orientation)
        # Rotate gravity vector by quaternion
        q_w, q_x, q_y, q_z = self.orientation
        gx, gy, gz = self.gravity
        
        # Quaternion rotation
        accel_gravity = np.array([
            (1 - 2*(q_y**2 + q_z**2)) * gx + 2*(q_x*q_y - q_w*q_z) * gy + 2*(q_x*q_z + q_w*q_y) * gz,
            2*(q_x*q_y + q_w*q_z) * gx + (1 - 2*(q_x**2 + q_z**2)) * gy + 2*(q_y*q_z - q_w*q_x) * gz,
            2*(q_x*q_z - q_w*q_y) * gx + 2*(q_y*q_z + q_w*q_x) * gy + (1 - 2*(q_x**2 + q_y**2)) * gz
        ])
        
        # Add drift and noise
        self.accel_bias += self.accel_drift * self.dt
        self.gyro_bias += self.gyro_drift * self.dt
        
        accel = accel_gravity + self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
        gyro_rad = self.angular_velocity + self.gyro_bias + np.random.normal(0, np.radians(self.gyro_noise_std), 3)
        gyro_deg = np.degrees(gyro_rad)
        
        # Magnetometer (simulate Earth's magnetic field)
        mag_field = np.array([20.0, 0.0, 45.0])  # µT (typical values)
        mag = mag_field + np.random.normal(0, self.mag_noise_std, 3)
        
        # Convert quaternion to Euler angles (simplified)
        q_w, q_x, q_y, q_z = self.orientation
        roll = np.arctan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x**2 + q_y**2))
        pitch = np.arcsin(2*(q_w*q_y - q_z*q_x))
        yaw = np.arctan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y**2 + q_z**2))
        
        return IMUData(
            timestamp=self.time,
            accel_x=accel[0],
            accel_y=accel[1],
            accel_z=accel[2],
            gyro_x=gyro_deg[0],
            gyro_y=gyro_deg[1],
            gyro_z=gyro_deg[2],
            mag_x=mag[0],
            mag_y=mag[1],
            mag_z=mag[2],
            temperature=self.temperature,
            quaternion_w=q_w,
            quaternion_x=q_x,
            quaternion_y=q_y,
            quaternion_z=q_z,
            euler_roll=np.degrees(roll),
            euler_pitch=np.degrees(pitch),
            euler_yaw=np.degrees(yaw)
        )


class IMUDriver:
    """Base class for IMU drivers."""
    
    def __init__(self, model: IMUModel, address: int, sample_rate: float = 50.0):
        """
        Initialize IMU driver.
        
        Args:
            model: IMU model
            address: I2C address
            sample_rate: Desired sample rate (Hz)
        """
        self.model = model
        self.address = address
        self.sample_rate = sample_rate
        self.sensor = None
    
    def initialize(self) -> bool:
        """Initialize the sensor. Returns True if successful."""
        raise NotImplementedError
    
    def read_data(self) -> Optional[IMUData]:
        """Read sensor data. Returns None if error."""
        raise NotImplementedError
    
    def close(self):
        """Clean up resources."""
        pass


class MPU6050Driver(IMUDriver):
    """Driver for MPU6050/MPU6500 using Adafruit library or raw SMBus."""
    
    def initialize(self) -> bool:
        """Initialize MPU6050 sensor."""
        if HAS_MPU6050 and HAS_ADAFRUIT_BLINKA:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_mpu6050.MPU6050(i2c, address=self.address)
                return True
            except Exception as e:
                print(f"Error initializing MPU6050 with Adafruit library: {e}")
        
        # Fallback to raw SMBus (basic implementation)
        if HAS_SMBUS:
            try:
                self.bus = SMBus(1)
                # Wake up MPU6050 (set power management register)
                self.bus.write_byte_data(self.address, 0x6B, 0x00)
                time.sleep(0.1)
                return True
            except Exception as e:
                print(f"Error initializing MPU6050 with SMBus: {e}")
        
        return False
    
    def read_data(self) -> Optional[IMUData]:
        """Read MPU6050 data."""
        if self.sensor:  # Adafruit library
            try:
                accel = self.sensor.acceleration
                gyro = self.sensor.gyro
                temp = self.sensor.temperature
                return IMUData(
                    timestamp=time.time(),
                    accel_x=accel[0],
                    accel_y=accel[1],
                    accel_z=accel[2],
                    gyro_x=gyro[0],
                    gyro_y=gyro[1],
                    gyro_z=gyro[2],
                    temperature=temp
                )
            except Exception as e:
                print(f"Error reading MPU6050: {e}")
                return None
        elif HAS_SMBUS:  # Raw SMBus
            try:
                # Read accelerometer (registers 0x3B-0x40)
                accel_data = self.bus.read_i2c_block_data(self.address, 0x3B, 6)
                accel_x = (accel_data[0] << 8 | accel_data[1]) / 16384.0 * 9.81
                accel_y = (accel_data[2] << 8 | accel_data[3]) / 16384.0 * 9.81
                accel_z = (accel_data[4] << 8 | accel_data[5]) / 16384.0 * 9.81
                
                # Read gyroscope (registers 0x43-0x48)
                gyro_data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
                gyro_x = (gyro_data[0] << 8 | gyro_data[1]) / 131.0
                gyro_y = (gyro_data[2] << 8 | gyro_data[3]) / 131.0
                gyro_z = (gyro_data[4] << 8 | gyro_data[5]) / 131.0
                
                # Read temperature (registers 0x41-0x42)
                temp_data = self.bus.read_i2c_block_data(self.address, 0x41, 2)
                temp = (temp_data[0] << 8 | temp_data[1]) / 340.0 + 36.53
                
                return IMUData(
                    timestamp=time.time(),
                    accel_x=accel_x,
                    accel_y=accel_y,
                    accel_z=accel_z,
                    gyro_x=gyro_x,
                    gyro_y=gyro_y,
                    gyro_z=gyro_z,
                    temperature=temp
                )
            except Exception as e:
                print(f"Error reading MPU6050 via SMBus: {e}")
                return None
        
        return None
    
    def close(self):
        """Close SMBus connection."""
        if hasattr(self, 'bus'):
            self.bus.close()


class BNO055Driver(IMUDriver):
    """Driver for BNO055 using Adafruit library."""
    
    def initialize(self) -> bool:
        """Initialize BNO055 sensor."""
        if HAS_BNO055 and HAS_ADAFRUIT_BLINKA:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.address)
                return True
            except Exception as e:
                print(f"Error initializing BNO055: {e}")
                return False
        else:
            print("BNO055 requires adafruit-circuitpython-bno055 library")
            return False
    
    def read_data(self) -> Optional[IMUData]:
        """Read BNO055 data."""
        if not self.sensor:
            return None
        
        try:
            # BNO055 provides fused data
            accel = self.sensor.linear_acceleration
            gyro = self.sensor.gyro
            mag = self.sensor.magnetic
            temp = self.sensor.temperature
            quat = self.sensor.quaternion
            euler = self.sensor.euler
            
            return IMUData(
                timestamp=time.time(),
                accel_x=accel[0] if accel else 0.0,
                accel_y=accel[1] if accel else 0.0,
                accel_z=accel[2] if accel else 0.0,
                gyro_x=gyro[0] if gyro else 0.0,
                gyro_y=gyro[1] if gyro else 0.0,
                gyro_z=gyro[2] if gyro else 0.0,
                mag_x=mag[0] if mag else 0.0,
                mag_y=mag[1] if mag else 0.0,
                mag_z=mag[2] if mag else 0.0,
                temperature=temp if temp else 25.0,
                quaternion_w=quat[0] if quat else 1.0,
                quaternion_x=quat[1] if quat else 0.0,
                quaternion_y=quat[2] if quat else 0.0,
                quaternion_z=quat[3] if quat else 0.0,
                euler_roll=euler[0] if euler else 0.0,
                euler_pitch=euler[1] if euler else 0.0,
                euler_yaw=euler[2] if euler else 0.0
            )
        except Exception as e:
            print(f"Error reading BNO055: {e}")
            return None


class LSM6DSDriver(IMUDriver):
    """Driver for LSM6DS3/LSM6DS33 using Adafruit library."""
    
    def initialize(self) -> bool:
        """Initialize LSM6DS sensor."""
        if HAS_LSM6DS and HAS_ADAFRUIT_BLINKA:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c, address=self.address)
                return True
            except Exception as e:
                print(f"Error initializing LSM6DS: {e}")
                return False
        else:
            print("LSM6DS requires adafruit-circuitpython-lsm6ds library")
            return False
    
    def read_data(self) -> Optional[IMUData]:
        """Read LSM6DS data."""
        if not self.sensor:
            return None
        
        try:
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
            temp = self.sensor.temperature
            
            return IMUData(
                timestamp=time.time(),
                accel_x=accel[0],
                accel_y=accel[1],
                accel_z=accel[2],
                gyro_x=gyro[0],
                gyro_y=gyro[1],
                gyro_z=gyro[2],
                temperature=temp
            )
        except Exception as e:
            print(f"Error reading LSM6DS: {e}")
            return None


def create_driver(model: IMUModel, address: int, sample_rate: float = 50.0) -> Optional[IMUDriver]:
    """
    Create appropriate driver for IMU model.
    
    Args:
        model: IMU model
        address: I2C address
        sample_rate: Sample rate in Hz
        
    Returns:
        IMUDriver instance or None if unsupported
    """
    if model == IMUModel.MPU6050:
        return MPU6050Driver(model, address, sample_rate)
    elif model == IMUModel.BNO055:
        return BNO055Driver(model, address, sample_rate)
    elif model in (IMUModel.LSM6DS3, IMUModel.LSM6DS33):
        return LSM6DSDriver(model, address, sample_rate)
    else:
        return None


class IMUTester:
    """Main IMU testing class."""
    
    def __init__(self, simulate: bool = False, sample_rate: float = 50.0, 
                 plot: bool = False, duration: Optional[float] = None,
                 show_raw: bool = False, movement_threshold: float = 0.5,
                 sensitivity: str = 'medium'):
        """
        Initialize IMU tester.
        
        Args:
            simulate: Run in simulation mode
            sample_rate: Sample rate in Hz
            plot: Enable live plotting
            duration: Run duration in seconds (None for infinite)
            show_raw: Show raw acceleration values instead of movement direction
            movement_threshold: Threshold for movement detection in m/s²
            sensitivity: Detection sensitivity: 'low', 'medium', or 'high'
        """
        self.simulate = simulate
        self.sample_rate = sample_rate
        self.plot = plot and HAS_MATPLOTLIB
        self.duration = duration
        self.show_raw = show_raw
        self.running = True
        self.driver: Optional[IMUDriver] = None
        self.simulator: Optional[IMUSimulator] = None
        self.start_time = time.time()
        self.sample_count = 0
        
        # Movement detection parameters based on sensitivity
        self.movement_threshold = movement_threshold  # m/s² threshold for detecting movement
        self.gravity_threshold = 8.0   # m/s² - minimum gravity to consider Z-axis as up
        self.accel_history = []        # Store recent acceleration values for smoothing
        self.base_gravity = None       # Baseline gravity vector for comparison
        self.calibration_samples = []  # Separate buffer for calibration (not limited)
        
        # Adjust parameters based on sensitivity
        if sensitivity == 'high':
            # Fast response, minimal smoothing
            self.history_size = 2
            self.base_samples = 5
        elif sensitivity == 'low':
            # Slower but more stable
            self.history_size = 4
            self.base_samples = 15
        else:  # medium
            # Balanced
            self.history_size = 3
            self.base_samples = 10
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Plotting setup
        if self.plot:
            self._setup_plotting()
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print("\nShutting down gracefully...")
        self.running = False
    
    def _setup_plotting(self):
        """Setup matplotlib for live plotting."""
        if not HAS_MATPLOTLIB:
            return
        
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('IMU Data - Live Plot')
        
        # Initialize data arrays
        self.plot_time = []
        self.plot_accel = [[], [], []]
        self.plot_gyro = [[], [], []]
        self.plot_mag = [[], [], []]
        self.plot_temp = []
        
        # Setup subplots
        self.axes[0, 0].set_title('Acceleration (m/s²)')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Acceleration')
        self.axes[0, 0].grid(True)
        self.accel_lines = [self.axes[0, 0].plot([], [], label=label)[0] 
                           for label in ['X', 'Y', 'Z']]
        self.axes[0, 0].legend()
        
        self.axes[0, 1].set_title('Gyroscope (deg/s)')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Angular Velocity')
        self.axes[0, 1].grid(True)
        self.gyro_lines = [self.axes[0, 1].plot([], [], label=label)[0] 
                          for label in ['X', 'Y', 'Z']]
        self.axes[0, 1].legend()
        
        self.axes[1, 0].set_title('Magnetometer (µT)')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].set_ylabel('Magnetic Field')
        self.axes[1, 0].grid(True)
        self.mag_lines = [self.axes[1, 0].plot([], [], label=label)[0] 
                         for label in ['X', 'Y', 'Z']]
        self.axes[1, 0].legend()
        
        self.axes[1, 1].set_title('Temperature (°C)')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].set_ylabel('Temperature')
        self.axes[1, 1].grid(True)
        self.temp_line = self.axes[1, 1].plot([], [])[0]
        
        plt.tight_layout()
        plt.ion()
        plt.show()
    
    def _update_plot(self, data: IMUData):
        """Update live plot with new data."""
        if not self.plot:
            return
        
        elapsed = data.timestamp - self.start_time
        
        # Keep only last 100 samples for performance
        max_samples = 100
        if len(self.plot_time) >= max_samples:
            self.plot_time.pop(0)
            for i in range(3):
                self.plot_accel[i].pop(0)
                self.plot_gyro[i].pop(0)
                self.plot_mag[i].pop(0)
            self.plot_temp.pop(0)
        
        self.plot_time.append(elapsed)
        self.plot_accel[0].append(data.accel_x)
        self.plot_accel[1].append(data.accel_y)
        self.plot_accel[2].append(data.accel_z)
        self.plot_gyro[0].append(data.gyro_x)
        self.plot_gyro[1].append(data.gyro_y)
        self.plot_gyro[2].append(data.gyro_z)
        self.plot_mag[0].append(data.mag_x)
        self.plot_mag[1].append(data.mag_y)
        self.plot_mag[2].append(data.mag_z)
        self.plot_temp.append(data.temperature)
        
        # Update plots
        for i, line in enumerate(self.accel_lines):
            line.set_data(self.plot_time, self.plot_accel[i])
        for i, line in enumerate(self.gyro_lines):
            line.set_data(self.plot_time, self.plot_gyro[i])
        for i, line in enumerate(self.mag_lines):
            line.set_data(self.plot_time, self.plot_mag[i])
        self.temp_line.set_data(self.plot_time, self.plot_temp)
        
        # Adjust axes limits
        for ax in self.axes.flat:
            ax.relim()
            ax.autoscale_view()
        
        plt.draw()
        plt.pause(0.001)
    
    def initialize(self) -> bool:
        """Initialize IMU or simulator."""
        if self.simulate:
            if not HAS_NUMPY:
                print("Error: numpy is required for simulation mode")
                return False
            self.simulator = IMUSimulator(self.sample_rate)
            print("Initialized: Simulation mode")
            return True
        else:
            # Try to detect IMU
            detected = IMUDetector.scan_i2c_bus(1)
            if not detected:
                print("No IMU detected on I2C bus. Switching to simulation mode...")
                if not HAS_NUMPY:
                    print("Error: numpy is required for simulation mode")
                    return False
                self.simulate = True
                self.simulator = IMUSimulator(self.sample_rate)
                print("Initialized: Simulation mode (fallback)")
                return True
            
            # Use first detected IMU
            address, model = detected[0]
            print(f"Detected: {model.value} at I2C address 0x{address:02X}")
            
            self.driver = create_driver(model, address, self.sample_rate)
            if not self.driver:
                print(f"Error: No driver available for {model.value}")
                return False
            
            if not self.driver.initialize():
                print(f"Error: Failed to initialize {model.value}")
                return False
            
            print(f"Initialized: {model.value} hardware mode")
            return True
    
    def detect_movement(self, data: IMUData) -> str:
        """
        Detect movement direction from acceleration data.
        Uses faster response algorithm with minimal smoothing.
        
        Args:
            data: IMU data containing acceleration values
            
        Returns:
            Movement direction string: "not moving", "forward", "backward", "leftward", "rightward"
        """
        current_sample = (data.accel_x, data.accel_y, data.accel_z)
        
        # Establish baseline gravity vector on startup (separate from history)
        if self.base_gravity is None:
            # Collect calibration samples (unlimited size)
            self.calibration_samples.append(current_sample)
            
            if len(self.calibration_samples) < self.base_samples:
                # Still calibrating - show progress
                return f"calibrating... ({len(self.calibration_samples)}/{self.base_samples})"
            
            # Calculate baseline gravity vector (average of calibration samples)
            base_x = sum(a[0] for a in self.calibration_samples) / len(self.calibration_samples)
            base_y = sum(a[1] for a in self.calibration_samples) / len(self.calibration_samples)
            base_z = sum(a[2] for a in self.calibration_samples) / len(self.calibration_samples)
            self.base_gravity = (base_x, base_y, base_z)
            # Clear calibration samples to free memory
            self.calibration_samples = []
            # Initialize history with current sample
            self.accel_history.append(current_sample)
            return "calibrating..."  # One more sample to initialize history
        
        # Add to history for minimal smoothing (only 2-4 samples)
        self.accel_history.append(current_sample)
        if len(self.accel_history) > self.history_size:
            self.accel_history.pop(0)
        
        # Use recent samples for faster response (minimal smoothing)
        if len(self.accel_history) < 2:
            return "calculating..."
        
        # Quick average of last 2-3 samples (faster than full history)
        recent_samples = self.accel_history[-2:] if len(self.accel_history) >= 2 else self.accel_history
        avg_x = sum(a[0] for a in recent_samples) / len(recent_samples)
        avg_y = sum(a[1] for a in recent_samples) / len(recent_samples)
        avg_z = sum(a[2] for a in recent_samples) / len(recent_samples)
        
        # Calculate change from baseline (removes gravity and static offset)
        delta_x = avg_x - self.base_gravity[0]
        delta_y = avg_y - self.base_gravity[1]
        delta_z = avg_z - self.base_gravity[2]
        
        # Determine which axis has gravity in baseline (sensor orientation)
        base_magnitudes = [abs(self.base_gravity[0]), abs(self.base_gravity[1]), abs(self.base_gravity[2])]
        gravity_axis = base_magnitudes.index(max(base_magnitudes))
        
        # Extract linear acceleration (movement) by removing gravity component
        if gravity_axis == 2:  # Z-axis has gravity (normal orientation)
            # X and Y are horizontal movement axes
            linear_x = delta_x
            linear_y = delta_y
        elif gravity_axis == 0:  # X-axis has gravity (rotated)
            linear_x = 0  # This axis has gravity
            linear_y = delta_y  # Horizontal movement
        else:  # Y-axis has gravity (rotated)
            linear_x = delta_x  # Horizontal movement
            linear_y = 0  # This axis has gravity
        
        # Determine movement direction based on linear acceleration
        abs_x = abs(linear_x)
        abs_y = abs(linear_y)
        
        # Check if movement is significant (above threshold)
        if abs_x < self.movement_threshold and abs_y < self.movement_threshold:
            return "not moving"
        
        # Determine primary direction (whichever has larger magnitude)
        if abs_x > abs_y:
            # Movement primarily in X direction (forward/backward)
            if linear_x > self.movement_threshold:
                return "forward"
            elif linear_x < -self.movement_threshold:
                return "backward"
        else:
            # Movement primarily in Y direction (left/right)
            if linear_y > self.movement_threshold:
                return "rightward"
            elif linear_y < -self.movement_threshold:
                return "leftward"
        
        return "not moving"
    
    def print_data(self, data: IMUData):
        """Print IMU data to console with movement direction or raw values."""
        elapsed = data.timestamp - self.start_time
        actual_rate = self.sample_count / elapsed if elapsed > 0 else 0
        
        if self.show_raw:
            # Show raw acceleration and gyro values
            print(f"\r[{elapsed:6.2f}s] Sample: {self.sample_count:5d} | "
                  f"Rate: {actual_rate:5.1f} Hz | "
                  f"Accel: [{data.accel_x:6.2f}, {data.accel_y:6.2f}, {data.accel_z:6.2f}] m/s² | "
                  f"Gyro: [{data.gyro_x:6.2f}, {data.gyro_y:6.2f}, {data.gyro_z:6.2f}] deg/s | "
                  f"Temp: {data.temperature:5.1f}C", end='', flush=True)
            
            # Print magnetometer if available
            if data.mag_x != 0.0 or data.mag_y != 0.0 or data.mag_z != 0.0:
                print(f" | Mag: [{data.mag_x:6.1f}, {data.mag_y:6.1f}, {data.mag_z:6.1f}] uT", end='')
        else:
            # Show movement direction
            movement = self.detect_movement(data)
            
            # Color codes for movement (optional, works on most terminals)
            reset_color = "\033[0m"
            color = ""
            if movement.startswith("calibrating"):
                color = "\033[90m"  # Gray for calibration
            elif movement == "not moving":
                color = ""
            elif movement == "forward":
                color = "\033[92m"  # Green
            elif movement == "backward":
                color = "\033[91m"  # Red
            elif movement == "leftward":
                color = "\033[93m"  # Yellow
            elif movement == "rightward":
                color = "\033[94m"  # Blue
            elif movement == "calculating...":
                color = "\033[90m"  # Gray
            
            # Format movement string (handle variable length for calibration progress)
            movement_display = movement
            if len(movement) > 20:  # Truncate if too long
                movement_display = movement[:17] + "..."
            
            print(f"\r[{elapsed:6.2f}s] Sample: {self.sample_count:5d} | "
                  f"Rate: {actual_rate:5.1f} Hz | "
                  f"Movement: {color}{movement_display:<20s}{reset_color} | "
                  f"Temp: {data.temperature:5.1f}C", end='', flush=True)
            
            # Optionally show gyro for rotation detection
            gyro_magnitude = (data.gyro_x**2 + data.gyro_y**2 + data.gyro_z**2)**0.5
            if gyro_magnitude > 5.0:  # Significant rotation
                print(f" | Rotating: {gyro_magnitude:5.1f} deg/s", end='')
    
    def run(self):
        """Main loop for reading and displaying IMU data."""
        if not self.initialize():
            print("Failed to initialize IMU")
            return
        
        print("\n" + "="*80)
        print("IMU Testing Toolkit - Starting data acquisition")
        print("="*80)
        if not self.show_raw:
            sensitivity_info = f" (sensitivity: {self.history_size}-sample, {self.base_samples}-sample baseline)"
            print(f"Movement detection: ON (threshold: {self.movement_threshold} m/s²{sensitivity_info})")
            print("Displaying: not moving, forward, backward, leftward, rightward")
        else:
            print("Displaying: Raw acceleration and gyroscope values")
        print("Press Ctrl+C to stop\n")
        
        dt = 1.0 / self.sample_rate
        next_sample_time = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                
                # Check duration
                if self.duration and (current_time - self.start_time) >= self.duration:
                    break
                
                # Read data
                if self.simulate:
                    data = self.simulator.generate_data()
                    data.timestamp = current_time
                else:
                    data = self.driver.read_data()
                    if data is None:
                        time.sleep(0.01)
                        continue
                
                self.sample_count += 1
                
                # Print data
                if not self.plot:
                    self.print_data(data)
                
                # Update plot
                if self.plot:
                    self._update_plot(data)
                
                # Sleep to maintain sample rate
                next_sample_time += dt
                sleep_time = next_sample_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Falling behind, reset timing
                    next_sample_time = time.time()
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        if self.driver:
            self.driver.close()
        
        if self.plot and HAS_MATPLOTLIB:
            plt.ioff()
            plt.close()
        
        elapsed = time.time() - self.start_time
        actual_rate = self.sample_count / elapsed if elapsed > 0 else 0
        
        print(f"\n\n" + "="*80)
        print(f"Session Summary:")
        print(f"  Duration: {elapsed:.2f} seconds")
        print(f"  Samples: {self.sample_count}")
        print(f"  Average rate: {actual_rate:.2f} Hz")
        print(f"  Target rate: {self.sample_rate:.2f} Hz")
        print("="*80)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='IMU Testing Toolkit for Raspberry Pi',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          # Auto-detect and show movement direction
  %(prog)s --simulate               # Run in simulation mode
  %(prog)s --raw                    # Show raw acceleration/gyro values
  %(prog)s --threshold 0.3          # Lower movement detection threshold
  %(prog)s --sensitivity high       # Fast response (may be noisy)
  %(prog)s --sensitivity low        # Slower but more stable
  %(prog)s --rate 100 --plot        # Run at 100 Hz with live plot
  %(prog)s --simulate --duration 10 # Simulate for 10 seconds
        """
    )
    
    parser.add_argument(
        '--simulate',
        action='store_true',
        help='Run in simulation mode (default: auto-detect)'
    )
    
    parser.add_argument(
        '--rate',
        type=float,
        default=50.0,
        help='Sample rate in Hz (default: 50)'
    )
    
    parser.add_argument(
        '--plot',
        action='store_true',
        help='Enable live plotting (requires matplotlib)'
    )
    
    parser.add_argument(
        '--duration',
        type=float,
        default=None,
        help='Run duration in seconds (default: infinite until Ctrl+C)'
    )
    
    parser.add_argument(
        '--raw',
        action='store_true',
        help='Show raw acceleration/gyro values instead of movement direction'
    )
    
    parser.add_argument(
        '--threshold',
        type=float,
        default=0.5,
        help='Movement detection threshold in m/s² (default: 0.5)'
    )
    
    parser.add_argument(
        '--sensitivity',
        type=str,
        choices=['low', 'medium', 'high'],
        default='medium',
        help='Movement detection sensitivity: low (slower but stable), medium (balanced), high (fast but may be noisy) (default: medium)'
    )
    
    args = parser.parse_args()
    
    # Validate arguments
    if args.rate <= 0:
        print("Error: Sample rate must be positive")
        sys.exit(1)
    
    if args.duration and args.duration <= 0:
        print("Error: Duration must be positive")
        sys.exit(1)
    
    # Create and run tester
    tester = IMUTester(
        simulate=args.simulate,
        sample_rate=args.rate,
        plot=args.plot,
        duration=args.duration,
        show_raw=args.raw,
        movement_threshold=args.threshold,
        sensitivity=args.sensitivity
    )
    
    tester.run()


if __name__ == '__main__':
    main()

