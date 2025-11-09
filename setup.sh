#!/bin/bash
# Setup script for IMU Testing Toolkit on Raspberry Pi
# This script automates the installation process

set -e  # Exit on error

echo "=========================================="
echo "IMU Testing Toolkit - Setup Script"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ] || ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "Warning: This script is designed for Raspberry Pi"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check Python version
echo "Checking Python installation..."
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 is not installed"
    exit 1
fi

PYTHON_VERSION=$(python3 --version)
echo "Found: $PYTHON_VERSION"
echo ""

# Update system packages
echo "Updating system packages..."
sudo apt update
echo ""

# Install required system packages
echo "Installing system dependencies..."
sudo apt install -y python3-venv python3-full python3-pip i2c-tools
echo ""

# Check I2C
echo "Checking I2C configuration..."
if [ ! -c /dev/i2c-1 ]; then
    echo "Warning: I2C device not found. Enable I2C with: sudo raspi-config"
    echo "Navigate to: Interface Options → I2C → Enable"
    read -p "Continue setup anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "I2C device found: /dev/i2c-1"
fi

# Check I2C permissions
if ! groups | grep -q i2c; then
    echo "Adding user to i2c group..."
    sudo usermod -a -G i2c $USER
    echo "User added to i2c group. You may need to log out and back in for changes to take effect."
else
    echo "User is already in i2c group"
fi
echo ""

# Create virtual environment
echo "Creating virtual environment..."
if [ -d "venv" ]; then
    echo "Virtual environment already exists. Removing old one..."
    rm -rf venv
fi

python3 -m venv venv
echo "Virtual environment created"
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate
echo ""

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip
echo ""

# Install requirements
echo "Installing Python packages..."
pip install -r requirements.txt
echo ""

# Test installation
echo "Testing installation..."
if python3 -c "import smbus2" 2>/dev/null; then
    echo "✓ smbus2 installed successfully"
else
    echo "✗ smbus2 installation failed"
fi

if python3 -c "import numpy" 2>/dev/null; then
    echo "✓ numpy installed successfully"
else
    echo "✗ numpy installation failed"
fi

echo ""
echo "=========================================="
echo "Setup completed successfully!"
echo "=========================================="
echo ""
echo "To use the IMU toolkit:"
echo "  1. Activate virtual environment: source venv/bin/activate"
echo "  2. Run the script: python3 imu_test.py"
echo ""
echo "For simulation mode (no hardware):"
echo "  python3 imu_test.py --simulate"
echo ""
echo "Note: If you were added to the i2c group, you may need to"
echo "      log out and log back in for the changes to take effect."
echo ""

