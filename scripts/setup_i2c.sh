#!/bin/bash
set -e

echo "Updating package list..."
sudo apt update

echo "Installing I2C tools and Python libraries..."
sudo apt install -y i2c-tools python3-smbus python3-pip

echo "Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

echo "All done! Please reboot to finish setup."
