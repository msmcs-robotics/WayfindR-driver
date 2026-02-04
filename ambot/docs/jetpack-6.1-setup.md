# JetPack 6.1 Setup for Jetson Orin Nano

Preferred software configuration for the Jetson Orin Nano in the Ambot project.

## Overview

**JetPack 6.1** is the recommended and tested version for the Jetson Orin Nano. This version has shown the most stability and compatibility with our robotics stack.

## JetPack 6.1 Specifications

| Component | Version |
|-----------|---------|
| JetPack | 6.1 |
| L4T (Linux for Tegra) | R36.4 |
| CUDA | 12.6 |
| cuDNN | 9.3 |
| TensorRT | 10.3 |
| Ubuntu | 22.04 LTS |
| Python | 3.10 |

## Why JetPack 6.1?

1. **Stability** - Most tested version with our hardware configuration
2. **CUDA 12.6** - Good balance of features and compatibility
3. **Ubuntu 22.04** - LTS release with long-term support
4. **ROS2 Humble compatibility** - Works well with ROS2 Humble Hawksbill

## Installation

### Method 1: SDK Manager (Recommended)

1. Download NVIDIA SDK Manager on a Ubuntu host PC
2. Connect Jetson Orin Nano via USB-C (recovery mode)
3. Select:
   - Target Hardware: Jetson Orin Nano
   - JetPack Version: **6.1**
4. Flash and install

### Method 2: SD Card Image

1. Download JetPack 6.1 SD card image from NVIDIA
2. Flash to microSD card using balenaEtcher or `dd`
3. Insert into Jetson and boot

## Post-Installation Setup

### 1. Update System

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install GPIO Library

```bash
sudo pip3 install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
```

### 3. Configure GPIO Permissions

```bash
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. Reboot

```bash
sudo reboot
```

## Verifying Installation

```bash
# Check JetPack version
cat /etc/nv_tegra_release

# Check CUDA version
nvcc --version

# Test GPIO access
python3 -c "import Jetson.GPIO as GPIO; print('GPIO OK')"
```

## Known Working Configuration

This JetPack version has been tested with:
- TB6612FNG motor driver
- FAGM25-370 DC motors
- YDLIDAR X4 (USB)
- USB webcams
- I2C devices (MPU6050)

## Troubleshooting

### GPIO Permission Denied

```bash
# Ensure user is in gpio group
groups $USER | grep gpio

# If not, add and relogin
sudo usermod -a -G gpio $USER
# Log out and back in
```

### CUDA Not Found

```bash
# Add to ~/.bashrc
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
source ~/.bashrc
```

## References

- [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [JetPack 6.1 Release Notes](https://developer.nvidia.com/embedded/jetpack-sdk-61)
