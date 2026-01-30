# YAHBOOM G1 Motor Driver + Jetson Orin Nano Integration

> Date: 2026-01-29
> Status: Research Complete
> Keywords: yahboom g1, motor driver, jetson orin nano, gpio, pwm, differential drive, tb6612fng

## Summary

Research findings for connecting the YAHBOOM G1 Smart Robot Kit motor driver to the Jetson Orin Nano GPIO for differential drive control.

---

## YAHBOOM G1 Smart Robot Kit Overview

### Hardware Specifications

| Component | Specification |
|-----------|---------------|
| **Motors** | 370 DC Motors x 2 (high-power) |
| **Drive Type** | Differential drive (tank tracks) |
| **Expansion Board** | BST-4WD Multi-functional |
| **Power Supply** | 18650-3S Battery (11.1V, 2000mAh) |
| **Voltage Regulator** | LM2596s |
| **Robot Size** | 245 x 234 x 178 mm |
| **Weight** | 1138g |

### Expansion Board Features

The BST-4WD expansion board is compatible with:
- Arduino UNO
- 51 Microcontroller
- STM32
- Raspberry Pi

**Motor Control Method**: The expansion board uses **PWM-based control** with:
- Duty cycle > 50% = Forward
- Duty cycle < 50% = Reverse
- Duty cycle = 50% = Stop

**Sources:**
- [YAHBOOM G1 Product Page](https://category.yahboom.net/products/g1tank)
- [YAHBOOM G1 Study Portal](https://www.yahboom.net/study/G1-T-PI)
- [GitHub Repository](https://github.com/YahboomTechnology/Raspberry-pi-G1-Tank)

---

## Motor Driver Chip Options

### Option 1: TB6612FNG (Recommended)

YAHBOOM sells a separate [TB6612FNG Motor Drive Module](https://category.yahboom.net/products/yahboom-tb6612fng-motor-drive-module) which is well-documented and suitable for direct GPIO control.

| Specification | Value |
|--------------|-------|
| **Supply Voltage (VM)** | 2.5V - 13.5V |
| **Logic Voltage (VCC)** | 2.7V - 5.5V |
| **Continuous Current** | 1.2A per channel |
| **Peak Current** | 3.2A per channel |
| **Channels** | 2 (perfect for differential drive) |

### Option 2: AT8236 (YAHBOOM Dual Motor Module)

YAHBOOM also offers an [AT8236 Dual Motor Module](https://category.yahboom.net/products/dual-md-module):
- Rated driving current: 3.6A per motor
- Peak current: 6A per motor
- 5-12V wide voltage input
- Built-in protection circuits

---

## TB6612FNG Pinout and Control Logic

### Pin Definitions

| Pin | Function | Description |
|-----|----------|-------------|
| **VM** | Motor Power | 2.5V - 13.5V |
| **VCC** | Logic Power | 2.7V - 5.5V (use 3.3V from Jetson) |
| **GND** | Ground | Common ground |
| **STBY** | Standby | HIGH to enable, LOW to disable |
| **AIN1** | Motor A Dir 1 | Direction control |
| **AIN2** | Motor A Dir 2 | Direction control |
| **PWMA** | Motor A Speed | PWM signal (0-100%) |
| **AO1** | Motor A Out 1 | Connect to motor |
| **AO2** | Motor A Out 2 | Connect to motor |
| **BIN1** | Motor B Dir 1 | Direction control |
| **BIN2** | Motor B Dir 2 | Direction control |
| **PWMB** | Motor B Speed | PWM signal |
| **BO1** | Motor B Out 1 | Connect to motor |
| **BO2** | Motor B Out 2 | Connect to motor |

### Control Logic Table

| IN1 | IN2 | PWM | Mode |
|-----|-----|-----|------|
| HIGH | LOW | PWM | **Forward** (CW) |
| LOW | HIGH | PWM | **Reverse** (CCW) |
| LOW | LOW | X | **Stop** (coast) |
| HIGH | HIGH | X | **Brake** (short) |

**Source:** [SparkFun TB6612FNG Hookup Guide](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all)

---

## Jetson Orin Nano GPIO Pinout

### 40-Pin Header (J12)

```
              3.3V (1)  (2)  5V
   I2C1_SDA/GPIO (3)  (4)  5V
   I2C1_SCL/GPIO (5)  (6)  GND
         GPIO09 (7)  (8)  UART1_TX
             GND (9)  (10) UART1_RX
       UART1_RTS (11) (12) I2S0_SCLK
        SPI1_SCK (13) (14) GND
  GPIO12/PWM (*) (15) (16) SPI1_CS1
            3.3V (17) (18) SPI1_CS0
       SPI1_MOSI (19) (20) GND
       SPI1_MISO (21) (22) GPIO13
        SPI1_CLK (23) (24) SPI1_CS0
             GND (25) (26) SPI1_CS1
   I2C0_SDA/GPIO (27) (28) I2C0_SCL
         GPIO01 (29) (30) GND
         GPIO11 (31) (32) GPIO07/PWM
  GPIO13/PWM (*) (33) (34) GND
      I2S0_FS   (35) (36) UART1_CTS
         GPIO27 (37) (38) I2S0_SDIN
             GND (39) (40) I2S0_SDOUT

(*) = Hardware PWM capable
```

### PWM-Capable Pins

| Pin | PWM Chip | Notes |
|-----|----------|-------|
| **Pin 15** | `/sys/devices/3280000.pwm` | Works on Orin Nano |
| **Pin 33** | `/sys/devices/32c0000.pwm` | Most reliable |
| Pin 32 | Limited | May only work as on/off |

**Important:** Pin 33 is the most reliable PWM pin on Jetson Orin Nano.

**Sources:**
- [JetsonHacks GPIO Pinout](https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/)
- [NVIDIA Jetson GPIO Issues](https://github.com/NVIDIA/jetson-gpio/issues/105)

---

## Recommended Wiring: Jetson Orin Nano to TB6612FNG

### Wiring Diagram

```
Jetson Orin Nano                    TB6612FNG
================                    =========

Pin 1  (3.3V)  ------------------>  VCC (Logic)
Pin 2  (5V)    ------[Optional]-->  (if motor needs 5V logic)
Pin 6  (GND)   ------------------>  GND
Pin 11 (GPIO)  ------------------>  STBY (Standby)

Motor A (Left):
Pin 13 (GPIO)  ------------------>  AIN1 (Direction)
Pin 15 (GPIO)  ------------------>  AIN2 (Direction)
Pin 33 (PWM)   ------------------>  PWMA (Speed) *Most reliable PWM*

Motor B (Right):
Pin 16 (GPIO)  ------------------>  BIN1 (Direction)
Pin 18 (GPIO)  ------------------>  BIN2 (Direction)
Pin 32 (PWM)   ------------------>  PWMB (Speed)

External Power:
Battery (+)    ------------------>  VM (Motor Voltage, 7-12V)
Battery (-)    ------------------>  GND (also connect to Jetson GND)

Motor Connections:
                AO1, AO2 --------->  Left Motor
                BO1, BO2 --------->  Right Motor
```

### Suggested Pin Mapping

| Function | Jetson Pin | Board Mode | BCM Mode | Notes |
|----------|------------|------------|----------|-------|
| STBY | 11 | 11 | 17 | Enable driver |
| AIN1 | 13 | 13 | 27 | Left motor dir 1 |
| AIN2 | 15 | 15 | 22 | Left motor dir 2 |
| PWMA | 33 | 33 | 13 | Left motor speed (PWM) |
| BIN1 | 16 | 16 | 23 | Right motor dir 1 |
| BIN2 | 18 | 18 | 24 | Right motor dir 2 |
| PWMB | 32 | 32 | 12 | Right motor speed (PWM) |

---

## Alternative: Using PCA9685 PWM Controller

If the Jetson's hardware PWM is insufficient, use an **I2C PWM controller**:

- **PCA9685**: 16-channel, 12-bit PWM controller
- Connects via I2C (Pins 3 & 5)
- Provides reliable PWM for multiple motors/servos

**Products:**
- [YAHBOOM Motor Driver HAT with PCA9685+TB6612FNG](https://www.amazon.com/Raspberry-Onboard-PCA9685-TB6612FNG-Interface/dp/B07K7NP7C9)

---

## Software Setup

### Install Jetson.GPIO

```bash
# Install the library
sudo pip3 install Jetson.GPIO

# Add user to gpio group
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER

# Copy udev rules
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reboot or re-login
```

### Configure Pinmux (if needed)

For Orin Nano, GPIO pins may need pinmux configuration:

```bash
# Check current pin configuration
sudo cat /sys/kernel/debug/tegra_pinctrl_reg

# Use jetson-io tool to configure
sudo /opt/nvidia/jetson-io/jetson-io.py
```

---

## Python Motor Control Code

### Basic Motor Class

```python
#!/usr/bin/env python3
"""
Motor control for YAHBOOM G1 + Jetson Orin Nano
Uses TB6612FNG motor driver
"""

import Jetson.GPIO as GPIO
import time

class Motor:
    """Control a single DC motor via TB6612FNG."""

    def __init__(self, in1_pin, in2_pin, pwm_pin, pwm_freq=1000):
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.pwm_pin = pwm_pin

        # Setup GPIO
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        # Setup PWM
        self.pwm = GPIO.PWM(self.pwm_pin, pwm_freq)
        self.pwm.start(0)

    def forward(self, speed=100):
        """Run motor forward (0-100 speed)."""
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(min(100, max(0, speed)))

    def reverse(self, speed=100):
        """Run motor in reverse (0-100 speed)."""
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(min(100, max(0, speed)))

    def stop(self):
        """Stop motor (coast)."""
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def brake(self):
        """Brake motor (short)."""
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self):
        """Stop PWM."""
        self.pwm.stop()


class DifferentialDrive:
    """Control a differential drive robot (two motors)."""

    def __init__(self, left_motor, right_motor, stby_pin):
        self.left = left_motor
        self.right = right_motor
        self.stby = stby_pin

        # Setup standby pin
        GPIO.setup(self.stby, GPIO.OUT)
        GPIO.output(self.stby, GPIO.HIGH)  # Enable driver

    def enable(self):
        """Enable motor driver."""
        GPIO.output(self.stby, GPIO.HIGH)

    def disable(self):
        """Disable motor driver (standby)."""
        GPIO.output(self.stby, GPIO.LOW)

    def forward(self, speed=100):
        """Move forward."""
        self.left.forward(speed)
        self.right.forward(speed)

    def reverse(self, speed=100):
        """Move backward."""
        self.left.reverse(speed)
        self.right.reverse(speed)

    def turn_left(self, speed=100):
        """Turn left (pivot)."""
        self.left.reverse(speed)
        self.right.forward(speed)

    def turn_right(self, speed=100):
        """Turn right (pivot)."""
        self.left.forward(speed)
        self.right.reverse(speed)

    def stop(self):
        """Stop both motors."""
        self.left.stop()
        self.right.stop()

    def cleanup(self):
        """Cleanup GPIO."""
        self.disable()
        self.left.cleanup()
        self.right.cleanup()


# Example usage
if __name__ == "__main__":
    # Use BOARD pin numbering
    GPIO.setmode(GPIO.BOARD)

    # Pin definitions (adjust based on your wiring)
    STBY_PIN = 11
    LEFT_IN1 = 13
    LEFT_IN2 = 15
    LEFT_PWM = 33
    RIGHT_IN1 = 16
    RIGHT_IN2 = 18
    RIGHT_PWM = 32

    try:
        # Create motors
        left_motor = Motor(LEFT_IN1, LEFT_IN2, LEFT_PWM)
        right_motor = Motor(RIGHT_IN1, RIGHT_IN2, RIGHT_PWM)
        robot = DifferentialDrive(left_motor, right_motor, STBY_PIN)

        print("Testing motors...")

        # Test sequence
        print("Forward...")
        robot.forward(50)
        time.sleep(2)

        print("Stop...")
        robot.stop()
        time.sleep(1)

        print("Reverse...")
        robot.reverse(50)
        time.sleep(2)

        print("Stop...")
        robot.stop()
        time.sleep(1)

        print("Turn left...")
        robot.turn_left(50)
        time.sleep(1)

        print("Turn right...")
        robot.turn_right(50)
        time.sleep(1)

        print("Stop...")
        robot.stop()

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        robot.cleanup()
        GPIO.cleanup()
        print("Cleanup complete")
```

---

## Troubleshooting

### GPIO Pin Not Working

1. **Check pinmux configuration**:
   ```bash
   sudo /opt/nvidia/jetson-io/jetson-io.py
   ```

2. **Check permissions**:
   ```bash
   groups  # Should include 'gpio'
   ls -la /dev/gpiochip*
   ```

3. **Try different pin** - Pin 33 is most reliable for PWM

### PWM Not Working

- Only **Pin 33** reliably supports PWM on Jetson Orin Nano
- For multiple PWM channels, use **PCA9685 I2C PWM controller**

### Motors Spin at Boot

- The GPIO state is undefined at boot
- Connect a **switch to STBY pin** for manual control
- Or add a delay in your startup script

### Motor Direction Wrong

- Swap IN1 and IN2 connections, or
- Use software offset (swap `forward()` and `reverse()`)

---

## Hardware Checklist

Before connecting:

- [ ] Verify battery voltage matches motor driver VM range (7-12V typical)
- [ ] Connect common ground between Jetson, motor driver, and battery
- [ ] Use 3.3V from Jetson for VCC (logic level)
- [ ] Never connect motor power (VM) to Jetson
- [ ] Add heatsink to motor driver if running high current
- [ ] Consider adding capacitor across motor terminals to reduce noise

---

## Next Steps

1. **Purchase TB6612FNG motor driver** (if not using built-in BST-4WD board)
2. **Wire according to diagram above**
3. **Test with simple script** before integrating with RAG system
4. **Add to bootylicious project** as motor control module

---

## Related

**Project Files:**
- [bootylicious/](../../bootylicious/) - Main Jetson project
- [bootylicious/rag/](../../bootylicious/rag/) - RAG system

**External Resources:**
- [SparkFun TB6612FNG Guide](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all)
- [NVIDIA Jetson.GPIO Library](https://github.com/NVIDIA/jetson-gpio)
- [JetsonHacks GPIO Pinout](https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/)
- [YAHBOOM G1 GitHub](https://github.com/YahboomTechnology/Raspberry-pi-G1-Tank)
- [Motor Driver Bluetin.io Tutorial](https://www.bluetin.io/dc-motors/motor-driver-raspberry-pi-tb6612fng/)

---

*Document created: 2026-01-29*
