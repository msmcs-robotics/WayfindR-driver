"""
Pin Configuration for L298N on Jetson Orin Nano

Uses gpiod (libgpiod v2) chip:line mapping instead of BOARD pin numbers.
Jetson.GPIO is broken on JetPack 6.x (R36.4.4) — GPIO.output() silently
fails to drive pins, while gpioset/gpiod work correctly.

Pin mapping (BOARD → gpiochip:line), confirmed via gpioinfo:

  BOARD 32 (ENA) → gpiochip1, line 9  (PBB.01) — PWM pin
  BOARD 29 (IN1) → gpiochip0, line 144 (PAC.06)
  BOARD 31 (IN2) → gpiochip1, line 15 (PCC.03)
  BOARD 33 (ENB) → gpiochip0, line 105 (PQ.05) — PWM pin
  BOARD  7 (IN3) → gpiochip1, line 12 (PCC.00)
  BOARD 13 (IN4) → gpiochip0, line 43 (PH.00)

Power:
  L298N VCC  -> external 12V supply (NOT from Jetson)
  L298N GND  -> shared ground with Jetson GND (e.g., pin 6, 9, 14, 20, 25, 30, 34, 39)
  L298N 5V   -> leave disconnected (Jetson has its own 5V rail)
"""

# L298N -> Jetson Orin Nano pin mapping (gpiochip:line)
# Each pin is a tuple: (chip_path, line_offset)
JETSON_L298N_CONFIG = {
    'ENA': ('/dev/gpiochip1', 9),    # Motor A enable (BOARD 32, PBB.01)
    'IN1': ('/dev/gpiochip0', 144),  # Motor A direction (BOARD 29, PAC.06)
    'IN2': ('/dev/gpiochip1', 15),   # Motor A direction (BOARD 31, PCC.03)
    'ENB': ('/dev/gpiochip0', 105),  # Motor B enable (BOARD 33, PQ.05)
    'IN3': ('/dev/gpiochip1', 12),   # Motor B direction (BOARD 7, PCC.00)
    'IN4': ('/dev/gpiochip0', 43),   # Motor B direction (BOARD 13, PH.00)
}

# PWM not available via gpiod — ENA/ENB use HIGH/LOW (full speed on/off).
# For variable speed, use sysfs PWM or Jetson's hardware PWM via /sys/class/pwm/.
PWM_FREQ = 1000  # Hz — retained for API compatibility (currently unused)
