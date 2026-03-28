"""
Pin Configuration for L298N on Jetson Orin Nano

Two backends supported:
  1. Jetson.GPIO (preferred) — uses BOARD pin numbers, works on original DTB
  2. gpiod / libgpiod v2 (fallback) — uses gpiochip0:line mapping

Pin mapping derived from Jetson.GPIO gpio_pin_data.py JETSON_ORIN_NX_PIN_DEFS:

  BOARD  7 (IN3) -> gpiochip0, line 144 (PAC.06)
  BOARD 13 (IN4) -> gpiochip0, line 122 (PY.00)
  BOARD 29 (IN1) -> gpiochip0, line 105 (PQ.05)
  BOARD 31 (IN2) -> gpiochip0, line 106 (PQ.06)
  BOARD 32 (ENA) -> gpiochip0, line 41  (PG.06) — PWM pin
  BOARD 33 (ENB) -> gpiochip0, line 43  (PH.00) — PWM pin

Power:
  L298N VCC  -> external 12V supply (NOT from Jetson)
  L298N GND  -> shared ground with Jetson GND (e.g., pin 6, 9, 14, 20, 25, 30, 34, 39)
  L298N 5V   -> leave disconnected (Jetson has its own 5V rail)
"""

# BOARD pin numbers for Jetson.GPIO (preferred backend)
JETSON_L298N_BOARD_PINS = {
    'ENA': 32,   # Motor A enable (PG.06, PWM)
    'IN1': 29,   # Motor A direction (PQ.05)
    'IN2': 31,   # Motor A direction (PQ.06)
    'ENB': 33,   # Motor B enable (PH.00, PWM)
    'IN3': 7,    # Motor B direction (PAC.06)
    'IN4': 13,   # Motor B direction (PY.00)
}

# gpiod chip:line mapping (fallback backend)
# All 40-pin header pins are on gpiochip0 (tegra234-gpio)
JETSON_L298N_GPIOD = {
    'ENA': ('/dev/gpiochip0', 41),   # BOARD 32, PG.06
    'IN1': ('/dev/gpiochip0', 105),  # BOARD 29, PQ.05
    'IN2': ('/dev/gpiochip0', 106),  # BOARD 31, PQ.06
    'ENB': ('/dev/gpiochip0', 43),   # BOARD 33, PH.00
    'IN3': ('/dev/gpiochip0', 144),  # BOARD  7, PAC.06
    'IN4': ('/dev/gpiochip0', 122),  # BOARD 13, PY.00
}

# Legacy alias — points to gpiod config for backward compatibility
JETSON_L298N_CONFIG = JETSON_L298N_GPIOD

# PWM frequency — used by Jetson.GPIO backend for hardware PWM on ENA/ENB.
# gpiod backend ignores this (ENA/ENB are HIGH/LOW only).
PWM_FREQ = 1000  # Hz
