"""
Pin Configuration for L298N on Jetson Orin Nano

BOARD pin numbering — physical header positions, not GPIO names.

L298N wiring summary:
  Motor A (left):  ENA=32 (PWM), IN1=29, IN2=31
  Motor B (right): ENB=33 (PWM), IN3=7,  IN4=13

Pins 32 and 33 are hardware PWM-capable on the Orin Nano 40-pin header.
Direction pins (29, 31, 7, 13) are standard digital GPIO.

Power:
  L298N VCC  -> external 12V supply (NOT from Jetson)
  L298N GND  -> shared ground with Jetson GND (e.g., pin 6, 9, 14, 20, 25, 30, 34, 39)
  L298N 5V   -> leave disconnected (Jetson has its own 5V rail)
"""

# L298N -> Jetson Orin Nano pin mapping (BOARD numbering)
JETSON_L298N_CONFIG = {
    'ENA': 32,   # Motor A PWM (hardware PWM pin)
    'IN1': 29,   # Motor A direction
    'IN2': 31,   # Motor A direction
    'ENB': 33,   # Motor B PWM (hardware PWM pin)
    'IN3': 7,    # Motor B direction
    'IN4': 13,   # Motor B direction
}

PWM_FREQ = 1000  # Hz
