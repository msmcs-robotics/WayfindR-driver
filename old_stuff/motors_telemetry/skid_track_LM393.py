import pigpio
import time

# ==== Constants ====
# Motor GPIO Pins
LEFT_IN1 = 17
LEFT_IN2 = 27
LEFT_EN = 22

RIGHT_IN1 = 23
RIGHT_IN2 = 24
RIGHT_EN = 25

# Encoder GPIO Pins
ENC_FL = 5
ENC_FR = 6
ENC_RL = 13
ENC_RR = 19

ENCODER_PINS = [ENC_FL, ENC_FR, ENC_RL, ENC_RR]
ENCODER_COUNTS = [0, 0, 0, 0]
ENCODER_CALLBACKS = []

WHEEL_CIRCUMFERENCE_CONSTANT = 13  # cm
PULSES_PER_REVOLUTION = 20
DEFAULT_SPEED = 255

# ==== pigpio Initialization ====
pi = pigpio.pi()
if not pi.connected:
    raise IOError("Cannot connect to pigpio daemon")

# ==== Motor Setup ====
def setup_motors():
    motor_pins = [LEFT_IN1, LEFT_IN2, LEFT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_EN]
    for pin in motor_pins:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)

# ==== Encoder Setup ====
def _make_encoder_callback(index):
    def _callback(gpio, level, tick):
        global ENCODER_COUNTS
        ENCODER_COUNTS[index] += 1
    return _callback

def setup_encoders():
    global ENCODER_CALLBACKS
    for i, pin in enumerate(ENCODER_PINS):
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        cb = pi.callback(pin, pigpio.FALLING_EDGE, _make_encoder_callback(i))
        ENCODER_CALLBACKS.append(cb)

def reset_encoders():
    global ENCODER_COUNTS
    ENCODER_COUNTS = [0, 0, 0, 0]

def get_average_distance_cm():
    total_pulses = sum(ENCODER_COUNTS)
    avg_pulses = total_pulses / 4
    rotations = avg_pulses / PULSES_PER_REVOLUTION
    distance_cm = rotations * WHEEL_CIRCUMFERENCE_CONSTANT
    return distance_cm

# ==== Motor Control ====
def set_motor(in1, in2, en, forward, speed=DEFAULT_SPEED):
    pi.write(in1, 1 if forward else 0)
    pi.write(in2, 0 if forward else 1)
    pi.set_PWM_dutycycle(en, speed)

def forward():
    set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, True)
    set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, True)

def backward():
    set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, False)
    set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, False)

def stop():
    pi.set_PWM_dutycycle(LEFT_EN, 0)
    pi.set_PWM_dutycycle(RIGHT_EN, 0)
    pi.write(LEFT_IN1, 0)
    pi.write(LEFT_IN2, 0)
    pi.write(RIGHT_IN1, 0)
    pi.write(RIGHT_IN2, 0)

def turn_left():
    pi.set_PWM_dutycycle(LEFT_EN, 0)
    set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, True)

def turn_right():
    set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, True)
    pi.set_PWM_dutycycle(RIGHT_EN, 0)

def rotate_left():
    set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, False)
    set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, True)

def rotate_right():
    set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, True)
    set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, False)

# ==== Cleanup ====
def cleanup():
    stop()
    for cb in ENCODER_CALLBACKS:
        cb.cancel()
    pi.stop()

# ==== Example Usage ====
if __name__ == "__main__":
    try:
        setup_motors()
        setup_encoders()

        reset_encoders()
        forward()
        time.sleep(3)
        stop()

        distance = get_average_distance_cm()
        print(f"Moved approximately {distance:.2f} cm")

    finally:
        cleanup()
