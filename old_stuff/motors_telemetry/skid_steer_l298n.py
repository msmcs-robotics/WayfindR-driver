# Basic 4-wheeled Skid Steer Robot Control using L298N Motor Drivers with Raspberry Pi
# RPI GPIO Python Library only works on Raspiban OS

'''
L298N Left Side:
 - IN1: GPIO 17 (BCM)
 - IN2: GPIO 27 (BCM)
 - ENA (enable): GPIO 22 (PWM optional)

L298N Right Side:
 - IN3: GPIO 23 (BCM)
 - IN4: GPIO 24 (BCM)
 - ENB (enable): GPIO 25 (PWM optional

Power:
 - L298N VCC to external 12V supply (not Pi!)
 - GND to battery ground and Pi GND
 - 5V jumper on L298N if you want to power logic (else provide 5V separately)

Motors:
 - Left L298N → Front Left and Rear Left motors
 - Right L298N → Front Right and Rear Right motors
'''


'''
sudo apt update
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
'''


import pigpio
import time

# GPIO Pins
LEFT_IN1 = 17
LEFT_IN2 = 27
LEFT_EN = 22

RIGHT_IN1 = 23
RIGHT_IN2 = 24
RIGHT_EN = 25

class SkidSteerRobot:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Cannot connect to pigpio daemon!")

        # Set all motor pins as outputs
        for pin in [LEFT_IN1, LEFT_IN2, LEFT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_EN]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        self.speed = 255  # Full speed (0-255)

    def set_motor(self, in1, in2, en, forward):
        if forward:
            self.pi.write(in1, 1)
            self.pi.write(in2, 0)
        else:
            self.pi.write(in1, 0)
            self.pi.write(in2, 1)
        self.pi.set_PWM_dutycycle(en, self.speed)

    def stop(self):
        for pin in [LEFT_EN, RIGHT_EN]:
            self.pi.set_PWM_dutycycle(pin, 0)
        for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]:
            self.pi.write(pin, 0)

    def forward(self):
        self.set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, True)
        self.set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, True)

    def backward(self):
        self.set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, False)
        self.set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, False)

    def turn_left(self):  # Right motors move forward
        self.pi.set_PWM_dutycycle(LEFT_EN, 0)
        self.set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, True)

    def turn_right(self):  # Left motors move forward
        self.set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, True)
        self.pi.set_PWM_dutycycle(RIGHT_EN, 0)

    def rotate_left(self):  # 0° Turn
        self.set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, False)
        self.set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, True)

    def rotate_right(self):  # 0° Turn
        self.set_motor(LEFT_IN1, LEFT_IN2, LEFT_EN, True)
        self.set_motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, False)

    def cleanup(self):
        self.stop()
        self.pi.stop()

# Example Usage
if __name__ == "__main__":
    robot = SkidSteerRobot()
    try:
        robot.forward()
        time.sleep(2)
        robot.rotate_left()
        time.sleep(1.5)
        robot.backward()
        time.sleep(2)
        robot.turn_right()
        time.sleep(1)
        robot.stop()
    finally:
        robot.cleanup()
