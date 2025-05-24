#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
float yaw;

// Motor pins
const int leftMotorForward = 3;
const int leftMotorBackward = 4;
const int rightMotorForward = 5;
const int rightMotorBackward = 6;

void setup() {
  Serial.begin(9600);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  stopMotors();
  Wire.begin();
  mpu.initialize();
}

void loop() {
  mpu.getRotation();
  yaw += mpu.getRotationZ() / 131.0 * 0.01; // approximate yaw in degrees
  Serial.println(yaw);
  delay(10);
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "F") moveForward();
    else if (cmd == "B") moveBackward();
    else if (cmd == "S") stopMotors();
    else if (cmd.startsWith("L")) {
      int deg = cmd.substring(1).toInt();
      turnLeft(deg);
    } else if (cmd.startsWith("R")) {
      int deg = cmd.substring(1).toInt();
      turnRight(deg);
    }
  }
}

void moveForward() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void moveBackward() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void stopMotors() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void turnLeft(int degrees) {
  // Turn left in place: left wheels backward, right wheels forward
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  
  // Simple timing control for degrees (tune experimentally)
  int turnTime = degrees * 10; // ms per degree approx
  delay(turnTime);
  stopMotors();
}

void turnRight(int degrees) {
  // Turn right in place: left wheels forward, right wheels backward
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);

  int turnTime = degrees * 10; // ms per degree approx
  delay(turnTime);
  stopMotors();
}