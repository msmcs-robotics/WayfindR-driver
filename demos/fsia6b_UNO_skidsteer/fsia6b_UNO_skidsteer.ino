// the L298N drivers and the uno all need to share the same ground connection.

/*
 * RC Car Differential Drive Controller
 * FS-i6B Receiver with Dual L298N Motor Drivers
 * 
 * Hardware Connections:
 * FS-i6B Receiver:
 * - CH1 → Pin 3 (Steering)
 * - CH2 → Pin 5 (Throttle)
 * - 5V → Arduino 5V
 * - GND → Arduino GND
 * 
 * LEFT L298N Driver (ENA/ENB jumpered):
 * - IN1 → Pin 6 (Left Front PWM)
 * - IN2 → Pin 7 (Left Front Direction)
 * - IN3 → Pin 8 (Left Rear PWM)
 * - IN4 → Pin 9 (Left Rear Direction)
 * 
 * RIGHT L298N Driver (ENA/ENB jumpered):
 * - IN1 → Pin 10 (Right Front PWM)
 * - IN2 → Pin 11 (Right Front Direction)
 * - IN3 → Pin 12 (Right Rear PWM)
 * - IN4 → Pin 13 (Right Rear Direction)
 * 
 * Status LED → Pin A1
 */

// Pin definitions
const int CH1_PIN = 3;   // Steering (Right stick horizontal)
const int CH2_PIN = 5;   // Throttle (Right stick vertical)
const int LED_PIN = A1;  // Status LED

// Left motor driver pins (ENA/ENB are jumpered on your drivers)
const int LEFT_FRONT_PWM = 6;     // IN1 (used as PWM)
const int LEFT_FRONT_DIR = 7;     // IN2 (used as direction)
const int LEFT_REAR_PWM = 8;      // IN3 (used as PWM)
const int LEFT_REAR_DIR = 9;      // IN4 (used as direction)

// Right motor driver pins (ENA/ENB are jumpered on your drivers)
const int RIGHT_FRONT_PWM = 10;   // IN1 (used as PWM)
const int RIGHT_FRONT_DIR = 11;   // IN2 (used as direction)
const int RIGHT_REAR_PWM = 12;    // IN3 (used as PWM)
const int RIGHT_REAR_DIR = 13;    // IN4 (used as direction)

// RC signal variables
volatile int ch1_value = 1500;  // Steering
volatile int ch2_value = 1500;  // Throttle
volatile unsigned long ch1_start = 0;
volatile bool ch1_updated = false;

// Control variables
const int DEADZONE = 50;        // Deadzone around center (1500)
const int STOP_THRESHOLD = 30;  // Minimum speed to actually move motors
const int MAX_SPEED = 255;      // Maximum motor speed
const int MIN_SPEED = 0;        // Minimum motor speed
const float TURN_SENSITIVITY = 0.8; // Turning sensitivity (reduced for better control)

// Timing variables
unsigned long lastCH2Read = 0;
unsigned long lastSignalTime = 0;
bool signalActive = false;

void setup() {
  Serial.begin(9600);
  
  // Setup receiver pins
  pinMode(CH1_PIN, INPUT_PULLUP);
  pinMode(CH2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Setup motor driver pins - using original pin configuration
  pinMode(LEFT_FRONT_PWM, OUTPUT);
  pinMode(LEFT_FRONT_DIR, OUTPUT);
  pinMode(LEFT_REAR_PWM, OUTPUT);
  pinMode(LEFT_REAR_DIR, OUTPUT);
  
  pinMode(RIGHT_FRONT_PWM, OUTPUT);
  pinMode(RIGHT_FRONT_DIR, OUTPUT);
  pinMode(RIGHT_REAR_PWM, OUTPUT);
  pinMode(RIGHT_REAR_DIR, OUTPUT);
  
  // Initialize all motors to stopped
  stopAllMotors();
  
  // Attach interrupt for CH1
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_interrupt, CHANGE);
  
  Serial.println("=== RC Car Differential Drive Controller - TURNING FIXED ===");
  Serial.println("CH1: Steering | CH2: Throttle");
  Serial.println("Ready for RC control!");
  Serial.println("==================================================");
  
  // LED startup sequence
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  // Read CH2 using pulseIn
  if (millis() - lastCH2Read > 25) {
    int newCH2 = pulseIn(CH2_PIN, HIGH, 25000);
    if (newCH2 > 800 && newCH2 < 2200) {
      ch2_value = newCH2;
      lastSignalTime = millis();
      signalActive = true;
    }
    lastCH2Read = millis();
  }
  
  // Check CH1 updates
  if (ch1_updated) {
    lastSignalTime = millis();
    signalActive = true;
    ch1_updated = false;
  }
  
  // Control LED and motors based on signal
  if (millis() - lastSignalTime < 200) {
    digitalWrite(LED_PIN, HIGH);
    processRCInput();
  } else {
    digitalWrite(LED_PIN, LOW);
    stopAllMotors();
    signalActive = false;
  }
  
  // Debug output
  printDebugInfo();
  
  delay(20);  // 50Hz update rate
}

void processRCInput() {
  // Map RC values to motor control values
  // CH1 (steering): 1000-2000 → -500 to +500
  // CH2 (throttle): 1000-2000 → -500 to +500
  
  int steering = map(ch1_value, 1000, 2000, -500, 500);
  int throttle = map(ch2_value, 1000, 2000, -500, 500);
  
  // Apply deadzone
  if (abs(steering) < DEADZONE) steering = 0;
  if (abs(throttle) < DEADZONE) throttle = 0;
  
  // Calculate differential drive with FIXED turning logic
  float leftSpeed = throttle;
  float rightSpeed = throttle;
  
  // Apply differential steering with SWAPPED logic (joystick right = turn right)
  if (steering != 0) {
    float steerAmount = abs(steering) * TURN_SENSITIVITY;
    
    if (steering > 0) {
      // Joystick RIGHT = Turn LEFT - LEFT motors should slow down/reverse, RIGHT motors continue
      if (abs(throttle) < 100) {
        // Pivot turn when throttle is low
        leftSpeed = -steerAmount;   // Left motors reverse
        rightSpeed = steerAmount;   // Right motors forward
      } else {
        // Normal turn - reduce left motor speed, right stays same
        rightSpeed = throttle;      // Right motors keep their speed
        leftSpeed = throttle - steerAmount; // Left motors slow down
        // Keep left speed from going negative unless we want to pivot
        if (throttle > 0 && leftSpeed < 0) leftSpeed = 0;
        if (throttle < 0 && leftSpeed > 0) leftSpeed = 0;
      }
    } else {
      // Joystick LEFT = Turn RIGHT - RIGHT motors should slow down/reverse, LEFT motors continue
      if (abs(throttle) < 100) {
        // Pivot turn when throttle is low 
        rightSpeed = -steerAmount;  // Right motors reverse
        leftSpeed = steerAmount;    // Left motors forward
      } else {
        // Normal turn - reduce right motor speed, left stays same
        leftSpeed = throttle;       // Left motors keep their speed
        rightSpeed = throttle - steerAmount;  // Right motors slow down
        // Keep right speed from going negative unless we want to pivot
        if (throttle > 0 && rightSpeed < 0) rightSpeed = 0;
        if (throttle < 0 && rightSpeed > 0) rightSpeed = 0;
      }
    }
  }
  
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -500, 500);
  rightSpeed = constrain(rightSpeed, -500, 500);
  
  // Control motors with corrected logic
  controlLeftMotors((int)leftSpeed);
  controlRightMotors((int)rightSpeed);
}

void controlLeftMotors(int speed) {
  if (abs(speed) < STOP_THRESHOLD) {
    // Stop left motors completely
    analogWrite(LEFT_FRONT_PWM, 0);
    analogWrite(LEFT_REAR_PWM, 0);
    digitalWrite(LEFT_FRONT_DIR, LOW);
    digitalWrite(LEFT_REAR_DIR, LOW);
  } else {
    // Convert speed to PWM value
    int pwmValue = map(abs(speed), 0, 500, 50, MAX_SPEED);
    
    if (speed > 0) {
      // Forward direction - FLIPPED because left motors were going backward when they should go forward
      digitalWrite(LEFT_FRONT_DIR, LOW);   // Flipped from HIGH
      digitalWrite(LEFT_REAR_DIR, HIGH);   // Flipped from LOW
    } else {
      // Reverse direction - FLIPPED because left motors were going forward when they should go backward
      digitalWrite(LEFT_FRONT_DIR, HIGH);  // Flipped from LOW
      digitalWrite(LEFT_REAR_DIR, LOW);    // Flipped from HIGH
    }
    
    // Set PWM speed
    analogWrite(LEFT_FRONT_PWM, pwmValue);
    analogWrite(LEFT_REAR_PWM, pwmValue);
  }
}

void controlRightMotors(int speed) {
  if (abs(speed) < STOP_THRESHOLD) {
    // Stop right motors completely
    analogWrite(RIGHT_FRONT_PWM, 0);
    analogWrite(RIGHT_REAR_PWM, 0);
    digitalWrite(RIGHT_FRONT_DIR, LOW);
    digitalWrite(RIGHT_REAR_DIR, LOW);
  } else {
    // Convert speed to PWM value
    int pwmValue = map(abs(speed), 0, 500, 50, MAX_SPEED);
    
    if (speed > 0) {
      // Forward direction - RIGHT MOTORS WERE CORRECT FOR FORWARD/BACKWARD
      digitalWrite(RIGHT_FRONT_DIR, LOW);  // Keep same - was correct
      digitalWrite(RIGHT_REAR_DIR, LOW);   // Keep same - was correct
    } else {
      // Reverse direction - RIGHT MOTORS WERE CORRECT FOR FORWARD/BACKWARD  
      digitalWrite(RIGHT_FRONT_DIR, HIGH); // Keep same - was correct
      digitalWrite(RIGHT_REAR_DIR, HIGH);  // Keep same - was correct
    }
    
    // Set PWM speed
    analogWrite(RIGHT_FRONT_PWM, pwmValue);
    analogWrite(RIGHT_REAR_PWM, pwmValue);
  }
}

void stopAllMotors() {
  // Set all PWM outputs to 0
  analogWrite(LEFT_FRONT_PWM, 0);
  analogWrite(LEFT_REAR_PWM, 0);
  analogWrite(RIGHT_FRONT_PWM, 0);
  analogWrite(RIGHT_REAR_PWM, 0);
  
  // Set all direction pins to LOW
  digitalWrite(LEFT_FRONT_DIR, LOW);
  digitalWrite(LEFT_REAR_DIR, LOW);
  digitalWrite(RIGHT_FRONT_DIR, LOW);
  digitalWrite(RIGHT_REAR_DIR, LOW);
}

void ch1_interrupt() {
  int trigger = digitalRead(CH1_PIN);
  if (trigger == 1) {
    ch1_start = micros();
  } else if (trigger == 0) {
    ch1_value = micros() - ch1_start;
    ch1_updated = true;
  }
}

void printDebugInfo() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 500) {  // Print every 500ms
    Serial.print("CH1: ");
    Serial.print(ch1_value);
    Serial.print(" | CH2: ");
    Serial.print(ch2_value);
    Serial.print(" | Signal: ");
    Serial.print(signalActive ? "ACTIVE" : "LOST");
    
    // Show control state
    int steering = map(ch1_value, 1000, 2000, -500, 500);
    int throttle = map(ch2_value, 1000, 2000, -500, 500);
    
    if (abs(steering) < DEADZONE) steering = 0;
    if (abs(throttle) < DEADZONE) throttle = 0;
    
    Serial.print(" | Steering: ");
    Serial.print(steering);
    Serial.print(" | Throttle: ");
    Serial.print(throttle);
    
    // Show movement
    if (signalActive) {
      if (throttle > 50) {
        Serial.print(" | FORWARD");
      } else if (throttle < -50) {
        Serial.print(" | REVERSE");
      } else {
        Serial.print(" | STOPPED");
      }
      
      if (steering > 50) {
        Serial.print(" + RIGHT");
      } else if (steering < -50) {
        Serial.print(" + LEFT");
      }
    }
    
    Serial.println();
    lastPrint = millis();
  }
}