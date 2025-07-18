/*
-- binding --

- bind key on B/VCC
- connect battery 4-8V on any other pins
- red on top, data/other color in middle, BLACK ON BOTTOM


-- PWM receiver output --

- bind on model 1
- right joystick lateral - ch1 - pin3
- right joystick vertical - ch2 - pin5
- RX setup - PPM output OFF
- PWM output pins are in second row from top
- just need 5v and GND connected somewhere along the channels
*/

/*
 * Simple FS-i6B Test Sketch
 * Right joystick control test
 * 
 * Hardware Connections:
 * - CH1 signal → Pin 3 (right stick horizontal)
 * - CH2 signal → Pin 5 (right stick vertical)
 * - 5V → Arduino 5V (one connection)
 * - GND → Arduino GND (one connection)
 */

// Pin definitions
const int CH1_PIN = 3;  // Right stick horizontal (steering)
const int CH2_PIN = 5;  // Right stick vertical (throttle)
const int LED_PIN = 13; // Built-in LED

// RC values
volatile int ch1_value = 1500;  // Horizontal (1000-2000, center ~1500)
volatile int ch2_value = 1500;  // Vertical (1000-2000, center ~1500)

// Interrupt variables for CH1 (pin 3 has interrupt)
volatile unsigned long ch1_start = 0;
volatile bool ch1_updated = false;

// Timing for CH2 (pin 5 - no interrupt, use pulseIn)
unsigned long lastCH2Read = 0;
bool signalActive = false;
unsigned long lastSignalTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup pins
  pinMode(CH1_PIN, INPUT_PULLUP);
  pinMode(CH2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Attach interrupt for CH1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_interrupt, CHANGE);
  
  Serial.println("=== FS-i6B Right Joystick Test ===");
  Serial.println("CH1 (Pin 3): Right stick horizontal");
  Serial.println("CH2 (Pin 5): Right stick vertical");
  Serial.println("Move right joystick to test");
  Serial.println("===================================");
}

void loop() {
  // Read CH2 using pulseIn (since pin 5 has no interrupt)
  if (millis() - lastCH2Read > 25) {  // Read every 25ms
    int newCH2 = pulseIn(CH2_PIN, HIGH, 25000);  // 25ms timeout
    if (newCH2 > 800 && newCH2 < 2200) {  // Valid range
      ch2_value = newCH2;
      lastSignalTime = millis();
      signalActive = true;
    }
    lastCH2Read = millis();
  }
  
  // Check if CH1 was updated
  if (ch1_updated) {
    lastSignalTime = millis();
    signalActive = true;
    ch1_updated = false;
  }
  
  // Control LED based on signal activity
  if (millis() - lastSignalTime < 200) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
    signalActive = false;
  }
  
  // Print values
  printValues();
  
  delay(100);  // 10Hz update rate
}

// Interrupt for CH1 (pin 3)
void ch1_interrupt() {
  int trigger = digitalRead(CH1_PIN);
  if (trigger == 1) {
    ch1_start = micros();
  } else if (trigger == 0) {
    ch1_value = micros() - ch1_start;
    ch1_updated = true;
  }
}

void printValues() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 200) {  // Print every 200ms
    Serial.print("CH1 (Horizontal): ");
    Serial.print(ch1_value);
    Serial.print(" μs");
    
    Serial.print(" | CH2 (Vertical): ");
    Serial.print(ch2_value);
    Serial.print(" μs");
    
    Serial.print(" | LED: ");
    Serial.print(digitalRead(LED_PIN) ? "ON" : "OFF");
    
    // Show stick position
    Serial.print(" | Stick: ");
    
    // Horizontal
    if (ch1_value > 1600) {
      Serial.print("RIGHT-");
    } else if (ch1_value < 1400) {
      Serial.print("LEFT-");
    } else {
      Serial.print("CENTER-");
    }
    
    // Vertical (flipped: lower values = stick pushed up)
    if (ch2_value > 1600) {
      Serial.print("DOWN");
    } else if (ch2_value < 1400) {
      Serial.print("UP");
    } else {
      Serial.print("CENTER");
    }
    
    Serial.println();
    lastPrint = millis();
  }
}

/*
 * EXPECTED BEHAVIOR:
 * - Right stick center: Both values ~1500, LED ON
 * - Right stick right: CH1 ~2000, LED ON
 * - Right stick left: CH1 ~1000, LED ON  
 * - Right stick up: CH2 ~2000, LED ON
 * - Right stick down: CH2 ~1000, LED ON
 * - Transmitter off: Values may drift, LED OFF
 */