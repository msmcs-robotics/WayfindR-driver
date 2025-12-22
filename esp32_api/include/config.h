#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===========================================
// Dual-Core Configuration
// ===========================================
// Core 0: Control loop (motors, sensors, flight controller)
// Core 1: Web server, WiFi, user interface
#define CONTROL_CORE 0
#define WEBSERVER_CORE 1

// Control loop frequency (Hz)
// Flight controllers typically need 100-400Hz
// Ground vehicles can use 50-100Hz
#define CONTROL_LOOP_FREQ 100  // 100Hz = 10ms per loop

// ===========================================
// Vehicle Type
// ===========================================
// Uncomment ONE vehicle type:
#define VEHICLE_TYPE_CAR        // 4-wheel skid steer
// #define VEHICLE_TYPE_BOAT    // 2 motors (left/right)
// #define VEHICLE_TYPE_PLANE   // 1 motor + servos (throttle, aileron, elevator, rudder)
// #define VEHICLE_TYPE_QUAD    // 4 motors (requires flight controller on Core 0)
// #define VEHICLE_TYPE_CUSTOM  // Custom configuration

// ===========================================
// WiFi Configuration
// ===========================================
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"

// Access Point mode (fallback if WiFi fails)
#define AP_SSID "WayfindR-Robot"
#define AP_PASSWORD "wayfinder123"

// ===========================================
// Output Channel Configuration
// ===========================================
// Maximum 8 output channels (PWM motors, servos, ESCs)
#define MAX_CHANNELS 8

// Channel types
enum ChannelType {
    CHANNEL_DISABLED = 0,
    CHANNEL_PWM_MOTOR,      // DC motor with PWM (0-255)
    CHANNEL_PWM_MOTOR_DIR,  // DC motor with PWM + direction pin
    CHANNEL_SERVO,          // Servo (0-180 degrees)
    CHANNEL_ESC,            // ESC (1000-2000us PWM)
    CHANNEL_BRUSHLESS       // Brushless motor via ESC
};

// Channel configuration structure
struct ChannelConfig {
    bool enabled;
    ChannelType type;
    uint8_t pwmPin;
    uint8_t dirPin1;        // For PWM_MOTOR_DIR type
    uint8_t dirPin2;        // For PWM_MOTOR_DIR type (optional, for H-bridge)
    bool inverted;          // Invert direction
    uint16_t minValue;      // Min output value
    uint16_t maxValue;      // Max output value
    uint16_t centerValue;   // Center/neutral value (for servos/ESCs)
};

// ===========================================
// Default Pin Configuration (4-wheel car)
// ===========================================
#ifdef VEHICLE_TYPE_CAR

// Left Front Motor (Channel 0)
#define CH0_ENABLED true
#define CH0_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH0_PWM_PIN 25
#define CH0_DIR1_PIN 26
#define CH0_DIR2_PIN 27
#define CH0_INVERTED false

// Left Rear Motor (Channel 1)
#define CH1_ENABLED true
#define CH1_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH1_PWM_PIN 14
#define CH1_DIR1_PIN 12
#define CH1_DIR2_PIN 13
#define CH1_INVERTED false

// Right Front Motor (Channel 2)
#define CH2_ENABLED true
#define CH2_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH2_PWM_PIN 32
#define CH2_DIR1_PIN 33
#define CH2_DIR2_PIN 15
#define CH2_INVERTED false

// Right Rear Motor (Channel 3)
#define CH3_ENABLED true
#define CH3_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH3_PWM_PIN 4
#define CH3_DIR1_PIN 16
#define CH3_DIR2_PIN 17
#define CH3_INVERTED false

// Channels 4-7 disabled for car
#define CH4_ENABLED false
#define CH5_ENABLED false
#define CH6_ENABLED false
#define CH7_ENABLED false

#endif // VEHICLE_TYPE_CAR

// ===========================================
// Boat Configuration (2 motors)
// ===========================================
#ifdef VEHICLE_TYPE_BOAT

#define CH0_ENABLED true
#define CH0_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH0_PWM_PIN 25
#define CH0_DIR1_PIN 26
#define CH0_DIR2_PIN 27
#define CH0_INVERTED false

#define CH1_ENABLED true
#define CH1_TYPE CHANNEL_PWM_MOTOR_DIR
#define CH1_PWM_PIN 32
#define CH1_DIR1_PIN 33
#define CH1_DIR2_PIN 15
#define CH1_INVERTED false

#define CH2_ENABLED false
#define CH3_ENABLED false
#define CH4_ENABLED false
#define CH5_ENABLED false
#define CH6_ENABLED false
#define CH7_ENABLED false

#endif // VEHICLE_TYPE_BOAT

// ===========================================
// Plane Configuration (1 motor + 3 servos)
// ===========================================
#ifdef VEHICLE_TYPE_PLANE

// Throttle (ESC)
#define CH0_ENABLED true
#define CH0_TYPE CHANNEL_ESC
#define CH0_PWM_PIN 25
#define CH0_DIR1_PIN 0
#define CH0_DIR2_PIN 0
#define CH0_INVERTED false

// Aileron Servo
#define CH1_ENABLED true
#define CH1_TYPE CHANNEL_SERVO
#define CH1_PWM_PIN 26
#define CH1_DIR1_PIN 0
#define CH1_DIR2_PIN 0
#define CH1_INVERTED false

// Elevator Servo
#define CH2_ENABLED true
#define CH2_TYPE CHANNEL_SERVO
#define CH2_PWM_PIN 27
#define CH2_DIR1_PIN 0
#define CH2_DIR2_PIN 0
#define CH2_INVERTED false

// Rudder Servo
#define CH3_ENABLED true
#define CH3_TYPE CHANNEL_SERVO
#define CH3_PWM_PIN 14
#define CH3_DIR1_PIN 0
#define CH3_DIR2_PIN 0
#define CH3_INVERTED false

#define CH4_ENABLED false
#define CH5_ENABLED false
#define CH6_ENABLED false
#define CH7_ENABLED false

#endif // VEHICLE_TYPE_PLANE

// ===========================================
// Quadcopter Configuration (4 ESCs)
// ===========================================
#ifdef VEHICLE_TYPE_QUAD

// Motor 1 (Front Left)
#define CH0_ENABLED true
#define CH0_TYPE CHANNEL_ESC
#define CH0_PWM_PIN 25
#define CH0_DIR1_PIN 0
#define CH0_DIR2_PIN 0
#define CH0_INVERTED false

// Motor 2 (Front Right)
#define CH1_ENABLED true
#define CH1_TYPE CHANNEL_ESC
#define CH1_PWM_PIN 26
#define CH1_DIR1_PIN 0
#define CH1_DIR2_PIN 0
#define CH1_INVERTED false

// Motor 3 (Rear Left)
#define CH2_ENABLED true
#define CH2_TYPE CHANNEL_ESC
#define CH2_PWM_PIN 27
#define CH2_DIR1_PIN 0
#define CH2_DIR2_PIN 0
#define CH2_INVERTED false

// Motor 4 (Rear Right)
#define CH3_ENABLED true
#define CH3_TYPE CHANNEL_ESC
#define CH3_PWM_PIN 14
#define CH3_DIR1_PIN 0
#define CH3_DIR2_PIN 0
#define CH3_INVERTED false

#define CH4_ENABLED false
#define CH5_ENABLED false
#define CH6_ENABLED false
#define CH7_ENABLED false

#endif // VEHICLE_TYPE_QUAD

// ===========================================
// Custom Configuration
// ===========================================
#ifdef VEHICLE_TYPE_CUSTOM

// Define your own channel configuration here
#define CH0_ENABLED false
#define CH1_ENABLED false
#define CH2_ENABLED false
#define CH3_ENABLED false
#define CH4_ENABLED false
#define CH5_ENABLED false
#define CH6_ENABLED false
#define CH7_ENABLED false

#endif // VEHICLE_TYPE_CUSTOM

// ===========================================
// PWM Configuration
// ===========================================
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8  // 0-255
#define SERVO_FREQ 50     // 50Hz for servos
#define ESC_FREQ 50       // 50Hz for ESCs (can be higher for some ESCs)

// ===========================================
// Control Parameters
// ===========================================
#define DEADZONE 50           // Deadzone around center (1500)
#define STOP_THRESHOLD 30     // Minimum speed to move motors
#define MAX_SPEED 255         // Maximum motor speed
#define TURN_SENSITIVITY 0.8f // Turning sensitivity

// ===========================================
// Safety
// ===========================================
#define WATCHDOG_TIMEOUT_MS 500  // Stop if no commands for 500ms
#define FAILSAFE_ENABLED true    // Enable failsafe on signal loss
#define ARM_REQUIRED false       // Require arming before motors spin

// ===========================================
// Telemetry
// ===========================================
#define TELEMETRY_RATE_HZ 10  // WebSocket telemetry rate

// ===========================================
// Flight Controller Integration
// ===========================================
// Set to true if running flight controller code on Core 0
#define EXTERNAL_FLIGHT_CONTROLLER false

// If using external flight controller, it should call:
// void setChannelOutput(uint8_t channel, int16_t value);
// Where value is -1000 to 1000 for motors, or microseconds for servos/ESCs

#endif // CONFIG_H
