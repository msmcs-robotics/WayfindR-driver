#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <Arduino.h>
#include "config.h"

/**
 * ControlLoop - Runs on Core 0 at fixed frequency
 *
 * Handles:
 * - Mixing throttle/steering to motor outputs
 * - Vehicle-specific control logic
 * - Watchdog safety
 * - Optional: flight controller integration
 *
 * For custom flight controllers:
 * 1. Set EXTERNAL_FLIGHT_CONTROLLER to true in config.h
 * 2. Implement your control logic in userControlLoop()
 * 3. Call outputChannels.setChannel() to control motors
 */

// Control input state (set by web server on Core 1)
struct ControlInput {
    float throttle;      // -1.0 to 1.0
    float steering;      // -1.0 to 1.0 (yaw for aircraft)
    float pitch;         // -1.0 to 1.0 (aircraft only)
    float roll;          // -1.0 to 1.0 (aircraft only)
    bool armed;
    unsigned long timestamp;
};

// Shared control state (thread-safe access)
extern volatile ControlInput controlInput;
extern portMUX_TYPE controlInputMutex;

// Control loop task handle
extern TaskHandle_t controlLoopTask;

// Initialize and start control loop on Core 0
void startControlLoop();

// Stop control loop
void stopControlLoop();

// Set control input (called from web server)
void setControlInput(float throttle, float steering, float pitch = 0, float roll = 0);

// Get current control input
ControlInput getControlInput();

// Arm/disarm motors
void armMotors();
void disarmMotors();
bool areMotorsArmed();

// User-defined control loop (for custom flight controllers)
// Override this in your own code if EXTERNAL_FLIGHT_CONTROLLER is true
void userControlLoop(ControlInput input, unsigned long deltaTime);

#endif // CONTROL_LOOP_H
