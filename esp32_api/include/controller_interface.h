#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <Arduino.h>
#include "config.h"

/**
 * Controller Interface
 *
 * Abstract interface for plugging in external control systems
 * (flight controllers, drive controllers, etc.)
 *
 * To integrate your own controller:
 * 1. Create a class that inherits from ControllerInterface
 * 2. Implement the required methods
 * 3. Register it with setController()
 *
 * Example:
 *   class MyFlightController : public ControllerInterface {
 *       void init() override { ... }
 *       void update(float dt) override { ... }
 *       void getMotorOutputs(int16_t* outputs) override { ... }
 *   };
 *
 *   MyFlightController fc;
 *   setController(&fc);
 */

// Input state from web server / radio
struct ControllerInput {
    // Normalized inputs (-1.0 to 1.0)
    float throttle;     // Vertical (0-1 for aircraft, -1 to 1 for ground)
    float roll;         // Roll axis
    float pitch;        // Pitch axis
    float yaw;          // Yaw axis

    // Auxiliary channels
    float aux[4];       // AUX1-AUX4

    // State flags
    bool armed;
    bool failsafe;

    // Timestamp
    unsigned long timestamp_ms;
};

// Output to motors/servos
struct ControllerOutput {
    // Motor outputs (0-1000 for ESCs, -1000 to 1000 for bidirectional)
    int16_t motors[8];

    // Servo outputs (-1000 to 1000)
    int16_t servos[8];

    // Number of active outputs
    uint8_t num_motors;
    uint8_t num_servos;
};

// Telemetry from controller
struct ControllerTelemetry {
    // Attitude (degrees)
    float roll;
    float pitch;
    float yaw;

    // Angular rates (deg/s)
    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    // Accelerations (g)
    float accel_x;
    float accel_y;
    float accel_z;

    // Controller state
    bool armed;
    bool in_flight;
    uint8_t flight_mode;

    // Loop timing
    uint32_t loop_time_us;
    uint32_t loop_count;
};


/**
 * Abstract Controller Interface
 *
 * Inherit from this class to create custom controllers.
 */
class ControllerInterface {
public:
    virtual ~ControllerInterface() {}

    /**
     * Initialize the controller
     * Called once at startup on Core 0
     */
    virtual void init() = 0;

    /**
     * Main control loop update
     * Called at CONTROL_LOOP_FREQ Hz on Core 0
     *
     * @param input Current input state from web/radio
     * @param dt Time since last update (seconds)
     */
    virtual void update(const ControllerInput& input, float dt) = 0;

    /**
     * Get motor outputs
     * Called after update() to retrieve motor commands
     *
     * @param output Output struct to fill with motor/servo commands
     */
    virtual void getOutputs(ControllerOutput& output) = 0;

    /**
     * Get telemetry for web dashboard
     * Called periodically from Core 1
     *
     * @param telemetry Struct to fill with current state
     */
    virtual void getTelemetry(ControllerTelemetry& telemetry) {
        // Default: no telemetry
        memset(&telemetry, 0, sizeof(telemetry));
    }

    /**
     * Arm/disarm the controller
     * @param arm true to arm, false to disarm
     * @return true if arm state changed
     */
    virtual bool setArmed(bool arm) {
        _armed = arm;
        return true;
    }

    /**
     * Check if controller is armed
     */
    virtual bool isArmed() { return _armed; }

    /**
     * Get controller name for display
     */
    virtual const char* getName() { return "Generic"; }

    /**
     * Get desired loop frequency (Hz)
     * Return 0 to use default CONTROL_LOOP_FREQ
     */
    virtual uint16_t getLoopFrequency() { return 0; }

protected:
    bool _armed = false;
};


/**
 * Built-in Default Controller
 *
 * Simple mixer that maps throttle/steering to outputs.
 * Used when no external controller is registered.
 */
class DefaultController : public ControllerInterface {
public:
    void init() override;
    void update(const ControllerInput& input, float dt) override;
    void getOutputs(ControllerOutput& output) override;
    void getTelemetry(ControllerTelemetry& telemetry) override;
    const char* getName() override { return "Default Mixer"; }

private:
    ControllerOutput _output;
    float _throttle;
    float _steering;
};


// Global controller instance
extern ControllerInterface* activeController;
extern DefaultController defaultController;

// Set the active controller (call from setup before startControlLoop)
void setController(ControllerInterface* controller);

// Get the active controller
ControllerInterface* getController();

// Shared input state (written by web server, read by controller)
extern volatile ControllerInput sharedInput;
extern portMUX_TYPE inputMutex;

// Update shared input (thread-safe, call from web server)
void updateControllerInput(const ControllerInput& input);

// Get shared input (thread-safe, call from control loop)
ControllerInput getControllerInput();

#endif // CONTROLLER_INTERFACE_H
