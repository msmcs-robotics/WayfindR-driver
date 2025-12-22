/**
 * Example: Flight Controller Wrapper for ESP32
 *
 * This shows how to wrap an existing flight controller (like dRehmFlight)
 * to work with the WayfindR control interface.
 *
 * The key insight is that most flight controllers have these common elements:
 * 1. IMU reading and filtering (Madgwick/Mahony)
 * 2. PID controllers for attitude stabilization
 * 3. Motor mixing for specific frame type
 *
 * This wrapper adapts those components to our ControllerInterface.
 */

#ifndef FLIGHT_CONTROLLER_WRAPPER_H
#define FLIGHT_CONTROLLER_WRAPPER_H

#include "controller_interface.h"

// ============================================================================
// IMU Interface - Adapt to your specific sensor
// ============================================================================

struct IMUData {
    float accel_x, accel_y, accel_z;  // m/s^2 or g's
    float gyro_x, gyro_y, gyro_z;     // rad/s or deg/s
    float mag_x, mag_y, mag_z;        // Optional magnetometer
    bool valid;
};

// Base class for IMU drivers
class IMUInterface {
public:
    virtual void init() = 0;
    virtual void read(IMUData& data) = 0;
    virtual const char* getName() { return "GenericIMU"; }
};

// ============================================================================
// Attitude Estimator - Madgwick or Mahony filter
// ============================================================================

class AttitudeEstimator {
public:
    // Quaternion state
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

    // Euler angles output (degrees)
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    // Filter gain (higher = trusts gyro more, lower = trusts accel more)
    float beta = 0.04f;  // Madgwick default

    void update(const IMUData& imu, float dt);
    void getEuler(float& roll, float& pitch, float& yaw);

private:
    float invSqrt(float x);
};

// ============================================================================
// PID Controller
// ============================================================================

class PIDController {
public:
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    float integralLimit = 100.0f;
    float outputLimit = 400.0f;

    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    float update(float setpoint, float measurement, float dt);
    void reset();

private:
    float integral = 0.0f;
    float prevError = 0.0f;
};

// ============================================================================
// Flight Controller Implementation
// ============================================================================

class FlightControllerWrapper : public ControllerInterface {
public:
    FlightControllerWrapper(IMUInterface* imu);

    // ControllerInterface implementation
    void init() override;
    void update(const ControllerInput& input, float dt) override;
    void getOutputs(ControllerOutput& output) override;
    void getTelemetry(ControllerTelemetry& telemetry) override;

    const char* getName() override { return "FlightController"; }
    uint16_t getLoopFrequency() override { return 500; }  // 500Hz recommended

    // Arming
    void setArmed(bool armed) override { _armed = armed; }
    bool isArmed() override { return _armed; }

    // Configuration
    void setPIDGains(float rollP, float rollI, float rollD,
                     float pitchP, float pitchI, float pitchD,
                     float yawP, float yawI, float yawD);

    void setAngleMode(bool enabled) { _angleMode = enabled; }
    void setMaxAngle(float degrees) { _maxAngle = degrees; }

private:
    IMUInterface* _imu;
    AttitudeEstimator _attitude;

    // PID controllers
    PIDController _rollPID{1.0f, 0.0f, 0.0f};
    PIDController _pitchPID{1.0f, 0.0f, 0.0f};
    PIDController _yawPID{1.0f, 0.0f, 0.0f};

    // State
    bool _armed = false;
    bool _angleMode = true;  // Angle mode (self-leveling) vs rate mode
    float _maxAngle = 30.0f; // Max angle in degrees for angle mode

    // Outputs
    int16_t _motorOutputs[4] = {0};

    // Mix motor outputs for X-configuration quad
    void mixQuadX(float throttle, float rollCmd, float pitchCmd, float yawCmd);
};

#endif // FLIGHT_CONTROLLER_WRAPPER_H
