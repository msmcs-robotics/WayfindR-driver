/**
 * Flight Controller Wrapper Implementation
 *
 * This is a minimal implementation showing the pattern.
 * For a real flight controller, you would:
 * 1. Use a proper IMU driver (MPU6050, ICM20689, etc.)
 * 2. Tune PID gains for your specific airframe
 * 3. Add more safety features (low battery, GPS fence, etc.)
 */

#include "flight_controller_wrapper.h"
#include <Arduino.h>
#include <math.h>

// ============================================================================
// Attitude Estimator (Simplified Madgwick)
// ============================================================================

float AttitudeEstimator::invSqrt(float x) {
    // Fast inverse square root (Quake III style, good enough for this)
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void AttitudeEstimator::update(const IMUData& imu, float dt) {
    float gx = imu.gyro_x * 0.0174533f;  // deg/s to rad/s
    float gy = imu.gyro_y * 0.0174533f;
    float gz = imu.gyro_z * 0.0174533f;
    float ax = imu.accel_x;
    float ay = imu.accel_y;
    float az = imu.accel_z;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // Rate of change from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient descent corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Calculate Euler angles
    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2958f;
    pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.2958f;
    yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2958f;
}

void AttitudeEstimator::getEuler(float& r, float& p, float& y) {
    r = roll;
    p = pitch;
    y = yaw;
}

// ============================================================================
// PID Controller
// ============================================================================

float PIDController::update(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float pTerm = kp * error;

    // Integral with anti-windup
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);
    float iTerm = ki * integral;

    // Derivative (on measurement to avoid setpoint spikes)
    float derivative = (error - prevError) / dt;
    float dTerm = kd * derivative;
    prevError = error;

    // Sum and limit
    float output = pTerm + iTerm + dTerm;
    return constrain(output, -outputLimit, outputLimit);
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}

// ============================================================================
// Flight Controller
// ============================================================================

FlightControllerWrapper::FlightControllerWrapper(IMUInterface* imu) : _imu(imu) {
}

void FlightControllerWrapper::init() {
    Serial.println("Flight controller initializing...");

    if (_imu) {
        _imu->init();
        Serial.printf("IMU: %s\n", _imu->getName());
    }

    // Reset PIDs
    _rollPID.reset();
    _pitchPID.reset();
    _yawPID.reset();

    // Clear outputs
    for (int i = 0; i < 4; i++) {
        _motorOutputs[i] = 1000;  // ESC minimum
    }

    Serial.printf("Flight controller ready (Loop: %dHz)\n", getLoopFrequency());
}

void FlightControllerWrapper::update(const ControllerInput& input, float dt) {
    // Read IMU
    IMUData imuData = {0};
    if (_imu) {
        _imu->read(imuData);
    }

    // Update attitude estimate
    _attitude.update(imuData, dt);

    float currentRoll, currentPitch, currentYaw;
    _attitude.getEuler(currentRoll, currentPitch, currentYaw);

    // If not armed, keep motors at minimum
    if (!_armed || input.failsafe) {
        for (int i = 0; i < 4; i++) {
            _motorOutputs[i] = 1000;
        }
        _rollPID.reset();
        _pitchPID.reset();
        _yawPID.reset();
        return;
    }

    // Calculate setpoints from stick input
    float rollSetpoint, pitchSetpoint, yawRate;

    if (_angleMode) {
        // Angle mode: sticks control target angle
        rollSetpoint = input.roll * _maxAngle;    // e.g., +/- 30 degrees
        pitchSetpoint = input.pitch * _maxAngle;
    } else {
        // Rate mode: sticks control rotation rate
        rollSetpoint = input.roll * 250.0f;       // deg/s
        pitchSetpoint = input.pitch * 250.0f;
    }
    yawRate = input.yaw * 200.0f;  // Yaw is always rate-controlled

    // Run PID controllers
    float rollCmd, pitchCmd, yawCmd;

    if (_angleMode) {
        // Outer loop: angle -> rate
        rollCmd = _rollPID.update(rollSetpoint, currentRoll, dt);
        pitchCmd = _pitchPID.update(pitchSetpoint, currentPitch, dt);
    } else {
        // Direct rate control
        rollCmd = _rollPID.update(rollSetpoint, imuData.gyro_x, dt);
        pitchCmd = _pitchPID.update(pitchSetpoint, imuData.gyro_y, dt);
    }
    yawCmd = _yawPID.update(yawRate, imuData.gyro_z, dt);

    // Map throttle from -1..1 to 0..1, then to ESC range
    float throttle = (input.throttle + 1.0f) * 0.5f;  // 0 to 1
    throttle = constrain(throttle, 0.0f, 1.0f);

    // Mix motor outputs
    mixQuadX(throttle, rollCmd, pitchCmd, yawCmd);
}

void FlightControllerWrapper::mixQuadX(float throttle, float rollCmd, float pitchCmd, float yawCmd) {
    // X-configuration quadcopter mixing
    //
    //    Front
    //  FL(CCW)  FR(CW)
    //      \  /
    //       \/
    //       /\
    //      /  \
    //  RL(CW)  RR(CCW)
    //    Rear
    //
    // Motor numbering: 0=FL, 1=FR, 2=RL, 3=RR

    // Base throttle (1000-2000 ESC range)
    float baseThrottle = 1000.0f + throttle * 1000.0f;

    // Scale commands for ESC (typically +/- 200 max adjustment)
    float rollMix = rollCmd * 0.5f;
    float pitchMix = pitchCmd * 0.5f;
    float yawMix = yawCmd * 0.3f;

    // Mix
    float m0 = baseThrottle - rollMix + pitchMix + yawMix;  // FL (CCW)
    float m1 = baseThrottle + rollMix + pitchMix - yawMix;  // FR (CW)
    float m2 = baseThrottle - rollMix - pitchMix - yawMix;  // RL (CW)
    float m3 = baseThrottle + rollMix - pitchMix + yawMix;  // RR (CCW)

    // Constrain to ESC range
    _motorOutputs[0] = (int16_t)constrain(m0, 1000.0f, 2000.0f);
    _motorOutputs[1] = (int16_t)constrain(m1, 1000.0f, 2000.0f);
    _motorOutputs[2] = (int16_t)constrain(m2, 1000.0f, 2000.0f);
    _motorOutputs[3] = (int16_t)constrain(m3, 1000.0f, 2000.0f);
}

void FlightControllerWrapper::getOutputs(ControllerOutput& output) {
    output.num_motors = 4;
    output.num_servos = 0;

    for (int i = 0; i < 4; i++) {
        output.motors[i] = _motorOutputs[i];
    }
}

void FlightControllerWrapper::getTelemetry(ControllerTelemetry& telemetry) {
    _attitude.getEuler(telemetry.roll, telemetry.pitch, telemetry.yaw);
    telemetry.armed = _armed;
    telemetry.in_flight = _armed && (_motorOutputs[0] > 1100);
    telemetry.flight_mode = _angleMode ? 1 : 0;
}

void FlightControllerWrapper::setPIDGains(float rollP, float rollI, float rollD,
                                           float pitchP, float pitchI, float pitchD,
                                           float yawP, float yawI, float yawD) {
    _rollPID.kp = rollP;
    _rollPID.ki = rollI;
    _rollPID.kd = rollD;

    _pitchPID.kp = pitchP;
    _pitchPID.ki = pitchI;
    _pitchPID.kd = pitchD;

    _yawPID.kp = yawP;
    _yawPID.ki = yawI;
    _yawPID.kd = yawD;
}
