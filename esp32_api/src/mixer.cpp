#include "mixer.h"

Mixer mixer;

Mixer::Mixer() : _turnSensitivity(TURN_SENSITIVITY), _expo(0.0f) {}

void Mixer::mix(float throttle, float steering, float pitch, float roll, int16_t* outputs) {
    // Initialize all outputs to 0
    for (int i = 0; i < MAX_CHANNELS; i++) {
        outputs[i] = 0;
    }

    // Apply expo curve if set
    throttle = applyExpo(throttle, _expo);
    steering = applyExpo(steering, _expo);
    pitch = applyExpo(pitch, _expo);
    roll = applyExpo(roll, _expo);

    // Call vehicle-specific mixer
    #ifdef VEHICLE_TYPE_CAR
    mixCar(throttle, steering, outputs);
    #elif defined(VEHICLE_TYPE_BOAT)
    mixBoat(throttle, steering, outputs);
    #elif defined(VEHICLE_TYPE_PLANE)
    mixPlane(throttle, steering, pitch, roll, outputs);
    #elif defined(VEHICLE_TYPE_QUAD)
    mixQuad(throttle, steering, pitch, roll, outputs);
    #endif
}

float Mixer::applyExpo(float input, float expo) {
    if (expo <= 0.0f) return input;

    // Exponential curve: output = input^3 * expo + input * (1 - expo)
    float sign = input >= 0 ? 1.0f : -1.0f;
    float absInput = fabs(input);
    float expoPart = absInput * absInput * absInput;
    return sign * (expoPart * expo + absInput * (1.0f - expo));
}

void Mixer::setTurnSensitivity(float sensitivity) {
    _turnSensitivity = constrain(sensitivity, 0.1f, 2.0f);
}

void Mixer::setExpo(float expo) {
    _expo = constrain(expo, 0.0f, 1.0f);
}

const char* Mixer::getVehicleType() {
    #ifdef VEHICLE_TYPE_CAR
    return "car";
    #elif defined(VEHICLE_TYPE_BOAT)
    return "boat";
    #elif defined(VEHICLE_TYPE_PLANE)
    return "plane";
    #elif defined(VEHICLE_TYPE_QUAD)
    return "quadcopter";
    #else
    return "custom";
    #endif
}

void Mixer::mixCar(float throttle, float steering, int16_t* outputs) {
    /**
     * 4-wheel skid steer mixing
     * Based on fsia6b_UNO_skidsteer differential drive logic
     *
     * Channels:
     * 0: Left Front
     * 1: Left Rear
     * 2: Right Front
     * 3: Right Rear
     */

    float leftSpeed = throttle;
    float rightSpeed = throttle;

    // Apply differential steering
    if (fabs(steering) > 0.05f) {
        float steerAmount = fabs(steering) * _turnSensitivity;

        if (steering > 0) {
            // Steering right: slow down right side
            if (fabs(throttle) < 0.1f) {
                // Pivot turn when throttle is low
                leftSpeed = steerAmount;
                rightSpeed = -steerAmount;
            } else {
                // Normal turn
                leftSpeed = throttle;
                rightSpeed = throttle - steerAmount;

                // Prevent sign flip during normal turns
                if (throttle > 0 && rightSpeed < 0) rightSpeed = 0;
                if (throttle < 0 && rightSpeed > 0) rightSpeed = 0;
            }
        } else {
            // Steering left: slow down left side
            if (fabs(throttle) < 0.1f) {
                // Pivot turn
                leftSpeed = -steerAmount;
                rightSpeed = steerAmount;
            } else {
                // Normal turn
                rightSpeed = throttle;
                leftSpeed = throttle - steerAmount;

                // Prevent sign flip
                if (throttle > 0 && leftSpeed < 0) leftSpeed = 0;
                if (throttle < 0 && leftSpeed > 0) leftSpeed = 0;
            }
        }
    }

    // Constrain and scale to -1000 to 1000
    leftSpeed = constrain(leftSpeed, -1.0f, 1.0f);
    rightSpeed = constrain(rightSpeed, -1.0f, 1.0f);

    int16_t leftOut = (int16_t)(leftSpeed * 1000);
    int16_t rightOut = (int16_t)(rightSpeed * 1000);

    // Apply to channels (front and rear same on each side)
    outputs[0] = leftOut;   // Left Front
    outputs[1] = leftOut;   // Left Rear
    outputs[2] = rightOut;  // Right Front
    outputs[3] = rightOut;  // Right Rear
}

void Mixer::mixBoat(float throttle, float steering, int16_t* outputs) {
    /**
     * 2-motor boat mixing (same as car but only 2 channels)
     *
     * Channels:
     * 0: Left Motor
     * 1: Right Motor
     */

    float leftSpeed = throttle;
    float rightSpeed = throttle;

    if (fabs(steering) > 0.05f) {
        float steerAmount = fabs(steering) * _turnSensitivity;

        if (steering > 0) {
            // Turn right
            if (fabs(throttle) < 0.1f) {
                leftSpeed = steerAmount;
                rightSpeed = -steerAmount;
            } else {
                rightSpeed = throttle - steerAmount;
                if (throttle > 0 && rightSpeed < 0) rightSpeed = 0;
                if (throttle < 0 && rightSpeed > 0) rightSpeed = 0;
            }
        } else {
            // Turn left
            if (fabs(throttle) < 0.1f) {
                leftSpeed = -steerAmount;
                rightSpeed = steerAmount;
            } else {
                leftSpeed = throttle - steerAmount;
                if (throttle > 0 && leftSpeed < 0) leftSpeed = 0;
                if (throttle < 0 && leftSpeed > 0) leftSpeed = 0;
            }
        }
    }

    leftSpeed = constrain(leftSpeed, -1.0f, 1.0f);
    rightSpeed = constrain(rightSpeed, -1.0f, 1.0f);

    outputs[0] = (int16_t)(leftSpeed * 1000);
    outputs[1] = (int16_t)(rightSpeed * 1000);
}

void Mixer::mixPlane(float throttle, float steering, float pitch, float roll, int16_t* outputs) {
    /**
     * Airplane mixing
     *
     * Channels:
     * 0: Throttle (ESC)
     * 1: Aileron (roll)
     * 2: Elevator (pitch)
     * 3: Rudder (yaw/steering)
     */

    // Throttle: 0 to 1000 for ESC
    outputs[0] = (int16_t)(constrain(throttle, 0.0f, 1.0f) * 1000);

    // Aileron: -1000 to 1000
    outputs[1] = (int16_t)(constrain(roll, -1.0f, 1.0f) * 1000);

    // Elevator: -1000 to 1000
    outputs[2] = (int16_t)(constrain(pitch, -1.0f, 1.0f) * 1000);

    // Rudder: -1000 to 1000
    outputs[3] = (int16_t)(constrain(steering, -1.0f, 1.0f) * 1000);
}

void Mixer::mixQuad(float throttle, float steering, float pitch, float roll, int16_t* outputs) {
    /**
     * Quadcopter mixing (basic, for testing only)
     * For real flight, use a proper flight controller!
     *
     * Channels:
     * 0: Front Left (CCW)
     * 1: Front Right (CW)
     * 2: Rear Left (CW)
     * 3: Rear Right (CCW)
     *
     * This is a very basic mixer - a real quad needs:
     * - PID control loops
     * - IMU feedback
     * - Attitude estimation
     * - Much higher update rates
     */

    // Base throttle (0 to 1000 for ESC)
    float baseThrottle = constrain(throttle, 0.0f, 1.0f) * 1000;

    // Control inputs scaled for mixing
    float yaw = steering * 200;   // Yaw authority
    float pitchCtrl = pitch * 200;
    float rollCtrl = roll * 200;

    // X-configuration mixing
    float fl = baseThrottle - pitchCtrl + rollCtrl - yaw;  // Front Left
    float fr = baseThrottle - pitchCtrl - rollCtrl + yaw;  // Front Right
    float rl = baseThrottle + pitchCtrl + rollCtrl + yaw;  // Rear Left
    float rr = baseThrottle + pitchCtrl - rollCtrl - yaw;  // Rear Right

    // Constrain to ESC range
    outputs[0] = (int16_t)constrain(fl, 0, 1000);
    outputs[1] = (int16_t)constrain(fr, 0, 1000);
    outputs[2] = (int16_t)constrain(rl, 0, 1000);
    outputs[3] = (int16_t)constrain(rr, 0, 1000);
}
