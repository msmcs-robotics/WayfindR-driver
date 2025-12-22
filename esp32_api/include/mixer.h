#ifndef MIXER_H
#define MIXER_H

#include <Arduino.h>
#include "config.h"

/**
 * Mixer - Converts control inputs to channel outputs
 *
 * Vehicle-specific mixing:
 * - Car: Differential drive (throttle + steering → left/right motors)
 * - Boat: Same as car but 2 motors
 * - Plane: Throttle → ESC, steering → rudder, pitch → elevator, roll → ailerons
 * - Quad: Requires full flight controller (throttle, pitch, roll, yaw → 4 motors)
 */

class Mixer {
public:
    Mixer();

    // Mix control inputs to channel outputs
    // Returns array of MAX_CHANNELS values (-1000 to 1000)
    void mix(float throttle, float steering, float pitch, float roll, int16_t* outputs);

    // Set mixing parameters
    void setTurnSensitivity(float sensitivity);
    void setExpo(float expo);  // Exponential curve (0 = linear, 1 = full expo)

    // Get vehicle type string
    const char* getVehicleType();

private:
    float _turnSensitivity;
    float _expo;

    // Apply exponential curve to input
    float applyExpo(float input, float expo);

    // Vehicle-specific mixing functions
    void mixCar(float throttle, float steering, int16_t* outputs);
    void mixBoat(float throttle, float steering, int16_t* outputs);
    void mixPlane(float throttle, float steering, float pitch, float roll, int16_t* outputs);
    void mixQuad(float throttle, float steering, float pitch, float roll, int16_t* outputs);
};

extern Mixer mixer;

#endif // MIXER_H
