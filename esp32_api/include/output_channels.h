#ifndef OUTPUT_CHANNELS_H
#define OUTPUT_CHANNELS_H

#include <Arduino.h>
#include "config.h"

// Forward declaration
class OutputChannels;

// Singleton instance
extern OutputChannels outputChannels;

/**
 * OutputChannels - Manages all motor/servo/ESC outputs
 *
 * Supports:
 * - DC motors with PWM only
 * - DC motors with PWM + direction pins (H-bridge)
 * - Servo motors (0-180 degrees)
 * - ESC/Brushless motors (1000-2000us)
 *
 * Thread-safe for dual-core operation.
 */
class OutputChannels {
public:
    OutputChannels();

    // Initialize all configured channels
    void begin();

    // Set individual channel output (-1000 to 1000, or us for servo/ESC)
    void setChannel(uint8_t channel, int16_t value);

    // Set all channels at once
    void setAllChannels(int16_t* values);

    // Enable/disable a channel at runtime
    void enableChannel(uint8_t channel, bool enabled);

    // Stop all outputs (emergency stop)
    void stopAll();

    // Get channel state
    int16_t getChannelValue(uint8_t channel);
    bool isChannelEnabled(uint8_t channel);
    ChannelType getChannelType(uint8_t channel);

    // Get number of enabled channels
    uint8_t getEnabledCount();

    // Arm/disarm (for safety)
    void arm();
    void disarm();
    bool isArmed() { return _armed; }

private:
    ChannelConfig _channels[MAX_CHANNELS];
    int16_t _channelValues[MAX_CHANNELS];
    bool _armed;

    // Mutex for thread safety
    portMUX_TYPE _mutex;

    // Apply output to hardware
    void applyOutput(uint8_t channel, int16_t value);

    // Channel-type specific output functions
    void outputPWMMotor(uint8_t channel, int16_t value);
    void outputPWMMotorDir(uint8_t channel, int16_t value);
    void outputServo(uint8_t channel, int16_t value);
    void outputESC(uint8_t channel, int16_t value);

    // Initialize a single channel
    void initChannel(uint8_t channel);
};

#endif // OUTPUT_CHANNELS_H
