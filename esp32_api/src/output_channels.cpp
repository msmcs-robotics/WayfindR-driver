#include "output_channels.h"
#include <ESP32Servo.h>

OutputChannels outputChannels;

// Servo objects for servo/ESC channels
static Servo servos[MAX_CHANNELS];

OutputChannels::OutputChannels() : _armed(false) {
    _mutex = portMUX_INITIALIZER_UNLOCKED;

    // Initialize all channels to default/disabled
    for (int i = 0; i < MAX_CHANNELS; i++) {
        _channels[i].enabled = false;
        _channels[i].type = CHANNEL_DISABLED;
        _channels[i].pwmPin = 0;
        _channels[i].dirPin1 = 0;
        _channels[i].dirPin2 = 0;
        _channels[i].inverted = false;
        _channels[i].minValue = 0;
        _channels[i].maxValue = 255;
        _channels[i].centerValue = 127;
        _channelValues[i] = 0;
    }
}

void OutputChannels::begin() {
    Serial.println("Initializing output channels...");

    // Configure channels from config.h defines
    #if CH0_ENABLED
    _channels[0].enabled = true;
    _channels[0].type = (ChannelType)CH0_TYPE;
    _channels[0].pwmPin = CH0_PWM_PIN;
    _channels[0].dirPin1 = CH0_DIR1_PIN;
    _channels[0].dirPin2 = CH0_DIR2_PIN;
    _channels[0].inverted = CH0_INVERTED;
    #endif

    #if CH1_ENABLED
    _channels[1].enabled = true;
    _channels[1].type = (ChannelType)CH1_TYPE;
    _channels[1].pwmPin = CH1_PWM_PIN;
    _channels[1].dirPin1 = CH1_DIR1_PIN;
    _channels[1].dirPin2 = CH1_DIR2_PIN;
    _channels[1].inverted = CH1_INVERTED;
    #endif

    #if CH2_ENABLED
    _channels[2].enabled = true;
    _channels[2].type = (ChannelType)CH2_TYPE;
    _channels[2].pwmPin = CH2_PWM_PIN;
    _channels[2].dirPin1 = CH2_DIR1_PIN;
    _channels[2].dirPin2 = CH2_DIR2_PIN;
    _channels[2].inverted = CH2_INVERTED;
    #endif

    #if CH3_ENABLED
    _channels[3].enabled = true;
    _channels[3].type = (ChannelType)CH3_TYPE;
    _channels[3].pwmPin = CH3_PWM_PIN;
    _channels[3].dirPin1 = CH3_DIR1_PIN;
    _channels[3].dirPin2 = CH3_DIR2_PIN;
    _channels[3].inverted = CH3_INVERTED;
    #endif

    #if CH4_ENABLED
    _channels[4].enabled = true;
    _channels[4].type = (ChannelType)CH4_TYPE;
    _channels[4].pwmPin = CH4_PWM_PIN;
    _channels[4].dirPin1 = CH4_DIR1_PIN;
    _channels[4].dirPin2 = CH4_DIR2_PIN;
    _channels[4].inverted = CH4_INVERTED;
    #endif

    #if CH5_ENABLED
    _channels[5].enabled = true;
    _channels[5].type = (ChannelType)CH5_TYPE;
    _channels[5].pwmPin = CH5_PWM_PIN;
    _channels[5].dirPin1 = CH5_DIR1_PIN;
    _channels[5].dirPin2 = CH5_DIR2_PIN;
    _channels[5].inverted = CH5_INVERTED;
    #endif

    #if CH6_ENABLED
    _channels[6].enabled = true;
    _channels[6].type = (ChannelType)CH6_TYPE;
    _channels[6].pwmPin = CH6_PWM_PIN;
    _channels[6].dirPin1 = CH6_DIR1_PIN;
    _channels[6].dirPin2 = CH6_DIR2_PIN;
    _channels[6].inverted = CH6_INVERTED;
    #endif

    #if CH7_ENABLED
    _channels[7].enabled = true;
    _channels[7].type = (ChannelType)CH7_TYPE;
    _channels[7].pwmPin = CH7_PWM_PIN;
    _channels[7].dirPin1 = CH7_DIR1_PIN;
    _channels[7].dirPin2 = CH7_DIR2_PIN;
    _channels[7].inverted = CH7_INVERTED;
    #endif

    // Set default min/max/center based on type
    for (int i = 0; i < MAX_CHANNELS; i++) {
        switch (_channels[i].type) {
            case CHANNEL_PWM_MOTOR:
            case CHANNEL_PWM_MOTOR_DIR:
                _channels[i].minValue = 0;
                _channels[i].maxValue = 255;
                _channels[i].centerValue = 0;
                break;
            case CHANNEL_SERVO:
                _channels[i].minValue = 0;
                _channels[i].maxValue = 180;
                _channels[i].centerValue = 90;
                break;
            case CHANNEL_ESC:
            case CHANNEL_BRUSHLESS:
                _channels[i].minValue = 1000;
                _channels[i].maxValue = 2000;
                _channels[i].centerValue = 1000;  // ESC at idle
                break;
            default:
                break;
        }
    }

    // Initialize hardware for each enabled channel
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (_channels[i].enabled) {
            initChannel(i);
        }
    }

    // Start with all outputs stopped
    stopAll();

    Serial.printf("Output channels initialized: %d enabled\n", getEnabledCount());
}

void OutputChannels::initChannel(uint8_t channel) {
    if (channel >= MAX_CHANNELS) return;

    ChannelConfig& cfg = _channels[channel];

    switch (cfg.type) {
        case CHANNEL_PWM_MOTOR:
            // Simple PWM output
            ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
            ledcAttachPin(cfg.pwmPin, channel);
            Serial.printf("CH%d: PWM motor on pin %d\n", channel, cfg.pwmPin);
            break;

        case CHANNEL_PWM_MOTOR_DIR:
            // PWM + direction pins
            ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
            ledcAttachPin(cfg.pwmPin, channel);
            pinMode(cfg.dirPin1, OUTPUT);
            pinMode(cfg.dirPin2, OUTPUT);
            digitalWrite(cfg.dirPin1, LOW);
            digitalWrite(cfg.dirPin2, LOW);
            Serial.printf("CH%d: PWM+DIR motor on pins %d/%d/%d\n",
                         channel, cfg.pwmPin, cfg.dirPin1, cfg.dirPin2);
            break;

        case CHANNEL_SERVO:
            // Servo output
            servos[channel].attach(cfg.pwmPin);
            servos[channel].write(cfg.centerValue);
            Serial.printf("CH%d: Servo on pin %d\n", channel, cfg.pwmPin);
            break;

        case CHANNEL_ESC:
        case CHANNEL_BRUSHLESS:
            // ESC output (uses Servo library for 1000-2000us)
            servos[channel].attach(cfg.pwmPin, 1000, 2000);
            servos[channel].writeMicroseconds(cfg.minValue);  // Arm at minimum
            Serial.printf("CH%d: ESC on pin %d\n", channel, cfg.pwmPin);
            break;

        default:
            break;
    }
}

void OutputChannels::setChannel(uint8_t channel, int16_t value) {
    if (channel >= MAX_CHANNELS || !_channels[channel].enabled) return;

    portENTER_CRITICAL(&_mutex);
    _channelValues[channel] = value;
    portEXIT_CRITICAL(&_mutex);

    if (_armed || !ARM_REQUIRED) {
        applyOutput(channel, value);
    }
}

void OutputChannels::setAllChannels(int16_t* values) {
    portENTER_CRITICAL(&_mutex);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        _channelValues[i] = values[i];
    }
    portEXIT_CRITICAL(&_mutex);

    if (_armed || !ARM_REQUIRED) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            if (_channels[i].enabled) {
                applyOutput(i, values[i]);
            }
        }
    }
}

void OutputChannels::applyOutput(uint8_t channel, int16_t value) {
    if (channel >= MAX_CHANNELS || !_channels[channel].enabled) return;

    // Apply inversion
    if (_channels[channel].inverted) {
        value = -value;
    }

    switch (_channels[channel].type) {
        case CHANNEL_PWM_MOTOR:
            outputPWMMotor(channel, value);
            break;
        case CHANNEL_PWM_MOTOR_DIR:
            outputPWMMotorDir(channel, value);
            break;
        case CHANNEL_SERVO:
            outputServo(channel, value);
            break;
        case CHANNEL_ESC:
        case CHANNEL_BRUSHLESS:
            outputESC(channel, value);
            break;
        default:
            break;
    }
}

void OutputChannels::outputPWMMotor(uint8_t channel, int16_t value) {
    // value: -1000 to 1000 → 0 to 255 (magnitude only, no direction)
    int pwm = map(abs(value), 0, 1000, 0, 255);
    pwm = constrain(pwm, 0, 255);
    ledcWrite(channel, pwm);
}

void OutputChannels::outputPWMMotorDir(uint8_t channel, int16_t value) {
    // value: -1000 to 1000
    // PWM: magnitude, DIR pins: direction
    ChannelConfig& cfg = _channels[channel];

    int pwm = map(abs(value), 0, 1000, 0, 255);
    pwm = constrain(pwm, 0, 255);

    if (pwm < STOP_THRESHOLD) {
        // Stop (brake)
        digitalWrite(cfg.dirPin1, LOW);
        digitalWrite(cfg.dirPin2, LOW);
        ledcWrite(channel, 0);
    } else if (value > 0) {
        // Forward
        digitalWrite(cfg.dirPin1, HIGH);
        digitalWrite(cfg.dirPin2, LOW);
        ledcWrite(channel, pwm);
    } else {
        // Reverse
        digitalWrite(cfg.dirPin1, LOW);
        digitalWrite(cfg.dirPin2, HIGH);
        ledcWrite(channel, pwm);
    }
}

void OutputChannels::outputServo(uint8_t channel, int16_t value) {
    // value: -1000 to 1000 → 0 to 180 degrees
    int angle = map(value, -1000, 1000, 0, 180);
    angle = constrain(angle, 0, 180);
    servos[channel].write(angle);
}

void OutputChannels::outputESC(uint8_t channel, int16_t value) {
    // value: 0 to 1000 → 1000 to 2000 microseconds
    // (ESCs typically don't reverse, so we use 0-1000 range)
    int us = map(constrain(value, 0, 1000), 0, 1000, 1000, 2000);
    servos[channel].writeMicroseconds(us);
}

void OutputChannels::enableChannel(uint8_t channel, bool enabled) {
    if (channel >= MAX_CHANNELS) return;

    portENTER_CRITICAL(&_mutex);
    _channels[channel].enabled = enabled;
    portEXIT_CRITICAL(&_mutex);

    if (!enabled) {
        // Stop the channel when disabled
        if (_channels[channel].type == CHANNEL_SERVO) {
            servos[channel].detach();
        } else if (_channels[channel].type == CHANNEL_ESC ||
                   _channels[channel].type == CHANNEL_BRUSHLESS) {
            servos[channel].writeMicroseconds(1000);
            servos[channel].detach();
        } else {
            ledcWrite(channel, 0);
        }
    } else {
        initChannel(channel);
    }
}

void OutputChannels::stopAll() {
    portENTER_CRITICAL(&_mutex);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        _channelValues[i] = 0;
    }
    portEXIT_CRITICAL(&_mutex);

    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (_channels[i].enabled) {
            switch (_channels[i].type) {
                case CHANNEL_PWM_MOTOR:
                case CHANNEL_PWM_MOTOR_DIR:
                    ledcWrite(i, 0);
                    if (_channels[i].type == CHANNEL_PWM_MOTOR_DIR) {
                        digitalWrite(_channels[i].dirPin1, LOW);
                        digitalWrite(_channels[i].dirPin2, LOW);
                    }
                    break;
                case CHANNEL_SERVO:
                    servos[i].write(_channels[i].centerValue);
                    break;
                case CHANNEL_ESC:
                case CHANNEL_BRUSHLESS:
                    servos[i].writeMicroseconds(1000);
                    break;
                default:
                    break;
            }
        }
    }
}

int16_t OutputChannels::getChannelValue(uint8_t channel) {
    if (channel >= MAX_CHANNELS) return 0;

    portENTER_CRITICAL(&_mutex);
    int16_t value = _channelValues[channel];
    portEXIT_CRITICAL(&_mutex);

    return value;
}

bool OutputChannels::isChannelEnabled(uint8_t channel) {
    if (channel >= MAX_CHANNELS) return false;
    return _channels[channel].enabled;
}

ChannelType OutputChannels::getChannelType(uint8_t channel) {
    if (channel >= MAX_CHANNELS) return CHANNEL_DISABLED;
    return _channels[channel].type;
}

uint8_t OutputChannels::getEnabledCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (_channels[i].enabled) count++;
    }
    return count;
}

void OutputChannels::arm() {
    Serial.println("Motors ARMED");
    _armed = true;
}

void OutputChannels::disarm() {
    Serial.println("Motors DISARMED");
    _armed = false;
    stopAll();
}
