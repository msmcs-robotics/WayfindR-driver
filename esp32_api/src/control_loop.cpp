#include "control_loop.h"
#include "controller_interface.h"
#include "output_channels.h"
#include "mixer.h"

// Shared control input state (legacy, for backward compatibility)
volatile ControlInput controlInput = {0, 0, 0, 0, false, 0};
portMUX_TYPE controlInputMutex = portMUX_INITIALIZER_UNLOCKED;

// Task handle
TaskHandle_t controlLoopTask = NULL;

// Control loop internals
static bool _running = false;
static unsigned long _loopCount = 0;
static unsigned long _lastLoopTime = 0;
static uint32_t _avgLoopTimeUs = 0;

// Forward declaration
void controlLoopTaskFunc(void* parameter);

void startControlLoop() {
    if (_running) return;

    Serial.println("Starting control loop on Core 0...");

    // Initialize the active controller
    ControllerInterface* controller = getController();
    controller->init();

    // Get loop frequency from controller or use default
    uint16_t loopFreq = controller->getLoopFrequency();
    if (loopFreq == 0) {
        loopFreq = CONTROL_LOOP_FREQ;
    }

    Serial.printf("Controller: %s\n", controller->getName());
    Serial.printf("Loop frequency: %d Hz\n", loopFreq);

    // Create task pinned to Core 0
    xTaskCreatePinnedToCore(
        controlLoopTaskFunc,    // Task function
        "ControlLoop",          // Name
        8192,                   // Stack size (increased for complex controllers)
        NULL,                   // Parameters
        configMAX_PRIORITIES - 1,  // Highest priority
        &controlLoopTask,       // Task handle
        CONTROL_CORE            // Core 0
    );

    _running = true;
}

void stopControlLoop() {
    if (!_running) return;

    _running = false;

    if (controlLoopTask != NULL) {
        vTaskDelete(controlLoopTask);
        controlLoopTask = NULL;
    }

    outputChannels.stopAll();
    Serial.println("Control loop stopped");
}

void controlLoopTaskFunc(void* parameter) {
    ControllerInterface* controller = getController();

    // Get loop frequency
    uint16_t loopFreq = controller->getLoopFrequency();
    if (loopFreq == 0) {
        loopFreq = CONTROL_LOOP_FREQ;
    }

    const unsigned long loopPeriodUs = 1000000 / loopFreq;
    unsigned long lastTime = micros();
    unsigned long loopStartTime;

    Serial.printf("Control loop running on Core %d at %d Hz\n",
                  xPortGetCoreID(), loopFreq);

    while (_running) {
        loopStartTime = micros();
        unsigned long now = loopStartTime;
        unsigned long deltaTime = now - lastTime;

        // Wait for next loop period if we're running too fast
        if (deltaTime < loopPeriodUs) {
            delayMicroseconds(loopPeriodUs - deltaTime);
            now = micros();
            deltaTime = now - lastTime;
        }

        lastTime = now;
        _loopCount++;

        // Calculate dt in seconds
        float dt = deltaTime / 1000000.0f;

        // Get input from shared state
        ControllerInput input = getControllerInput();

        // Check for watchdog timeout (failsafe)
        unsigned long timeSinceCommand = millis() - input.timestamp_ms;
        if (FAILSAFE_ENABLED && timeSinceCommand > WATCHDOG_TIMEOUT_MS) {
            input.failsafe = true;
            input.throttle = 0;
            input.roll = 0;
            input.pitch = 0;
            input.yaw = 0;
        }

        // Update the controller
        controller->update(input, dt);

        // Get outputs from controller
        ControllerOutput output;
        controller->getOutputs(output);

        // Apply outputs to channels
        for (int i = 0; i < output.num_motors && i < MAX_CHANNELS; i++) {
            outputChannels.setChannel(i, output.motors[i]);
        }

        // Track loop timing
        unsigned long loopTime = micros() - loopStartTime;
        _avgLoopTimeUs = (_avgLoopTimeUs * 15 + loopTime) / 16;  // Exponential average

        // Yield briefly to prevent watchdog issues
        taskYIELD();
    }

    vTaskDelete(NULL);
}

// Legacy functions for backward compatibility
void setControlInput(float throttle, float steering, float pitch, float roll) {
    ControllerInput input;
    input.throttle = constrain(throttle, -1.0f, 1.0f);
    input.yaw = constrain(steering, -1.0f, 1.0f);  // steering maps to yaw
    input.pitch = constrain(pitch, -1.0f, 1.0f);
    input.roll = constrain(roll, -1.0f, 1.0f);
    input.armed = getController()->isArmed();
    input.failsafe = false;
    input.timestamp_ms = millis();

    updateControllerInput(input);

    // Also update legacy struct
    portENTER_CRITICAL(&controlInputMutex);
    controlInput.throttle = throttle;
    controlInput.steering = steering;
    controlInput.pitch = pitch;
    controlInput.roll = roll;
    controlInput.timestamp = millis();
    portEXIT_CRITICAL(&controlInputMutex);
}

ControlInput getControlInput() {
    ControlInput input;

    portENTER_CRITICAL(&controlInputMutex);
    input.throttle = controlInput.throttle;
    input.steering = controlInput.steering;
    input.pitch = controlInput.pitch;
    input.roll = controlInput.roll;
    input.armed = controlInput.armed;
    input.timestamp = controlInput.timestamp;
    portEXIT_CRITICAL(&controlInputMutex);

    return input;
}

void armMotors() {
    getController()->setArmed(true);

    portENTER_CRITICAL(&controlInputMutex);
    controlInput.armed = true;
    portEXIT_CRITICAL(&controlInputMutex);

    outputChannels.arm();
}

void disarmMotors() {
    getController()->setArmed(false);

    portENTER_CRITICAL(&controlInputMutex);
    controlInput.armed = false;
    portEXIT_CRITICAL(&controlInputMutex);

    outputChannels.disarm();
}

bool areMotorsArmed() {
    return getController()->isArmed();
}

// Get loop statistics
uint32_t getLoopTimeUs() {
    return _avgLoopTimeUs;
}

unsigned long getLoopCount() {
    return _loopCount;
}

// Default weak implementation of userControlLoop for backward compatibility
__attribute__((weak))
void userControlLoop(ControlInput input, unsigned long deltaTime) {
    // Default: do nothing, controller interface handles it
}
