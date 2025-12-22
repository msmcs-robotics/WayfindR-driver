#include "controller_interface.h"
#include "output_channels.h"
#include "mixer.h"

// Global instances
ControllerInterface* activeController = nullptr;
DefaultController defaultController;

// Shared input state
volatile ControllerInput sharedInput = {0};
portMUX_TYPE inputMutex = portMUX_INITIALIZER_UNLOCKED;

void setController(ControllerInterface* controller) {
    activeController = controller;
    if (activeController) {
        Serial.printf("Controller set: %s\n", activeController->getName());
    }
}

ControllerInterface* getController() {
    if (activeController) {
        return activeController;
    }
    return &defaultController;
}

void updateControllerInput(const ControllerInput& input) {
    portENTER_CRITICAL(&inputMutex);
    sharedInput.throttle = input.throttle;
    sharedInput.roll = input.roll;
    sharedInput.pitch = input.pitch;
    sharedInput.yaw = input.yaw;
    sharedInput.armed = input.armed;
    sharedInput.failsafe = input.failsafe;
    sharedInput.timestamp_ms = millis();
    for (int i = 0; i < 4; i++) {
        sharedInput.aux[i] = input.aux[i];
    }
    portEXIT_CRITICAL(&inputMutex);
}

ControllerInput getControllerInput() {
    ControllerInput input;
    portENTER_CRITICAL(&inputMutex);
    input.throttle = sharedInput.throttle;
    input.roll = sharedInput.roll;
    input.pitch = sharedInput.pitch;
    input.yaw = sharedInput.yaw;
    input.armed = sharedInput.armed;
    input.failsafe = sharedInput.failsafe;
    input.timestamp_ms = sharedInput.timestamp_ms;
    for (int i = 0; i < 4; i++) {
        input.aux[i] = sharedInput.aux[i];
    }
    portEXIT_CRITICAL(&inputMutex);
    return input;
}


// ============================================================================
// Default Controller Implementation
// ============================================================================

void DefaultController::init() {
    Serial.println("Default mixer controller initialized");
    _throttle = 0;
    _steering = 0;
    memset(&_output, 0, sizeof(_output));

    // Configure output counts based on vehicle type
    #if defined(VEHICLE_TYPE_CAR)
        _output.num_motors = 4;
        _output.num_servos = 0;
    #elif defined(VEHICLE_TYPE_BOAT)
        _output.num_motors = 2;
        _output.num_servos = 0;
    #elif defined(VEHICLE_TYPE_PLANE)
        _output.num_motors = 1;
        _output.num_servos = 3;
    #elif defined(VEHICLE_TYPE_QUAD)
        _output.num_motors = 4;
        _output.num_servos = 0;
    #else
        _output.num_motors = 4;
        _output.num_servos = 0;
    #endif
}

void DefaultController::update(const ControllerInput& input, float dt) {
    _throttle = input.throttle;
    _steering = input.yaw;  // Use yaw as steering for ground vehicles

    // Use the mixer to calculate outputs
    int16_t mixerOutputs[MAX_CHANNELS];
    mixer.mix(input.throttle, input.yaw, input.pitch, input.roll, mixerOutputs);

    // Copy to output struct
    for (int i = 0; i < _output.num_motors; i++) {
        _output.motors[i] = mixerOutputs[i];
    }
}

void DefaultController::getOutputs(ControllerOutput& output) {
    output = _output;
}

void DefaultController::getTelemetry(ControllerTelemetry& telemetry) {
    telemetry.roll = 0;
    telemetry.pitch = 0;
    telemetry.yaw = 0;
    telemetry.armed = _armed;
    telemetry.in_flight = false;
    telemetry.flight_mode = 0;
}
