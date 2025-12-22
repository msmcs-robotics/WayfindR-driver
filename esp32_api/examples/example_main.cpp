/**
 * Example: Using the Flight Controller Wrapper
 *
 * This shows how to integrate a flight controller with the WayfindR
 * ESP32 control system. The flight controller runs on Core 0 at 500Hz
 * while the web server runs on Core 1.
 *
 * To use this example:
 * 1. Copy this file to src/main.cpp
 * 2. Copy flight_controller_wrapper.h/.cpp to include/ and src/
 * 3. Copy mpu6050_imu.h to include/
 * 4. Update config.h with VEHICLE_TYPE_QUAD
 * 5. Build and upload
 */

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "output_channels.h"
#include "control_loop.h"
#include "controller_interface.h"
#include "web_server.h"

// Include the flight controller wrapper
#include "flight_controller_wrapper.h"
#include "mpu6050_imu.h"

// Create IMU and flight controller instances
MPU6050_IMU imu(0x68);
FlightControllerWrapper flightController(&imu);

// Telemetry timing
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 1000 / TELEMETRY_RATE_HZ;

void setupWiFi() {
    Serial.println("Connecting to WiFi...");
    Serial.printf("SSID: %s\n", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi failed, starting Access Point...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        Serial.print("AP SSID: ");
        Serial.println(AP_SSID);
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  WayfindR Flight Controller Example");
    Serial.println("  ESP32 Dual-Core Architecture");
    Serial.println("========================================\n");

    Serial.printf("ESP32 Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());

    // Initialize output channels
    outputChannels.begin();

    // Calibrate IMU (optional, but recommended)
    imu.init();
    imu.calibrate(500);  // 500 samples, takes ~1 second

    // Configure flight controller
    flightController.setPIDGains(
        2.0f, 0.02f, 0.5f,   // Roll: P, I, D
        2.0f, 0.02f, 0.5f,   // Pitch: P, I, D
        3.0f, 0.01f, 0.0f    // Yaw: P, I, D
    );
    flightController.setAngleMode(true);  // Self-leveling mode
    flightController.setMaxAngle(30.0f);  // Max 30 degree tilt

    // Register flight controller with the control loop
    setController(&flightController);

    // Connect to WiFi
    setupWiFi();

    // Start web server
    webServer.begin();

    // Start control loop on Core 0 (uses our flight controller)
    startControlLoop();

    Serial.println("\n========================================");
    Serial.println("Flight Controller Ready!");
    Serial.println("========================================");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP());
    Serial.println("\nWARNING: PROPS OFF for initial testing!");
    Serial.println("Use /api/arm to arm motors");
    Serial.println("========================================\n");
}

void loop() {
    // This runs on Core 1 (Arduino default)

    // Handle web server
    webServer.loop();

    // Broadcast telemetry
    if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
        webServer.broadcastTelemetry();
        lastTelemetryTime = millis();
    }

    delay(1);
}
