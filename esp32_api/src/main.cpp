/**
 * WayfindR ESP32 Robot Control API
 * Dual-Core Architecture
 *
 * Core 0: Control loop (100Hz)
 *   - Motor/servo/ESC output
 *   - Mixer (differential drive, plane, quad)
 *   - Watchdog safety
 *   - Optional: custom flight controller
 *
 * Core 1: Web server (default Arduino core)
 *   - WiFi management
 *   - HTTP REST API
 *   - WebSocket real-time control
 *   - Dashboard UI
 *
 * Both cores run at 240MHz (ESP32 default)
 * This provides ample processing for 100-400Hz control loops
 * while maintaining responsive web interface.
 */

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "output_channels.h"
#include "control_loop.h"
#include "mixer.h"
#include "web_server.h"

// Telemetry broadcast timing
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 1000 / TELEMETRY_RATE_HZ;

void setupWiFi() {
    Serial.println("Connecting to WiFi...");
    Serial.printf("SSID: %s\n", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait up to 10 seconds for connection
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
        // Fall back to Access Point mode
        Serial.println("\nWiFi failed, starting Access Point...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        Serial.print("AP SSID: ");
        Serial.println(AP_SSID);
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
}

void printSystemInfo() {
    Serial.println("\n========================================");
    Serial.println("  WayfindR ESP32 Robot Control");
    Serial.println("  Dual-Core Architecture");
    Serial.println("========================================\n");

    Serial.printf("ESP32 Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / 1024 / 1024);

    Serial.println("\nCore Assignment:");
    Serial.printf("  Core 0: Control loop (%d Hz)\n", CONTROL_LOOP_FREQ);
    Serial.printf("  Core 1: Web server + WiFi\n");

    Serial.printf("\nVehicle Type: %s\n", mixer.getVehicleType());
    Serial.printf("Output Channels: %d enabled\n", outputChannels.getEnabledCount());

    #if ARM_REQUIRED
    Serial.println("Arming: REQUIRED before motors spin");
    #else
    Serial.println("Arming: Not required (motors active immediately)");
    #endif

    Serial.printf("Watchdog: %d ms timeout\n", WATCHDOG_TIMEOUT_MS);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    printSystemInfo();

    // Initialize output channels (motors/servos/ESCs)
    outputChannels.begin();

    // Connect to WiFi (runs on Core 1)
    setupWiFi();

    // Start web server (runs on Core 1)
    webServer.begin();

    // Start control loop (runs on Core 0)
    startControlLoop();

    Serial.println("\n========================================");
    Serial.println("System Ready!");
    Serial.println("========================================");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP());
    Serial.println("API: /api/move, /api/stop, /api/status");
    Serial.println("WebSocket: ws://<ip>/ws");
    Serial.println("========================================\n");
}

void loop() {
    // This runs on Core 1 (Arduino default)

    // Handle web server tasks
    webServer.loop();

    // Broadcast telemetry to WebSocket clients
    if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
        webServer.broadcastTelemetry();
        lastTelemetryTime = millis();
    }

    // Small yield for WiFi stack
    delay(1);
}
