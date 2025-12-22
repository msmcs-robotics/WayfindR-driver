#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

class RobotWebServer {
public:
    RobotWebServer(uint16_t port = 80);

    void begin();
    void loop();

    // WebSocket broadcast
    void broadcastTelemetry();

private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;

    // Route handlers
    void setupRoutes();
    void setupWebSocket();

    // API handlers
    void handleMove(AsyncWebServerRequest *request);
    void handleStop(AsyncWebServerRequest *request);
    void handleForward(AsyncWebServerRequest *request);
    void handleBackward(AsyncWebServerRequest *request);
    void handleRotate(AsyncWebServerRequest *request);
    void handleStatus(AsyncWebServerRequest *request);
    void handleCommand(AsyncWebServerRequest *request);

    // WebSocket handlers
    void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                   AwsEventType type, void *arg, uint8_t *data, size_t len);

    // Utility
    void sendJson(AsyncWebServerRequest *request, JsonDocument &doc, int code = 200);
    void sendSuccess(AsyncWebServerRequest *request, const char *message);
    void sendError(AsyncWebServerRequest *request, const char *message, int code = 400);
};

extern RobotWebServer webServer;

#endif // WEB_SERVER_H
