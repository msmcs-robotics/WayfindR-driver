#include "web_server.h"
#include "output_channels.h"
#include "control_loop.h"
#include "mixer.h"
#include "config.h"
#include <SPIFFS.h>

RobotWebServer webServer;

// Static reference for callbacks
static RobotWebServer *serverInstance = nullptr;

RobotWebServer::RobotWebServer(uint16_t port)
    : _server(port), _ws("/ws") {
    serverInstance = this;
}

void RobotWebServer::begin() {
    // Initialize SPIFFS for serving static files
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed, using embedded HTML");
    }

    setupWebSocket();
    setupRoutes();

    _server.begin();
    Serial.printf("Web server started on port 80 (Core %d)\n", xPortGetCoreID());
}

void RobotWebServer::loop() {
    _ws.cleanupClients();
}

void RobotWebServer::setupWebSocket() {
    _ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                   AwsEventType type, void *arg, uint8_t *data, size_t len) {
        if (serverInstance) {
            serverInstance->onWsEvent(server, client, type, arg, data, len);
        }
    });

    _server.addHandler(&_ws);
}

void RobotWebServer::setupRoutes() {
    // Serve dashboard from SPIFFS or embedded
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (SPIFFS.exists("/index.html")) {
            request->send(SPIFFS, "/index.html", "text/html");
        } else {
            // Embedded fallback HTML (simplified for brevity)
            request->send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>WayfindR ESP32</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{font-family:Arial,sans-serif;background:#1a1a2e;color:#e0e0e0;padding:20px;display:flex;flex-direction:column;align-items:center}
        h1{color:#4ecca3;margin-bottom:15px}
        .status{padding:5px 15px;border-radius:20px;margin-bottom:20px}
        .connected{background:#4ecca3;color:#1a1a2e}
        .disconnected{background:#e74c3c;color:white}
        .controls{background:rgba(22,33,62,0.8);padding:25px;border-radius:16px}
        .dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:20px}
        .btn{width:70px;height:70px;border:none;border-radius:12px;background:#16213e;color:#e0e0e0;font-size:24px;cursor:pointer;touch-action:manipulation}
        .btn:active,.btn.active{background:#4ecca3;color:#1a1a2e}
        .btn.stop{background:#e74c3c;color:white;font-size:12px}
        .speed{width:100%;margin:15px 0}
        .speed input{width:100%}
        .telem{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:20px;padding:15px;background:rgba(15,15,35,0.5);border-radius:8px}
        .telem-item{text-align:center}
        .telem-item label{display:block;font-size:0.75rem;color:#8892b0}
        .telem-item span{font-size:1.1rem;color:#4ecca3;font-family:monospace}
        .info{margin-top:15px;text-align:center;font-size:0.85rem;color:#8892b0}
    </style>
</head>
<body>
    <h1>WayfindR Control</h1>
    <span id="status" class="status disconnected">Disconnected</span>
    <div class="controls">
        <div class="dpad">
            <button class="btn" data-t="1" data-s="-0.5">&#8598;</button>
            <button class="btn" data-t="1" data-s="0">&#8593;</button>
            <button class="btn" data-t="1" data-s="0.5">&#8599;</button>
            <button class="btn" data-t="0" data-s="-1">&#8630;</button>
            <button class="btn stop" id="stop">STOP</button>
            <button class="btn" data-t="0" data-s="1">&#8631;</button>
            <button class="btn" data-t="-1" data-s="-0.5">&#8601;</button>
            <button class="btn" data-t="-1" data-s="0">&#8595;</button>
            <button class="btn" data-t="-1" data-s="0.5">&#8600;</button>
        </div>
        <div class="speed">
            <label>Speed: <span id="speedVal">50</span>%</label>
            <input type="range" id="speedSlider" min="10" max="100" value="50">
        </div>
        <div class="telem">
            <div class="telem-item"><label>CH0</label><span id="ch0">0</span></div>
            <div class="telem-item"><label>CH1</label><span id="ch1">0</span></div>
            <div class="telem-item"><label>CH2</label><span id="ch2">0</span></div>
            <div class="telem-item"><label>CH3</label><span id="ch3">0</span></div>
            <div class="telem-item"><label>Vehicle</label><span id="vehicle">--</span></div>
            <div class="telem-item"><label>Uptime</label><span id="uptime">0s</span></div>
        </div>
    </div>
    <div class="info">WASD/Arrows | Q/E Rotate | Space Stop</div>
    <script>
        let ws,speed=0.5,keys=new Set();
        function connect(){
            ws=new WebSocket('ws://'+location.host+'/ws');
            ws.onopen=()=>{document.getElementById('status').className='status connected';document.getElementById('status').textContent='Connected';};
            ws.onclose=()=>{document.getElementById('status').className='status disconnected';document.getElementById('status').textContent='Disconnected';setTimeout(connect,2000);};
            ws.onmessage=(e)=>{try{const d=JSON.parse(e.data);
                if(d.ch0!==undefined)document.getElementById('ch0').textContent=d.ch0;
                if(d.ch1!==undefined)document.getElementById('ch1').textContent=d.ch1;
                if(d.ch2!==undefined)document.getElementById('ch2').textContent=d.ch2;
                if(d.ch3!==undefined)document.getElementById('ch3').textContent=d.ch3;
                if(d.vehicle)document.getElementById('vehicle').textContent=d.vehicle;
                if(d.uptime!==undefined)document.getElementById('uptime').textContent=d.uptime+'s';
            }catch(e){}};
        }
        connect();
        function send(t,s){if(ws&&ws.readyState===1)ws.send(JSON.stringify({throttle:t*speed,steering:s*speed}));}
        function stop(){if(ws&&ws.readyState===1)ws.send(JSON.stringify({throttle:0,steering:0}));}
        document.querySelectorAll('.btn[data-t]').forEach(btn=>{
            const t=parseFloat(btn.dataset.t),s=parseFloat(btn.dataset.s);
            ['mousedown','touchstart'].forEach(e=>btn.addEventListener(e,(ev)=>{ev.preventDefault();send(t,s);btn.classList.add('active');}));
            ['mouseup','mouseleave','touchend','touchcancel'].forEach(e=>btn.addEventListener(e,()=>{stop();btn.classList.remove('active');}));
        });
        document.getElementById('stop').addEventListener('click',stop);
        document.getElementById('speedSlider').addEventListener('input',(e)=>{speed=e.target.value/100;document.getElementById('speedVal').textContent=e.target.value;});
        const keyMap={ArrowUp:'w',ArrowDown:'s',ArrowLeft:'a',ArrowRight:'d',' ':'space'};
        document.addEventListener('keydown',(e)=>{if(e.target.tagName==='INPUT')return;const k=keyMap[e.key]||e.key.toLowerCase();if(!keys.has(k)){keys.add(k);updateKeys();}if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key))e.preventDefault();});
        document.addEventListener('keyup',(e)=>{const k=keyMap[e.key]||e.key.toLowerCase();keys.delete(k);updateKeys();});
        window.addEventListener('blur',()=>{keys.clear();stop();});
        function updateKeys(){
            if(keys.has('space')){stop();return;}
            let t=0,s=0;
            if(keys.has('w'))t=1;else if(keys.has('s'))t=-1;
            if(keys.has('a'))s=-1;else if(keys.has('d'))s=1;
            if(keys.has('q')&&t===0){send(0,-1);return;}
            if(keys.has('e')&&t===0){send(0,1);return;}
            if(t!==0||s!==0)send(t,s);else stop();
        }
    </script>
</body>
</html>
)rawliteral");
        }
    });

    // API: Move with throttle/steering
    _server.on("/api/move", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleMove(request);
    });
    _server.on("/api/move", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleMove(request);
    });

    // API: Stop
    _server.on("/api/stop", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleStop(request);
    });
    _server.on("/api/stop", HTTP_POST, [this](AsyncWebServerRequest *request) {
        handleStop(request);
    });

    // API: Forward
    _server.on("/api/forward", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleForward(request);
    });

    // API: Backward
    _server.on("/api/backward", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleBackward(request);
    });

    // API: Rotate
    _server.on("/api/rotate", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleRotate(request);
    });

    // API: Status
    _server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleStatus(request);
    });

    // API: Natural language command
    _server.on("/api/command", HTTP_GET, [this](AsyncWebServerRequest *request) {
        handleCommand(request);
    });

    // API: Arm/Disarm
    _server.on("/api/arm", HTTP_POST, [](AsyncWebServerRequest *request) {
        armMotors();
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Armed\"}");
    });
    _server.on("/api/disarm", HTTP_POST, [](AsyncWebServerRequest *request) {
        disarmMotors();
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Disarmed\"}");
    });

    // API: Channel control
    _server.on("/api/channel", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("ch") || !request->hasParam("value")) {
            request->send(400, "application/json", "{\"error\":\"Missing ch or value\"}");
            return;
        }
        uint8_t ch = request->getParam("ch")->value().toInt();
        int16_t value = request->getParam("value")->value().toInt();
        outputChannels.setChannel(ch, value);
        request->send(200, "application/json", "{\"success\":true}");
    });

    // API: Enable/disable channel
    _server.on("/api/channel/enable", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("ch")) {
            request->send(400, "application/json", "{\"error\":\"Missing ch\"}");
            return;
        }
        uint8_t ch = request->getParam("ch")->value().toInt();
        bool enable = true;
        if (request->hasParam("enable")) {
            enable = request->getParam("enable")->value() == "true";
        }
        outputChannels.enableChannel(ch, enable);
        request->send(200, "application/json", "{\"success\":true}");
    });

    // API: Get channels info
    _server.on("/api/channels", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray channels = doc["channels"].to<JsonArray>();

        for (int i = 0; i < MAX_CHANNELS; i++) {
            JsonObject ch = channels.add<JsonObject>();
            ch["id"] = i;
            ch["enabled"] = outputChannels.isChannelEnabled(i);
            ch["type"] = (int)outputChannels.getChannelType(i);
            ch["value"] = outputChannels.getChannelValue(i);
        }

        doc["vehicle"] = mixer.getVehicleType();
        doc["armed"] = areMotorsArmed();

        String json;
        serializeJson(doc, json);
        request->send(200, "application/json", json);
    });

    // Serve static files from SPIFFS
    _server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // CORS headers
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
}

void RobotWebServer::handleMove(AsyncWebServerRequest *request) {
    float throttle = 0, steering = 0, pitch = 0, roll = 0;

    if (request->hasParam("throttle")) throttle = request->getParam("throttle")->value().toFloat();
    if (request->hasParam("steering")) steering = request->getParam("steering")->value().toFloat();
    if (request->hasParam("pitch")) pitch = request->getParam("pitch")->value().toFloat();
    if (request->hasParam("roll")) roll = request->getParam("roll")->value().toFloat();
    if (request->hasParam("t")) throttle = request->getParam("t")->value().toFloat();
    if (request->hasParam("s")) steering = request->getParam("s")->value().toFloat();

    setControlInput(throttle, steering, pitch, roll);
    sendSuccess(request, "Moving");
}

void RobotWebServer::handleStop(AsyncWebServerRequest *request) {
    setControlInput(0, 0, 0, 0);
    sendSuccess(request, "Stopped");
}

void RobotWebServer::handleForward(AsyncWebServerRequest *request) {
    float speed = 0.5f;
    if (request->hasParam("speed")) {
        speed = request->getParam("speed")->value().toFloat() / 255.0f;
    }
    setControlInput(speed, 0, 0, 0);
    sendSuccess(request, "Moving forward");
}

void RobotWebServer::handleBackward(AsyncWebServerRequest *request) {
    float speed = 0.5f;
    if (request->hasParam("speed")) {
        speed = request->getParam("speed")->value().toFloat() / 255.0f;
    }
    setControlInput(-speed, 0, 0, 0);
    sendSuccess(request, "Moving backward");
}

void RobotWebServer::handleRotate(AsyncWebServerRequest *request) {
    String dir = "left";
    float speed = 0.5f;

    if (request->hasParam("direction")) {
        dir = request->getParam("direction")->value();
    }
    if (request->hasParam("speed")) {
        speed = request->getParam("speed")->value().toFloat() / 255.0f;
    }

    float steering = (dir == "right") ? speed : -speed;
    setControlInput(0, steering, 0, 0);

    sendSuccess(request, ("Rotating " + dir).c_str());
}

void RobotWebServer::handleStatus(AsyncWebServerRequest *request) {
    JsonDocument doc;

    ControlInput input = getControlInput();

    doc["throttle"] = input.throttle;
    doc["steering"] = input.steering;
    doc["pitch"] = input.pitch;
    doc["roll"] = input.roll;
    doc["armed"] = input.armed;
    doc["vehicle"] = mixer.getVehicleType();
    doc["uptime"] = millis() / 1000;
    doc["heap"] = ESP.getFreeHeap();

    JsonArray channels = doc["channels"].to<JsonArray>();
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (outputChannels.isChannelEnabled(i)) {
            JsonObject ch = channels.add<JsonObject>();
            ch["id"] = i;
            ch["value"] = outputChannels.getChannelValue(i);
        }
    }

    sendJson(request, doc);
}

void RobotWebServer::handleCommand(AsyncWebServerRequest *request) {
    if (!request->hasParam("command") && !request->hasParam("cmd")) {
        sendError(request, "Missing command parameter");
        return;
    }

    String cmd = request->hasParam("command")
        ? request->getParam("command")->value()
        : request->getParam("cmd")->value();

    cmd.toLowerCase();

    if (cmd.indexOf("forward") >= 0 || cmd.indexOf("ahead") >= 0) {
        setControlInput(0.7f, 0, 0, 0);
        sendSuccess(request, "Moving forward");
    }
    else if (cmd.indexOf("backward") >= 0 || cmd.indexOf("reverse") >= 0 || cmd.indexOf("back") >= 0) {
        setControlInput(-0.7f, 0, 0, 0);
        sendSuccess(request, "Moving backward");
    }
    else if (cmd.indexOf("left") >= 0 && cmd.indexOf("rotate") >= 0) {
        setControlInput(0, -0.6f, 0, 0);
        sendSuccess(request, "Rotating left");
    }
    else if (cmd.indexOf("right") >= 0 && cmd.indexOf("rotate") >= 0) {
        setControlInput(0, 0.6f, 0, 0);
        sendSuccess(request, "Rotating right");
    }
    else if (cmd.indexOf("left") >= 0) {
        setControlInput(0.5f, -0.5f, 0, 0);
        sendSuccess(request, "Turning left");
    }
    else if (cmd.indexOf("right") >= 0) {
        setControlInput(0.5f, 0.5f, 0, 0);
        sendSuccess(request, "Turning right");
    }
    else if (cmd.indexOf("stop") >= 0 || cmd.indexOf("halt") >= 0) {
        setControlInput(0, 0, 0, 0);
        sendSuccess(request, "Stopped");
    }
    else if (cmd.indexOf("arm") >= 0) {
        armMotors();
        sendSuccess(request, "Armed");
    }
    else if (cmd.indexOf("disarm") >= 0) {
        disarmMotors();
        sendSuccess(request, "Disarmed");
    }
    else {
        sendError(request, "Unknown command");
    }
}

void RobotWebServer::onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected\n", client->id());
            break;

        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            setControlInput(0, 0, 0, 0);  // Safety: stop on disconnect
            break;

        case WS_EVT_DATA: {
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, data, len);

            if (!error) {
                float throttle = doc["throttle"] | 0.0f;
                float steering = doc["steering"] | 0.0f;
                float pitch = doc["pitch"] | 0.0f;
                float roll = doc["roll"] | 0.0f;

                setControlInput(throttle, steering, pitch, roll);
            }
            break;
        }

        default:
            break;
    }
}

void RobotWebServer::broadcastTelemetry() {
    if (_ws.count() == 0) return;

    JsonDocument doc;

    doc["ch0"] = outputChannels.getChannelValue(0);
    doc["ch1"] = outputChannels.getChannelValue(1);
    doc["ch2"] = outputChannels.getChannelValue(2);
    doc["ch3"] = outputChannels.getChannelValue(3);
    doc["vehicle"] = mixer.getVehicleType();
    doc["armed"] = areMotorsArmed();
    doc["uptime"] = millis() / 1000;

    String json;
    serializeJson(doc, json);
    _ws.textAll(json);
}

void RobotWebServer::sendJson(AsyncWebServerRequest *request, JsonDocument &doc, int code) {
    String json;
    serializeJson(doc, json);
    request->send(code, "application/json", json);
}

void RobotWebServer::sendSuccess(AsyncWebServerRequest *request, const char *message) {
    JsonDocument doc;
    doc["success"] = true;
    doc["message"] = message;
    sendJson(request, doc);
}

void RobotWebServer::sendError(AsyncWebServerRequest *request, const char *message, int code) {
    JsonDocument doc;
    doc["success"] = false;
    doc["error"] = message;
    sendJson(request, doc, code);
}
