# WayfindR-driver Demos Folder

## Overview

This folder contains demonstration code and examples for various hardware components and systems used in the WayfindR project. The demos range from basic hardware testing to complete AI-powered control systems, covering RC receivers, motor drivers, GPIO control, and voice AI integration.

## Available Demos

### 1. FlySky FS-i6B RC Receiver Demos

#### fsia6b_basic_PWM
**Purpose:** Basic test sketch for the FlySky FS-i6B RC receiver
**Hardware:** Arduino (tested on UNO), FS-i6B receiver
**File:** `fsia6b_basic_PWM/fsia6b_basic_PWM.ino`

**Features:**
- Reads right joystick PWM signals (horizontal and vertical)
- CH1 (Pin 3): Right stick horizontal (steering) - uses interrupt
- CH2 (Pin 5): Right stick vertical (throttle) - uses pulseIn
- LED indicator shows signal activity
- Serial output displays stick position and values

**Hardware Connections:**
- CH1 signal → Arduino Pin 3
- CH2 signal → Arduino Pin 5
- 5V → Arduino 5V
- GND → Arduino GND

**How to Run:**
1. Connect FS-i6B receiver to Arduino as specified
2. Bind receiver on Model 1 with PPM output OFF
3. Upload sketch to Arduino
4. Open Serial Monitor at 9600 baud
5. Move right joystick to see values (1000-2000μs, center ~1500μs)

**Expected Behavior:**
- Center position: Both channels ~1500μs
- Stick movements: 1000-2000μs range
- LED on when signal active, off when transmitter off

#### fsia6b_UNO_skidsteer
**Purpose:** Full RC car differential drive controller using FS-i6B receiver
**Hardware:** Arduino UNO, FS-i6B receiver, dual L298N motor drivers, 4 motors
**File:** `fsia6b_UNO_skidsteer/fsia6b_UNO_skidsteer.ino`

**Features:**
- Differential drive control (skid-steer)
- Left and right motor groups controlled independently
- Turning logic: joystick direction correctly maps to vehicle turns
- Deadzone filtering and configurable sensitivity
- Status LED on Pin A1
- Supports both normal turns and pivot turns

**Hardware Connections:**
- FS-i6B Receiver:
  - CH1 → Pin 3 (Steering)
  - CH2 → Pin 5 (Throttle)
  - 5V/GND to Arduino

- LEFT L298N Driver:
  - IN1 → Pin 6 (Left Front PWM)
  - IN2 → Pin 7 (Left Front Direction)
  - IN3 → Pin 8 (Left Rear PWM)
  - IN4 → Pin 9 (Left Rear Direction)

- RIGHT L298N Driver:
  - IN1 → Pin 10 (Right Front PWM)
  - IN2 → Pin 11 (Right Front Direction)
  - IN3 → Pin 12 (Right Rear PWM)
  - IN4 → Pin 13 (Right Rear Direction)

- Status LED → Pin A1

**Important:** All components must share common ground

**How to Run:**
1. Wire all components as specified
2. Ensure L298N ENA/ENB jumpers are in place
3. Upload sketch to Arduino
4. Power on transmitter and move sticks
5. Vehicle should respond to joystick inputs

**Configuration:**
- `DEADZONE = 50` - Center deadzone
- `TURN_SENSITIVITY = 0.8` - Turning sensitivity
- `MAX_SPEED = 255` - Maximum motor PWM

---

### 2. Raspberry Pi GPIO Differential Drive

**Purpose:** Python-based differential drive control using Raspberry Pi GPIO
**Hardware:** Raspberry Pi, motor driver
**File:** `pi_GPIO_diff_drive/main.py`

**Status:** STUB FILE - Contains only a single line, not implemented
**Implementation State:** Placeholder only

---

### 3. Voice AI + Pi LED Control System

This is a complete multi-component system demonstrating AI-controlled Raspberry Pi with LED control and text-to-speech capabilities.

#### System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         LAPTOP (localhost)                               │
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────┐       │
│  │           laptop_api_demo/app.py (FastAPI)                   │       │
│  │              Web Interface on :10000                          │       │
│  │                                                               │       │
│  │  • WebSocket chat interface                                  │       │
│  │  • Connects to Ollama (llama3:8b) on :11434                 │       │
│  │  • Parses [TOOL:xxx] commands from LLM responses            │       │
│  │  • Makes HTTP calls to Pi API                                │       │
│  │  • Auto-speaks ALL LLM responses via Pi TTS                  │       │
│  └───────────────────────┬──────────────────────────────────────┘       │
│                          │                                               │
│                          │ HTTP REST API                                 │
│                          ▼                                               │
└─────────────────────────────────────────────────────────────────────────┘
                           │
                           │
┌──────────────────────────┼──────────────────────────────────────────────┐
│         RASPBERRY PI (169.254.123.217)                                   │
│                          │                                               │
│  ┌───────────────────────▼──────────────────────────────────────┐       │
│  │           pi_api_demo/main.py (FastAPI)                      │       │
│  │              API Server on :8000                              │       │
│  │                                                               │       │
│  │  Endpoints:                                                   │       │
│  │  • POST /led/on, /led/off, /led/toggle - LED control        │       │
│  │  • GET /led - Get LED state                                  │       │
│  │  • POST /speak - Text-to-speech via 3.5mm AUX               │       │
│  │  • GET /health - System status                               │       │
│  └─────────────┬─────────────────────┬──────────────────────────┘       │
│                │                     │                                   │
│                ▼                     ▼                                   │
│           ┌────────┐         ┌──────────────┐                          │
│           │  LED   │         │   Speaker    │                          │
│           │ GPIO21 │         │ (3.5mm AUX)  │                          │
│           └────────┘         └──────────────┘                          │
└─────────────────────────────────────────────────────────────────────────┘
```

#### Component A: pi_api_demo

**Purpose:** FastAPI server running on Raspberry Pi for LED control and TTS
**Location:** `voice_AI_pi_led/pi_api_demo/`
**Main File:** `main.py`

**Features:**
- LED control via GPIO 21 (BCM numbering)
- Text-to-speech through 3.5mm AUX jack
- RESTful API with CORS support
- Mock mode when RPi.GPIO unavailable

**Hardware Requirements:**
- Raspberry Pi 4 (tested on Raspbian Bullseye)
- LED on GPIO 21 with 220Ω resistor to GND
- Speaker connected to 3.5mm headphone jack

**Software Dependencies:**
```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
RPi.GPIO>=0.7.1
pydantic>=2.0.0
espeak-ng
sox
alsa-utils
```

**TTS Pipeline:**
1. espeak-ng generates 22050 Hz mono WAV
2. sox converts to 48000 Hz stereo (required for bcm2835)
3. aplay plays through plughw:0,0 (3.5mm AUX)

**Audio Configuration:**
```bash
# Force audio to headphone jack
sudo raspi-config nonint do_audio 1

# Set volume to maximum
amixer -c 0 set PCM 100%
```

**How to Run:**
```bash
# Installation
cd voice_AI_pi_led/pi_api_demo
pip install -r requirements.txt
sudo apt-get install espeak-ng sox alsa-utils

# Run server
uvicorn main:app --host 0.0.0.0 --port 8000
# OR
python main.py

# Background mode
nohup python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/pi_api.log 2>&1 &
```

**API Examples:**
```bash
# LED control
curl -X POST http://169.254.123.217:8000/led/on
curl -X POST http://169.254.123.217:8000/led/off
curl http://169.254.123.217:8000/led

# Text-to-speech
curl -X POST http://169.254.123.217:8000/speak \
  -H "Content-Type: application/json" \
  -d '{"text": "Hello from Raspberry Pi"}'

# Health check
curl http://169.254.123.217:8000/health
```

**Network Configuration:**
- IP: 169.254.123.217 (link-local via USB/Ethernet)
- Port: 8000
- SSH: `ssh dnichols4@169.254.123.217`

#### Component B: laptop_api_demo

**Purpose:** Web-based chat interface for controlling Pi using Ollama LLM
**Location:** `voice_AI_pi_led/laptop_api_demo/`
**Main File:** `app.py`

**Features:**
- Modern web chat interface (HTML/CSS/JS embedded)
- WebSocket-based real-time communication
- Ollama integration (llama3:8b model)
- Custom tool-calling system via prompt engineering
- Auto text-to-speech for ALL LLM responses
- Status indicators for Ollama, Pi, and WebSocket

**Tool System (NOT MCP):**
The system uses custom regex-based tool parsing:
- LLM taught to output `[TOOL:led_on]`, `[TOOL:led_off]`, etc. via SYSTEM_PROMPT
- Regex pattern `r'\[TOOL:(\w+)\]'` extracts tool commands
- Hardcoded HTTP calls to Pi API in `execute_tool()`
- All LLM responses automatically spoken via Pi

**Available Tools:**
- `[TOOL:led_on]` - Turn LED on
- `[TOOL:led_off]` - Turn LED off
- `[TOOL:led_toggle]` - Toggle LED
- `[TOOL:led_status]` - Check LED status

**Software Dependencies:**
```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
requests>=2.31.0
websockets>=12.0
```

**Prerequisites:**
- Ollama running locally with llama3:8b model
- Pi API server running at 169.254.123.217:8000

**Configuration:**
Edit in `app.py`:
```python
OLLAMA_BASE_URL = "http://localhost:11434"
MODEL_NAME = "llama3:8b"
PI_API_URL = "http://169.254.123.217:8000"
PORT = 10000
```

**How to Run:**
```bash
# Install dependencies
cd voice_AI_pi_led/laptop_api_demo
pip install -r requirements.txt

# Ensure Ollama is running
curl http://localhost:11434/api/tags

# Run application
python app.py

# Access web interface
# Open browser to http://localhost:10000
```

**Usage Examples:**
- "Turn on the LED"
- "Switch off the light"
- "What's the LED status?"
- "Tell me a joke" (response is spoken through Pi)

**Quick Action Buttons:**
- LED On/Off/Status
- Say Hello
- Tell Joke
- Clear Chat

**Message Flow:**
1. User types message in browser
2. WebSocket sends to app.py
3. app.py queries Ollama with SYSTEM_PROMPT
4. LLM returns response with optional [TOOL:xxx] tags
5. app.py parses tools, executes via HTTP to Pi
6. app.py sends text to Pi TTS endpoint
7. Response shown in browser, spoken by Pi

---

### 4. Reference Documentation

**File:** `FlySky_IA6B_pinout.jpg`
**Purpose:** Visual pinout diagram for FlySky IA6B receiver
**Type:** JPEG image (30,993 bytes)

Provides wiring reference for connecting the receiver, showing:
- Power connections (B/VCC binding)
- PWM output pins arrangement
- Channel assignments
- Ground and voltage requirements

---

## Implementation Status Summary

| Demo | Status | Completeness | Hardware Required |
|------|--------|-------------|-------------------|
| fsia6b_basic_PWM | ✓ Complete | 100% | Arduino, FS-i6B |
| fsia6b_UNO_skidsteer | ✓ Complete | 100% | Arduino, FS-i6B, 2x L298N, motors |
| pi_GPIO_diff_drive | ✗ Stub only | 0% | Raspberry Pi, motor driver |
| voice_AI_pi_led/pi_api_demo | ✓ Complete | 100% | Raspberry Pi, LED, speaker |
| voice_AI_pi_led/laptop_api_demo | ✓ Complete | 100% | Laptop, Ollama |
| FlySky_IA6B_pinout.jpg | ✓ Reference | N/A | N/A |

---

## Dependencies & Requirements

### Arduino Demos (fsia6b_*)
- **Hardware:**
  - Arduino UNO or compatible
  - FlySky FS-i6B RC receiver
  - FlySky transmitter (FS-i6, FS-i6S, etc.)
  - For skidsteer: 2x L298N motor drivers, 4 motors, power supply

- **Software:**
  - Arduino IDE
  - No external libraries required (uses built-in Arduino functions)

- **Skills:**
  - Basic Arduino programming
  - PWM signal understanding
  - Motor driver wiring (for skidsteer)

### Raspberry Pi API Demo
- **Hardware:**
  - Raspberry Pi 4 (Raspbian Bullseye)
  - LED with 220Ω resistor
  - Speaker with 3.5mm jack
  - Power supply

- **Software:**
  ```bash
  # Python packages
  pip install fastapi uvicorn[standard] RPi.GPIO pydantic

  # System packages
  sudo apt-get install espeak-ng sox alsa-utils
  ```

- **Network:**
  - Pi accessible at 169.254.123.217:8000
  - SSH access for deployment

### Laptop API Demo
- **Hardware:**
  - Any laptop/desktop capable of running Python
  - Pi API must be accessible on network

- **Software:**
  ```bash
  pip install fastapi uvicorn[standard] requests websockets
  ```

- **External Services:**
  - Ollama with llama3:8b model
  - Running at localhost:11434

- **Skills:**
  - Python web development
  - Understanding of REST APIs
  - Basic LLM/prompt engineering concepts

---

## Quick Start Guide

### To test RC receiver:
```bash
cd demos/fsia6b_basic_PWM
# Upload fsia6b_basic_PWM.ino to Arduino
# Open Serial Monitor at 9600 baud
# Move right joystick on transmitter
```

### To run RC car:
```bash
cd demos/fsia6b_UNO_skidsteer
# Wire everything per comments in .ino file
# Upload fsia6b_UNO_skidsteer.ino to Arduino
# Power on transmitter and test driving
```

### To run Pi LED/TTS system:
```bash
# On Raspberry Pi:
cd demos/voice_AI_pi_led/pi_api_demo
pip install -r requirements.txt
sudo apt-get install espeak-ng sox alsa-utils
python main.py

# On laptop:
cd demos/voice_AI_pi_led/laptop_api_demo
pip install -r requirements.txt
# Ensure Ollama is running with llama3:8b
python app.py
# Open http://localhost:10000 in browser
```

---

## Troubleshooting

### Arduino Issues
- **No RC signal:** Check binding, ensure PPM is OFF in receiver settings
- **Erratic motor behavior:** Verify common ground between Arduino, drivers, and power
- **Motors don't respond:** Check ENA/ENB jumpers on L298N, verify power supply

### Raspberry Pi Issues
- **GPIO not available:** Install `RPi.GPIO`, check permissions
- **No audio:** Run `sudo raspi-config nonint do_audio 1`, set volume with `amixer`
- **TTS not working:** Install `espeak-ng` and `sox`, test audio device with `speaker-test`
- **API not accessible:** Check firewall, verify server is running with `ps aux | grep uvicorn`

### Laptop API Issues
- **Ollama connection failed:** Ensure Ollama running, check `curl http://localhost:11434/api/tags`
- **Pi connection failed:** Ping Pi, verify API running, check URL configuration
- **No audio from Pi:** Speaker connected to 3.5mm? Volume up? Test with `speaker-test`

---

## Future Development

Potential enhancements:
1. Complete `pi_GPIO_diff_drive` implementation
2. Add camera integration demos
3. GPS/navigation examples
4. Sensor integration (ultrasonic, IMU, etc.)
5. Convert laptop_api_demo to use proper MCP protocol
6. Add motor encoder feedback examples
7. Autonomous navigation demos

---

## Notes

- **NOT MCP:** The voice AI system uses custom tool parsing, not Model Context Protocol
- **Audio HAT:** RASPIAUDIO Ultra V3 HAT was tested but abandoned due to clock issues
- **IP Address:** Pi configured with link-local 169.254.123.217 for USB/Ethernet connection
- **RC Binding:** FS-i6B must be bound on B/VCC with battery power, Model 1 selected
- **Safety:** Always test motor systems with props/wheels removed initially

---

## Documentation Files

Additional documentation within subdirectories:
- `voice_AI_pi_led/pi_api_demo/README.md` - Pi API setup and usage
- `voice_AI_pi_led/pi_api_demo/docs/README.md` - Detailed Pi API documentation
- `voice_AI_pi_led/pi_api_demo/docs/AUDIO_SETUP.md` - Audio troubleshooting
- `voice_AI_pi_led/laptop_api_demo/README.md` - Laptop app overview
- `voice_AI_pi_led/laptop_api_demo/docs/README.md` - Detailed system architecture

---

## Contributing

When adding new demos:
1. Create a subdirectory with descriptive name
2. Include README.md with setup instructions
3. Document all hardware connections
4. List all dependencies
5. Provide example usage
6. Update this scope.md file

---

**Last Updated:** 2026-01-11
