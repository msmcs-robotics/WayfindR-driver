# Laptop API Demo - Pi Control Chat

A FastAPI web application that provides a chat interface to control a Raspberry Pi using an LLM (Ollama). The LLM's responses are automatically spoken through the Pi's speakers.

## Features

- **Web-based Chat Interface** - Modern chat UI accessible via browser at port 10000
- **LLM Integration** - Uses Ollama with llama3:8b model
- **LED Control** - Control an LED on the Raspberry Pi through natural language
- **Auto Text-to-Speech** - ALL LLM responses are automatically spoken through the Pi's 3.5mm AUX output
- **Real-time Updates** - WebSocket-based communication for instant feedback

## System Architecture

This is **NOT MCP** - it uses a custom hardcoded tool-calling pattern with HTTP REST APIs.

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              LAPTOP (localhost)                                  │
│                                                                                  │
│  ┌─────────────────┐         ┌─────────────────────────────────────────────┐    │
│  │   Web Browser   │◄───────►│         laptop_api_demo/app.py              │    │
│  │ localhost:10000 │ WebSocket│            (FastAPI Server)                 │    │
│  │                 │         │                                              │    │
│  │  User types:    │         │  1. Receives user message via WebSocket     │    │
│  │  "Turn on LED"  │         │  2. Sends to Ollama with SYSTEM_PROMPT      │    │
│  └─────────────────┘         │  3. Parses LLM response for [TOOL:xxx]      │    │
│                              │  4. Makes HTTP requests to Pi API           │    │
│                              │  5. Auto-speaks response via Pi TTS         │    │
│                              │  6. Sends response back to browser          │    │
│                              └──────────────┬──────────────────────────────┘    │
│                                             │                                    │
│                                             │ HTTP POST /api/generate            │
│                                             ▼                                    │
│                              ┌─────────────────────────────────────────────┐    │
│                              │              Ollama                          │    │
│                              │         localhost:11434                      │    │
│                              │                                              │    │
│                              │  Model: llama3:8b                           │    │
│                              │  System Prompt teaches it to output:         │    │
│                              │    [TOOL:led_on], [TOOL:led_off], etc.      │    │
│                              └─────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────────┘
                                             │
                                             │ HTTP REST calls
                                             │ POST /led/on, POST /speak, etc.
                                             ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI (169.254.123.217)                          │
│                                                                                  │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                    pi_api_demo/main.py                                   │    │
│  │                    (FastAPI Server on :8000)                             │    │
│  │                                                                          │    │
│  │   Endpoints:                                                             │    │
│  │   POST /led/on   ──► GPIO.output(21, HIGH) ──► LED turns ON             │    │
│  │   POST /led/off  ──► GPIO.output(21, LOW)  ──► LED turns OFF            │    │
│  │   POST /speak    ──► espeak-ng + sox + aplay ──► Audio out AUX jack     │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                              │                        │                          │
│                              ▼                        ▼                          │
│                         ┌────────┐            ┌────────────┐                    │
│                         │  LED   │            │  Speaker   │                    │
│                         │ GPIO21 │            │ (3.5mm AUX)│                    │
│                         └────────┘            └────────────┘                    │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Message Flow Example: "Turn on the LED"

```
Step 1: User types "Turn on the LED" in browser
        ↓
Step 2: Browser sends via WebSocket to app.py
        {"type": "message", "text": "Turn on the LED"}
        ↓
Step 3: app.py calls chat_with_ollama()
        POST http://localhost:11434/api/generate
        {
          "model": "llama3:8b",
          "prompt": "User: Turn on the LED",
          "system": "You are a helpful assistant... use [TOOL:led_on]..."
        }
        ↓
Step 4: Ollama returns:
        "I'll turn on the LED now. [TOOL:led_on]"
        ↓
Step 5: app.py calls process_response()
        - Regex finds: [TOOL:led_on]
        - Calls execute_tool("led_on")
          → POST http://169.254.123.217:8000/led/on
        - Calls speak_text("I'll turn on the LED now.")
          → POST http://169.254.123.217:8000/speak
        ↓
Step 6: Pi API receives requests:
        - /led/on → GPIO.output(21, HIGH) → LED lights up
        - /speak  → espeak-ng → sox → aplay → Audio plays through AUX
        ↓
Step 7: app.py sends response to browser:
        {"type": "response", "text": "I'll turn on the LED now.",
         "tool_results": ["LED turned ON", "Speaking..."]}
```

## This is NOT MCP

| What we have | What MCP would be |
|--------------|-------------------|
| System prompt teaches LLM to output `[TOOL:xxx]` | Formal tool definitions via MCP protocol |
| Regex parsing: `r'\[TOOL:(\w+)\]'` | MCP client handles tool calls automatically |
| Hardcoded HTTP calls in `execute_tool()` | MCP server exposes tools dynamically |
| Manual wiring of everything | Standardized protocol with discovery |

The current approach is simpler but less flexible. MCP would allow dynamic tool discovery and a standardized interface.

## Setup

### Prerequisites
- Python 3.8+
- Ollama running locally with llama3:8b model
- Pi API Demo running on Raspberry Pi at 169.254.123.217:8000

### Installation

1. Use the project virtual environment:
   ```bash
   cd /home/devel/WayfindR-driver
   source venv/bin/activate
   ```

2. Install dependencies (if not already):
   ```bash
   pip install -r laptop_api_demo/requirements.txt
   ```

### Configuration

Edit `app.py` to configure:
```python
OLLAMA_BASE_URL = "http://localhost:11434"  # Ollama API URL
MODEL_NAME = "llama3:8b"                     # LLM model to use
PI_API_URL = "http://169.254.123.217:8000"   # Raspberry Pi API URL
PORT = 10000                                  # Web server port
```

### Running

```bash
cd /home/devel/WayfindR-driver/laptop_api_demo
source ../venv/bin/activate
python app.py
```

Then open http://localhost:10000 in your browser.

## Usage

### Web Interface

The web interface provides:
- Chat input for natural language commands
- Quick action buttons for common operations
- Status indicators for Ollama, Pi, and WebSocket connections
- Chat history display

### Example Commands

**LED Control:**
- "Turn on the LED"
- "Switch off the light"
- "What's the LED status?"
- "Toggle the LED"

**General Chat (all responses are spoken):**
- "Tell me a joke" - LLM responds AND speaks through Pi
- "What time is it?" - Response is automatically spoken
- Any conversation - Everything is spoken aloud

### Quick Action Buttons

- **LED On** - Turn the LED on
- **LED Off** - Turn the LED off
- **LED Status** - Check current LED state
- **Say Hello** - Have the Pi say hello
- **Tell Joke** - Have the Pi tell a joke
- **Clear Chat** - Clear conversation history

## API Endpoints

- `GET /` - Serve the chat HTML page
- `GET /health` - Health check (Ollama and Pi status)
- `WebSocket /ws` - Real-time chat communication

## How It Works

### Tool System (Prompt Engineering)

The LLM is taught via SYSTEM_PROMPT to output tool commands in a specific format:

```python
SYSTEM_PROMPT = """You are a helpful assistant that can control a Raspberry Pi with an LED.

You have access to these LED commands. When the user asks you to control the LED,
respond with the appropriate command in this EXACT format:

[TOOL:led_on] - Turn the LED on
[TOOL:led_off] - Turn the LED off
[TOOL:led_toggle] - Toggle the LED state
[TOOL:led_status] - Check if the LED is on or off
...
"""
```

### Tool Parsing (Regex)

The `process_response()` function uses regex to find tool commands:

```python
tool_pattern = r'\[TOOL:(\w+)\]'
tool_matches = re.findall(tool_pattern, response)
```

### Auto TTS

Every LLM response is automatically spoken through the Pi:

```python
# ALWAYS speak the cleaned response through the Pi
if cleaned:
    speak_result = speak_text(cleaned)
    tool_results.append(speak_result)
```

## Troubleshooting

### Ollama Connection Failed
- Ensure Ollama is running: `curl http://localhost:11434/api/tags`
- Check model is available: `ollama list` should show `llama3:8b`
- If using HPC port forwarding: `ssh -L 11434:localhost:11434 user@hpc`

### Pi Connection Failed
- Check Pi is accessible: `ping 169.254.123.217`
- Verify Pi API is running: `curl http://169.254.123.217:8000/health`
- Restart Pi API: `ssh dnichols4@169.254.123.217 "cd ~/Desktop/pi_api_demo && python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 &"`

### No Audio from Pi
- Check speaker is connected to 3.5mm AUX jack
- Verify volume: `ssh dnichols4@169.254.123.217 "amixer -c 0 set PCM 100%"`
- Test directly: `ssh dnichols4@169.254.123.217 "speaker-test -D plughw:0,0 -c 2 -t sine -f 440 -l 1"`

### WebSocket Disconnects
- Check browser console for errors
- The app auto-reconnects after 3 seconds

## Project Structure

```
laptop_api_demo/
├── app.py              # Main FastAPI application (all-in-one)
├── requirements.txt    # Python dependencies
├── docs/
│   └── README.md       # This documentation
└── test_system.py      # System tests
```

## Adding New Tools

1. Add tool description to `SYSTEM_PROMPT` in `app.py`
2. Add handler in `execute_tool()` function
3. Add corresponding endpoint to Pi API if needed
4. The regex `[TOOL:tool_name]` pattern will auto-detect it

## Dependencies

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
requests>=2.31.0
websockets>=12.0
```
