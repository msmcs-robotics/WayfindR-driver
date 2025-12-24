# Laptop API Demo

Local application for your laptop that provides:
1. Chat interface to Ollama (llama3.3:70b via HPC port forwarding)
2. MCP server for controlling the Raspberry Pi LED

## Prerequisites

- Ollama port forwarded from HPC: `ssh -L 11434:localhost:11434 user@hpc-cluster`
- Raspberry Pi running the `pi_api_demo` API

## Installation

```bash
cd laptop_api_demo
pip install -r requirements.txt
```

## Components

### 1. Chat App (`chat_app.py`)

Interactive terminal chat with Ollama.

```bash
python chat_app.py
```

Commands:
- `/quit` or `/exit` - Exit chat
- `/models` - List available models
- `/clear` - Clear conversation history

### 2. MCP Server (`led_mcp_server.py`)

MCP server providing LED control tools.

```bash
# Run directly (stdio mode)
python led_mcp_server.py

# Or with fastmcp CLI
fastmcp run led_mcp_server.py
```

## Configuration

Edit the Pi API URL in `led_mcp_server.py`:

```python
PI_API_URL = "http://raspberrypi.local:8000"  # Or your Pi's IP
```

Edit Ollama settings in `chat_app.py`:

```python
OLLAMA_BASE_URL = "http://localhost:11434"
MODEL_NAME = "llama3.3:70b"
```

## MCP Tools Available

| Tool               | Description                    |
|--------------------|--------------------------------|
| turn_led_on        | Turn the LED on                |
| turn_led_off       | Turn the LED off               |
| toggle_led         | Toggle LED state               |
| get_led_status     | Get current LED state          |
| check_pi_connection| Check if Pi API is accessible  |

## Using with Claude Desktop

Add to your Claude Desktop config (`claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "led-controller": {
      "command": "python",
      "args": ["/path/to/laptop_api_demo/led_mcp_server.py"]
    }
  }
}
```

## Architecture

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│   Your Laptop   │         │  HPC Cluster    │         │  Raspberry Pi   │
│                 │         │                 │         │                 │
│  ┌───────────┐  │  SSH    │  ┌───────────┐  │         │  ┌───────────┐  │
│  │ Chat App  │──┼─Tunnel──┼──│  Ollama   │  │         │  │  Pi API   │  │
│  └───────────┘  │ :11434  │  │ llama3.3  │  │         │  │  :8000    │  │
│                 │         │  └───────────┘  │         │  └─────┬─────┘  │
│  ┌───────────┐  │         │                 │         │        │        │
│  │MCP Server │──┼─────────┼─────────────────┼─ HTTP ──┼────────┘        │
│  └───────────┘  │         │                 │         │     GPIO 14     │
│                 │         │                 │         │        │        │
└─────────────────┘         └─────────────────┘         │       LED       │
                                                        └─────────────────┘
```
