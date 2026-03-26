# AMBOT Motor Control MCP Server

FastMCP server that lets an LLM control AMBOT's motors via tool calls.

## Quick Start

```bash
# From ambot/ directory

# Simulate mode (no hardware needed), stdio transport
python -m mcp_ability.run

# Simulate mode, SSE transport on port 8100
python -m mcp_ability.run --transport sse --port 8100

# Real hardware on RPi (L298N driver)
python -m mcp_ability.run --no-simulate --transport sse --port 8100
```

## Tools

| Tool | Description |
|------|-------------|
| `move_forward(speed, duration)` | Drive forward. Speed 0-100, duration in seconds (0 = continuous). |
| `move_backward(speed, duration)` | Drive backward. |
| `turn_left(speed, duration)` | Spin left (counterclockwise). |
| `turn_right(speed, duration)` | Spin right (clockwise). |
| `stop_motors()` | Stop all motors immediately. |
| `get_motor_status()` | Get current speeds, direction, uptime. |

## Safety

- **Auto-stop watchdog**: Motors stop after 30 seconds with no new command (configurable via `--auto-stop`).
- **Duration timer**: `duration > 0` runs for exactly N seconds then stops.
- **Thread-safe**: All motor access goes through a lock.

## Connect from Claude Desktop

Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "ambot-motors": {
      "command": "python",
      "args": ["-m", "mcp_ability.run"],
      "cwd": "/path/to/ambot"
    }
  }
}
```

## Connect from Ollama (via SSE)

Start the server with SSE transport:

```bash
python -m mcp_ability.run --transport sse --port 8100
```

Then point your Ollama chat loop at `http://<host>:8100/sse` to discover tools.

## Example Conversation

```
User: Move forward slowly for 3 seconds.
LLM:  [calls move_forward(speed=30, duration=3)]
      -> Moving forward at speed 30 for 3.0 seconds.

User: Turn right.
LLM:  [calls turn_right(speed=50)]
      -> Turning right at speed 50, continuous until stopped.

User: Stop!
LLM:  [calls stop_motors()]
      -> Motors stopped.

User: What are the motors doing?
LLM:  [calls get_motor_status()]
      -> Stopped. Idle 5.2s. 3 commands issued. Uptime 42.1s.
```

## Options

```
--simulate        Log commands only (default)
--no-simulate     Use real L298N hardware
--transport       stdio (default) or sse
--host            SSE bind address (default 0.0.0.0)
--port            SSE port (default 8100)
--auto-stop       Watchdog timeout in seconds (default 30, 0 to disable)
--log-level       DEBUG, INFO, WARNING, ERROR
```
