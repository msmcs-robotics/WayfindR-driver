# MCP + Ollama Integration Research

**Date:** 2026-03-26
**Purpose:** Evaluate how to connect FastMCP tools (motor control) with Ollama (llama3.2:3b) on Jetson Orin Nano.

---

## Key Question: Can Ollama Use MCP Tools Directly?

**No.** Ollama does not natively speak MCP protocol. A wrapper/bridge layer is required.

Ollama has its own **tool calling API** (function calling) that works with compatible models. The pattern is:

1. Define tools as JSON schemas
2. Pass them to `ollama.chat()` in the `tools` parameter
3. Model returns `tool_calls` in its response
4. Your code executes the tool and sends results back
5. Model generates final answer

MCP is a separate protocol. To connect them, you need a **client that translates MCP tool definitions into Ollama's tool format** and orchestrates the call loop.

---

## Architecture Pattern: MCP Server + Ollama

```
User Input
    |
    v
Python Client (agent loop)
    |
    +--> Ollama API (with tool schemas)
    |        |
    |        v
    |    Model decides: call tool or respond
    |        |
    |        v (tool_call)
    +--> MCP Server (FastMCP)
    |        |
    |        v
    |    Execute tool (e.g., move_motor)
    |        |
    |        v (result)
    +--> Send result back to Ollama
    |        |
    |        v
    |    Model generates final response
    |
    v
User Output
```

Three components:
- **FastMCP Server** -- exposes tools via `@mcp.tool()` decorators
- **Python Client** -- the bridge/agent loop
- **Ollama** -- local LLM with tool calling support

---

## Approach 1: Direct Ollama Tool Calling (No MCP)

The simplest approach. Skip MCP entirely and use Ollama's native tool calling API.

```python
import ollama

def move_motor(direction: str, speed: int) -> str:
    """Move robot motor. direction: forward/backward/left/right. speed: 0-100."""
    # actual motor control code here
    return f"Motor moving {direction} at speed {speed}"

response = ollama.chat(
    model="llama3.2:3b",
    messages=[{"role": "user", "content": "Move forward slowly"}],
    tools=[move_motor],  # Pass function directly, Ollama auto-generates schema
)

# Check for tool calls
if response["message"].get("tool_calls"):
    for call in response["message"]["tool_calls"]:
        name = call["function"]["name"]
        args = call["function"]["arguments"]
        # Execute the function
        result = move_motor(**args)
        # Feed result back to model for final response
```

**Pros:** Simple, no extra dependencies, no server process.
**Cons:** No standardized tool protocol, tighter coupling.

### Approach 2: FastMCP Server + Ollama Client Bridge

Use FastMCP to define tools, then a client that converts MCP tool definitions to Ollama format.

```python
# --- server.py (FastMCP) ---
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("robot-control")

@mcp.tool()
def move_motor(direction: str, speed: int) -> str:
    """Move the robot. direction: forward/backward/left/right. speed: 0-100."""
    # hardware control
    return f"Moving {direction} at {speed}%"

mcp.run(transport="stdio")  # or "sse" for HTTP
```

```python
# --- client.py (bridge) ---
# 1. Connect to MCP server, list tools
# 2. Convert MCP tool schemas to Ollama format:
ollama_tools = []
for tool in mcp_tools:
    ollama_tools.append({
        "type": "function",
        "function": {
            "name": tool.name,
            "description": tool.description,
            "parameters": tool.inputSchema,
        }
    })

# 3. Send to Ollama with tools
response = ollama.chat(model="llama3.2:3b", messages=msgs, tools=ollama_tools)

# 4. Execute tool calls via MCP client
# 5. Feed results back to Ollama
```

**Pros:** Standardized protocol, tools reusable by any MCP client (Claude Desktop, etc.).
**Cons:** Extra complexity, two processes to manage.

### Approach 3: Existing Wrappers

- **ollama-fastmcp-wrapper** ([GitHub](https://github.com/andreamoro/ollama-fastmcp-wrapper)) -- Proxy that sits in front of Ollama, pre-loads MCP servers at startup, injects tools into every chat request. Supports HTTP and STDIO transports.
- **ollama-mcp-bridge** ([GitHub](https://github.com/patruff/ollama-mcp-bridge)) -- TypeScript bridge connecting Ollama to MCP servers.
- **mcp-client-for-ollama** ([PyPI](https://pypi.org/project/mcp-client-for-ollama/)) -- Python package for Ollama + MCP integration.
- **mcp-use** library (used with LangChain's `ChatOllama`) -- `MCPAgent` class orchestrates the loop.

---

## Ollama Tool Calling: Key Details

### Supported Models (for tool/function calling)

| Model | Size | Tool Calling | Notes |
|-------|------|-------------|-------|
| qwen2.5 | 7B+ | Good | Recommended by Ollama docs |
| qwen3 | 8B+ | Good | Used in latest Ollama examples |
| llama3.1 | 8B+ | Good | 89% accuracy benchmark |
| llama3.2 | 3B | Works but unreliable | Fine-tuned for tools, but small size causes format errors |
| mistral | 7B+ | Good | 85% accuracy benchmark |
| llama3.1 | 70B | Best | 94% accuracy benchmark |

### llama3.2:3b Specifically

- **Officially supports tool calling** -- Meta fine-tuned it for single, nested, parallel, and multi-turn function calling.
- **BUT reliability is a concern at 3B size.** vLLM docs flag it with a warning. LangChain forum reports format errors. Smaller models frequently fail to emit tool calls in the correct format.
- **Mitigation strategies:**
  - Keep tool schemas simple (few parameters, clear descriptions)
  - Validate tool call output format before executing
  - Use retry logic for malformed tool calls
  - Consider upgrading to qwen2.5:7b if tool calling is critical
  - Explicit system prompt: "You have access to these tools. Use them when appropriate."

### Tool Schema Format (Ollama)

```python
tools = [{
    "type": "function",
    "function": {
        "name": "move_motor",
        "description": "Move the robot in a direction at a given speed",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {
                    "type": "string",
                    "description": "Movement direction: forward, backward, left, right"
                },
                "speed": {
                    "type": "integer",
                    "description": "Speed percentage 0-100"
                }
            },
            "required": ["direction", "speed"]
        }
    }
}]
```

### Agent Loop Pattern

```python
import ollama
import json

messages = [{"role": "user", "content": user_input}]

while True:
    response = ollama.chat(
        model="llama3.2:3b",
        messages=messages,
        tools=tools,
    )

    msg = response["message"]
    messages.append(msg)

    if not msg.get("tool_calls"):
        # No more tool calls -- final answer
        print(msg["content"])
        break

    for tool_call in msg["tool_calls"]:
        name = tool_call["function"]["name"]
        args = tool_call["function"]["arguments"]

        # GOTCHA: args may be a JSON string, not dict
        if isinstance(args, str):
            args = json.loads(args)

        result = execute_tool(name, args)

        messages.append({
            "role": "tool",
            "content": json.dumps(result) if isinstance(result, dict) else str(result),
        })
```

---

## MCP + Robotics: Existing Projects

### Robot Control MCP Servers

1. **robot-mcp** ([GitHub](https://github.com/monteslu/robot-mcp)) -- Node.js MCP server using Johnny-Five for Arduino. Exposes `moveMyServo` tool (0-180 degrees). Simple but proves the pattern works.

2. **choturobo** ([GitHub](https://github.com/vishalmysore/choturobo)) -- Arduino robot controlled by Claude AI via MCP. ESP32/Arduino Nano with NodeMCU.

3. **phosphobot MCP** ([docs](https://docs.phospho.ai/examples/mcp-for-robotics)) -- Controls robots with LLMs using MCP protocol.

4. **ROS2 MCP Servers** -- Multiple projects bridging MCP to ROS2 for robot arm control (SO-ARM100, LeKiwi).

### FastMCP + GPIO Pattern (ESP32 Example)

From the ESP32 GPIO example, the pattern for hardware control tools:

```python
@mcp.tool()
def set_gpio_pin(pin: int, state: str) -> str:
    """Set a GPIO pin high or low. pin: GPIO number. state: 'on' or 'off'."""
    SAFE_PINS = [2, 4, 5, 12, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33]
    if pin not in SAFE_PINS:
        return f"Error: pin {pin} not in safe list"
    # actual hardware control
    return f"Pin {pin} set to {state}"

@mcp.tool()
def set_pwm(pin: int, duty: int) -> str:
    """Set PWM duty cycle on a pin. duty: 0-255."""
    # motor speed control
    return f"PWM on pin {pin} set to {duty}"
```

Key patterns:
- **Pin validation** -- whitelist safe GPIO pins
- **PWM for motor speed** -- duty cycle 0-255 maps to motor speed
- **Status queries** -- tools to read current pin state
- **Error handling** -- return error strings, never crash

---

## Recommendations for AMBOT

### Recommended Architecture

**Start with Approach 1 (Direct Ollama tool calling)** for simplicity:

1. Define motor control functions in Python with type hints and docstrings
2. Pass them directly to `ollama.chat()` as tools
3. Implement the agent loop in the existing `chat.py` or a new controller
4. Hardware functions call into `demos_common/robot.py` (RobotAdapter)

Later, if MCP interoperability is needed (e.g., Claude Desktop controlling the robot), wrap the same functions in a FastMCP server.

### Tool Design for llama3.2:3b

Keep tools **simple and few** to maximize reliability with the 3B model:

- `move(direction, speed)` -- forward/backward/left/right at speed%
- `stop()` -- halt all motors
- `turn(degrees)` -- rotate in place
- `get_status()` -- return sensor readings (LiDAR, IMU)

Avoid complex nested parameters. Use string enums and integers.

### Fallback: qwen2.5:7b

If llama3.2:3b proves too unreliable for tool calling, **qwen2.5:7b** is the recommended upgrade. It fits in the Jetson's 7.4 GiB unified memory (barely -- may need quantization) and has much better tool calling accuracy.

### Key Dependencies

```
ollama (Python SDK)
fastmcp >= 2.8.1 (if using MCP server approach)
```

---

## Local vs Cloud Model Tool Calling Comparison

| Aspect | Cloud (Claude, GPT-4) | Local (Ollama) |
|--------|----------------------|----------------|
| Tool calling reliability | Very high | Model-dependent; 3B models unreliable |
| Latency | Network round-trip | Local, fast on GPU |
| MCP support | Native (Claude Desktop) | Requires bridge/wrapper |
| Cost | Per-token pricing | Free after hardware |
| Privacy | Data leaves device | Fully local |
| Tool schema format | MCP native | Ollama JSON schema (different from MCP) |

The key difference: **Cloud models speak MCP natively. Ollama needs a translation layer.** The translation is straightforward (convert MCP inputSchema to Ollama tool format) but adds a component to maintain.

---

## Sources

- [FastMCP + ESP32 GPIO Control](https://medium.com/@jayaprakash.j/control-esp32-gpio-pins-with-fastmcp-building-smart-iot-solutions-37043bba1258)
- [Robot Control MCP Server (johnny-five)](https://github.com/monteslu/robot-mcp)
- [Arduino Robot Controlled by Claude AI MCP](https://dev.to/vishalmysore/arduino-robot-controlled-by-claude-ai-mcp-2fja)
- [choturobo - Arduino + MCP](https://github.com/vishalmysore/choturobo)
- [MCP in Robotics Overview](https://medium.com/@AdithyaMS/application-of-model-context-protocol-mcp-in-robotics-a09a2eb8f9eb)
- [phosphobot MCP for Robotics](https://docs.phospho.ai/examples/mcp-for-robotics)
- [Building MCP Server+Client with Ollama](https://medium.com/@smrati.katiyar/building-mcp-server-and-client-in-python-and-using-ollama-as-llm-provider-dd79fe3a2b16)
- [ollama-fastmcp-wrapper](https://github.com/andreamoro/ollama-fastmcp-wrapper)
- [ollama-mcp-bridge](https://github.com/patruff/ollama-mcp-bridge)
- [mcp-client-for-ollama on PyPI](https://pypi.org/project/mcp-client-for-ollama/)
- [MCP + Ollama Tool Calling Guide](https://dev.to/ajitkumar/building-your-first-agentic-ai-complete-guide-to-mcp-ollama-tool-calling-2o8g)
- [FastMCP + Ollama Implementation](https://www.c-sharpcorner.com/article/implementation-of-mcp-using-fastmcp-and-ollama-llm/)
- [Ollama Tool Calling Docs](https://docs.ollama.com/capabilities/tool-calling)
- [Ollama Tool Support Blog](https://ollama.com/blog/tool-support)
- [Best Ollama Models for Function Calling 2025](https://collabnix.com/best-ollama-models-for-function-calling-tools-complete-guide-2025/)
- [Ollama Tool Calling in 5 Lines of Python](https://dev.to/0coceo/ollama-tool-calling-in-5-lines-of-python-3h5f)
- [Tool Calling with Llama 3.2](https://medium.com/@stephan.pirner93/tool-calling-with-llama-3-2-23e3d783a6d8)
- [LangChain Forum: Llama-3.2-3B Tool Calling Issues](https://forum.langchain.com/t/tool-function-calling-with-llama-3-2-3b-instruct-model-local/2574)
- [Ollama Streaming Tool Calls](https://ollama.com/blog/streaming-tool)
- [ROS2 Robot Control MCP](https://www.pulsemcp.com/servers/kakimochi-ros2-robot-control)
