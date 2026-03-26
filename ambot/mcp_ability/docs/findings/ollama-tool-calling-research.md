# Ollama Tool/Function Calling Research

**Date:** 2026-03-26
**Purpose:** Evaluate Ollama tool calling for MCP motor integration on Jetson Orin Nano

---

## Key Findings Summary

1. **llama3.2:3b DOES support tool calling** -- the model page tags it with "tools" and Meta's docs confirm it was fine-tuned for tool use
2. The Ollama Python library auto-generates tool schemas from Python functions (type annotations + docstrings)
3. Tool responses come back as `response.message.tool_calls` with function name and arguments
4. Streaming tool calls are supported but non-streaming (`stream=false`) is simpler for parsing
5. The agent loop pattern (call -> tool -> call -> ...) is well-documented and straightforward

---

## Does llama3.2:3b Support Tool Calling?

**Yes**, with caveats.

- The [Ollama model page for llama3.2](https://ollama.com/library/llama3.2) explicitly lists "Tool use" as a capability and tags the model with "tools"
- Meta's [llama3.2 prompt format docs](https://github.com/meta-llama/llama-models/blob/main/models/llama3_2/text_prompt_format.md) confirm support for "zero shot function calling" with a new pythonic format
- Llama 3.2 supports single, nested, parallel, and multi-turn function calling
- The 3B model has 128K context length, multilingual support, and is optimized for "agentic retrieval and summarization tasks"

**Caveats:**
- The 3B model is small; users report ~80% success rate on tool calls (vs higher for 8B+ models)
- The [official tools model search page](https://ollama.com/search?c=tools) features newer models (qwen3, granite4, etc.) more prominently, but llama3.2 IS tagged as tools-capable on its own page
- Llama 3.2 uses a **pythonic format** for zero-shot function calling: `[func_name(param1='value1')]` -- but Ollama handles the translation to/from its API format, so this is transparent to us
- Some GitHub issues report occasional problems with tool support in various models; having a fallback or retry strategy is wise

**Recommendation for AMBOT:** llama3.2:3b should work for our use case (simple motor commands with 3-5 tools). The tool definitions are straightforward and the function signatures are simple. If reliability is an issue, consider upgrading to llama3.1:8b (but that requires more VRAM/RAM on the Jetson).

---

## Ollama API Format for Tools

### Request Format (HTTP API)

```json
POST /api/chat
{
  "model": "llama3.2:3b",
  "messages": [
    {"role": "user", "content": "Move the robot forward"}
  ],
  "tools": [
    {
      "type": "function",
      "function": {
        "name": "move_forward",
        "description": "Move the robot forward at a given speed",
        "parameters": {
          "type": "object",
          "properties": {
            "speed": {
              "type": "integer",
              "description": "Speed percentage from 0 to 100"
            },
            "duration": {
              "type": "number",
              "description": "Duration in seconds"
            }
          },
          "required": ["speed"]
        }
      }
    }
  ],
  "stream": false
}
```

### Response Format (with tool call)

```json
{
  "message": {
    "role": "assistant",
    "content": "",
    "tool_calls": [
      {
        "function": {
          "name": "move_forward",
          "arguments": {
            "speed": 50,
            "duration": 2.0
          }
        }
      }
    ]
  }
}
```

### Key Notes
- Set `"stream": false` for simpler tool call parsing (streaming is possible but adds complexity)
- When tools are provided, the model may return `tool_calls` instead of `content`
- Tool results are sent back as messages with `"role": "tool"`

---

## Python Implementation

### Approach 1: Pass Python Functions Directly (Recommended)

The Ollama Python library (v0.4+) auto-generates JSON schemas from Python functions using type annotations and Google-style docstrings.

```python
import ollama

def move_forward(speed: int, duration: float = 2.0) -> str:
    """Move the robot forward

    Args:
        speed: Speed percentage from 0 to 100
        duration: Duration in seconds (default 2.0)

    Returns:
        str: Status message
    """
    # actual motor control here
    return f"Moving forward at {speed}% for {duration}s"

def turn_left(angle: int) -> str:
    """Turn the robot left

    Args:
        angle: Angle in degrees to turn (0-180)

    Returns:
        str: Status message
    """
    return f"Turning left {angle} degrees"

def stop() -> str:
    """Stop all robot movement immediately

    Returns:
        str: Status message
    """
    return "Robot stopped"

# Map function names to callables
available_functions = {
    'move_forward': move_forward,
    'turn_left': turn_left,
    'stop': stop,
}

messages = [
    {'role': 'system', 'content': 'You are a robot controller. Use the available tools to execute movement commands.'},
    {'role': 'user', 'content': 'Go forward slowly for 3 seconds'}
]

response = ollama.chat(
    model='llama3.2:3b',
    messages=messages,
    tools=[move_forward, turn_left, stop],
)

# Parse and execute tool calls
if response.message.tool_calls:
    for call in response.message.tool_calls:
        fn = available_functions.get(call.function.name)
        if fn:
            result = fn(**call.function.arguments)
            print(f"Called {call.function.name}: {result}")
        else:
            print(f"Unknown tool: {call.function.name}")
else:
    # Model responded with text instead of a tool call
    print(response.message.content)
```

### Approach 2: Manual JSON Schema (for HTTP API / non-Python)

```python
import requests
import json

tools = [
    {
        "type": "function",
        "function": {
            "name": "move_forward",
            "description": "Move the robot forward at a given speed",
            "parameters": {
                "type": "object",
                "properties": {
                    "speed": {"type": "integer", "description": "Speed 0-100"},
                    "duration": {"type": "number", "description": "Seconds"}
                },
                "required": ["speed"]
            }
        }
    }
]

response = requests.post("http://10.33.155.83:11434/api/chat", json={
    "model": "llama3.2:3b",
    "messages": [{"role": "user", "content": "Move forward at half speed"}],
    "tools": tools,
    "stream": False
})

data = response.json()
tool_calls = data.get("message", {}).get("tool_calls", [])
for tc in tool_calls:
    name = tc["function"]["name"]
    args = tc["function"]["arguments"]
    print(f"Tool: {name}, Args: {args}")
```

### Approach 3: Full Agent Loop (Multi-Turn)

```python
import ollama

available_functions = {
    'move_forward': move_forward,
    'turn_left': turn_left,
    'stop': stop,
}
all_tools = list(available_functions.values())

messages = [
    {'role': 'system', 'content': 'You are a robot controller.'},
    {'role': 'user', 'content': 'Go forward, then turn left 90 degrees'}
]

while True:
    response = ollama.chat(
        model='llama3.2:3b',
        messages=messages,
        tools=all_tools,
    )
    messages.append(response.message)

    if not response.message.tool_calls:
        # No more tools to call; model is done
        print("Final:", response.message.content)
        break

    for tc in response.message.tool_calls:
        fn = available_functions.get(tc.function.name)
        if fn:
            result = fn(**tc.function.arguments)
            messages.append({
                'role': 'tool',
                'tool_name': tc.function.name,
                'content': str(result),
            })
        else:
            messages.append({
                'role': 'tool',
                'tool_name': tc.function.name,
                'content': 'Error: unknown tool',
            })
```

---

## Model Comparison for Tool Calling

| Model | Size | Tool Accuracy | Speed | RAM | Notes |
|-------|------|--------------|-------|-----|-------|
| llama3.2:3b | 2.0 GB | ~80% | Fast | ~3 GB | Our current model, good enough for simple tools |
| llama3.1:8b | 4.7 GB | ~89% | Medium | ~7 GB | Better accuracy, fits on Jetson (7.4 GB shared) |
| mistral:7b | 4.1 GB | Good | Fastest | ~7 GB | Strong tool calling, efficient |
| qwen3 | Varies | Very good | Varies | Varies | Featured in latest Ollama docs as primary example |

**For Jetson Orin Nano (7.4 GB unified memory):**
- llama3.2:3b is the safe choice -- leaves headroom for other services (RAG, PostgreSQL, Redis)
- llama3.1:8b would work but consumes most of the memory
- For MCP motor control with 3-5 simple tools, 3B should be sufficient

---

## Important Implementation Notes

1. **Python SDK auto-schema**: Use Google-style docstrings with `Args:` sections and type annotations. The SDK parses these into JSON tool schemas automatically.

2. **Tool response role**: After executing a tool, send the result back as `{'role': 'tool', 'tool_name': '...', 'content': '...'}`.

3. **Stream parameter**: For simplicity, use `stream=False` when doing tool calling. Streaming tool calls are supported but add parsing complexity.

4. **Known issue**: Some versions of the ollama Python library have a Pydantic validation error where `tool_calls.function.arguments` comes back as a JSON string instead of a dict. Keep the library updated (`pip install --upgrade ollama`).

5. **Fallback strategy**: If the model returns text instead of a tool call, you may need to re-prompt or parse the intent manually. Small models sometimes "forget" to use tools.

6. **Docker context**: From Docker containers on Jetson, Ollama is at `http://172.18.0.1:11434` (bridge gateway). From the host or RPi, it is at `http://10.33.155.83:11434`.

7. **Ollama Python library version**: Need v0.4+ for the function-as-tool auto-schema feature. Install: `pip install ollama>=0.4`.

---

## Next Steps for MCP Integration

1. Define motor control functions with proper type annotations and docstrings
2. Test tool calling with llama3.2:3b on Jetson using a simple script
3. If accuracy is too low, try llama3.1:8b or add few-shot examples in the system prompt
4. Integrate into MCP motor server as the LLM intent parser
5. Consider adding a confidence threshold -- if the model returns text instead of a tool call, ask for clarification

---

## Sources

- [Ollama Tool Calling Documentation](https://docs.ollama.com/capabilities/tool-calling)
- [Ollama Python Library 0.4 - Functions as Tools](https://ollama.com/blog/functions-as-tools)
- [Ollama Blog - Tool Support](https://ollama.com/blog/tool-support)
- [Ollama Blog - Streaming Tool Calls](https://ollama.com/blog/streaming-tool)
- [Ollama Chat API Documentation](https://docs.ollama.com/api/chat)
- [Ollama API (GitHub)](https://github.com/ollama/ollama/blob/main/docs/api.md)
- [Ollama Python Library (GitHub)](https://github.com/ollama/ollama-python)
- [Llama 3.2 Model Page](https://ollama.com/library/llama3.2)
- [Llama 3.2 Prompt Format (Meta)](https://github.com/meta-llama/llama-models/blob/main/models/llama3_2/text_prompt_format.md)
- [Ollama Tools Model Search](https://ollama.com/search?c=tools)
- [Best Ollama Models for Function Calling (Collabnix)](https://collabnix.com/best-ollama-models-for-function-calling-tools-complete-guide-2025/)
- [Function Calling with Ollama and Llama 3.2 (Zilliz)](https://zilliz.com/blog/function-calling-ollama-llama-3-milvus)
- [LangChain Forum - Tool Calling with Llama 3.2 3B](https://forum.langchain.com/t/tool-function-calling-with-llama-3-2-3b-instruct-model-local/2574)
