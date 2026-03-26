"""
Ollama Tool-Calling Bridge for AMBOT Motor Control.

Connects Ollama's native tool calling API to the MotorInterface.
This is the simplest integration path — no MCP server needed.

The LLM (llama3.2:3b) receives tool definitions, decides when to call them,
and this bridge executes the actual motor commands.

Usage:
    from mcp_ability.ollama_bridge import AmbotAgent
    agent = AmbotAgent(simulate=True)
    response = agent.chat("Move the robot forward")
    # LLM calls move_forward tool → motors spin → response returned

Architecture:
    User message → Ollama LLM → tool_call decision → bridge executes tool
    → result sent back to LLM → LLM generates final response → user
"""

import json
import logging
from typing import Any

logger = logging.getLogger(__name__)

# Ollama API URL (default for Jetson local install)
DEFAULT_OLLAMA_URL = "http://localhost:11434"
DEFAULT_MODEL = "llama3.2:3b"


# ── Tool Definitions (Ollama format) ─────────────────────

MOTOR_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "move_forward",
            "description": "Move the robot forward. The robot will keep moving until stop_motors is called, or until duration seconds if specified.",
            "parameters": {
                "type": "object",
                "properties": {
                    "speed": {
                        "type": "integer",
                        "description": "Speed percentage, 0 to 100. Default 50.",
                    },
                    "duration": {
                        "type": "number",
                        "description": "Seconds to move. 0 means continuous until stop. Default 0.",
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "move_backward",
            "description": "Move the robot backward.",
            "parameters": {
                "type": "object",
                "properties": {
                    "speed": {
                        "type": "integer",
                        "description": "Speed percentage, 0 to 100. Default 50.",
                    },
                    "duration": {
                        "type": "number",
                        "description": "Seconds to move. 0 means continuous. Default 0.",
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "turn_left",
            "description": "Spin the robot left (counterclockwise).",
            "parameters": {
                "type": "object",
                "properties": {
                    "speed": {
                        "type": "integer",
                        "description": "Speed percentage, 0 to 100. Default 50.",
                    },
                    "duration": {
                        "type": "number",
                        "description": "Seconds to turn. 0 means continuous. Default 0.",
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "turn_right",
            "description": "Spin the robot right (clockwise).",
            "parameters": {
                "type": "object",
                "properties": {
                    "speed": {
                        "type": "integer",
                        "description": "Speed percentage, 0 to 100. Default 50.",
                    },
                    "duration": {
                        "type": "number",
                        "description": "Seconds to turn. 0 means continuous. Default 0.",
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_motors",
            "description": "Stop all motors immediately. Always call this when the user wants the robot to stop moving.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_motor_status",
            "description": "Get the current motor state including speeds, direction, and how long the robot has been running.",
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        },
    },
]


SYSTEM_PROMPT = """\
You are AMBOT, a robot assistant. You can control the robot's motors using tools.

When the user asks you to move, turn, or stop, use the appropriate tool.
When the user asks about the robot's state, use get_motor_status.

Keep your responses short and conversational. After executing a motor command,
briefly confirm what you did.

Examples:
- "Go forward" → call move_forward
- "Turn right for 3 seconds" → call turn_right with duration=3
- "Stop" → call stop_motors
- "How fast are you going?" → call get_motor_status
- "Move forward slowly" → call move_forward with speed=25
"""


class AmbotAgent:
    """
    Chat agent that connects Ollama tool calling to motor control.

    Uses Ollama's /api/chat endpoint with tools parameter.
    Handles the tool call → execute → send result → get final response loop.
    """

    def __init__(self, simulate=True, model=DEFAULT_MODEL,
                 ollama_url=DEFAULT_OLLAMA_URL, auto_stop_timeout=30.0):
        from .motor_interface import MotorInterface
        self.motors = MotorInterface(simulate=simulate,
                                     auto_stop_timeout=auto_stop_timeout)
        self.model = model
        self.ollama_url = ollama_url
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        self._tool_map = {
            "move_forward": self._move_forward,
            "move_backward": self._move_backward,
            "turn_left": self._turn_left,
            "turn_right": self._turn_right,
            "stop_motors": self._stop_motors,
            "get_motor_status": self._get_motor_status,
        }
        mode = "SIMULATE" if simulate else "HARDWARE"
        logger.info("AmbotAgent ready (%s, model=%s)", mode, model)

    # ── Tool implementations ─────────────────────────────

    def _move_forward(self, speed=50, duration=0, **_):
        speed = max(0, min(100, int(speed)))
        return self.motors.drive(speed, speed, duration=float(duration),
                                 label="move_forward")

    def _move_backward(self, speed=50, duration=0, **_):
        speed = max(0, min(100, int(speed)))
        return self.motors.drive(-speed, -speed, duration=float(duration),
                                 label="move_backward")

    def _turn_left(self, speed=50, duration=0, **_):
        speed = max(0, min(100, int(speed)))
        return self.motors.drive(-speed, speed, duration=float(duration),
                                 label="turn_left")

    def _turn_right(self, speed=50, duration=0, **_):
        speed = max(0, min(100, int(speed)))
        return self.motors.drive(speed, -speed, duration=float(duration),
                                 label="turn_right")

    def _stop_motors(self, **_):
        return self.motors.stop(reason="llm_tool_call")

    def _get_motor_status(self, **_):
        return self.motors.get_status()

    # ── Execute a tool call ──────────────────────────────

    def _execute_tool(self, name: str, arguments: dict) -> str:
        """Execute a tool and return the result as a JSON string."""
        func = self._tool_map.get(name)
        if func is None:
            return json.dumps({"error": f"Unknown tool: {name}"})
        try:
            result = func(**arguments)
            return json.dumps(result)
        except Exception as e:
            logger.error("Tool %s failed: %s", name, e)
            return json.dumps({"error": str(e)})

    # ── Chat with tool calling loop ──────────────────────

    def chat(self, user_message: str) -> str:
        """
        Send a message to the LLM and handle any tool calls.

        Returns the LLM's final text response after all tools are executed.
        """
        import urllib.request

        self.messages.append({"role": "user", "content": user_message})

        # Agent loop: keep going until LLM returns a text response (no more tool calls)
        max_iterations = 5  # Safety limit
        for _ in range(max_iterations):
            # Call Ollama
            payload = {
                "model": self.model,
                "messages": self.messages,
                "tools": MOTOR_TOOLS,
                "stream": False,
            }

            try:
                req = urllib.request.Request(
                    f"{self.ollama_url}/api/chat",
                    data=json.dumps(payload).encode(),
                    headers={"Content-Type": "application/json"},
                )
                with urllib.request.urlopen(req, timeout=60) as resp:
                    result = json.loads(resp.read().decode())
            except Exception as e:
                error_msg = f"Ollama error: {e}"
                logger.error(error_msg)
                return error_msg

            message = result.get("message", {})
            tool_calls = message.get("tool_calls", [])

            if not tool_calls:
                # No tool calls — LLM gave a text response
                text = message.get("content", "")
                self.messages.append({"role": "assistant", "content": text})
                return text

            # Execute each tool call
            self.messages.append(message)  # Add assistant message with tool_calls

            for call in tool_calls:
                func_info = call.get("function", {})
                name = func_info.get("name", "")
                args = func_info.get("arguments", {})

                logger.info("Tool call: %s(%s)", name, args)
                result_str = self._execute_tool(name, args)

                # Send tool result back to LLM
                self.messages.append({
                    "role": "tool",
                    "content": result_str,
                })

        return "I processed your request."

    def reset(self):
        """Clear conversation history."""
        self.messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    def cleanup(self):
        """Release motor resources."""
        self.motors.cleanup()
