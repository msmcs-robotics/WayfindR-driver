#!/usr/bin/env python3
"""
Web Chat App with LLM + Pi Control + TTS

A FastAPI web application that provides:
- Web-based chat interface with Ollama (llama3:8b)
- LED control on Raspberry Pi
- Text-to-speech through RASPIAUDIO Ultra V3

Run with: python app.py
Access at: http://localhost:10000
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import requests
import json
import re
import asyncio
import uvicorn

# Configuration
OLLAMA_BASE_URL = "http://localhost:11434"
MODEL_NAME = "llama3:8b"
PI_API_URL = "http://169.254.123.217:8000"
PORT = 10000

# System prompt for the LLM
SYSTEM_PROMPT = """You are a helpful assistant that can control a Raspberry Pi with an LED.

You have access to these LED commands. When the user asks you to control the LED, respond with the appropriate command in this EXACT format:

[TOOL:led_on] - Turn the LED on
[TOOL:led_off] - Turn the LED off
[TOOL:led_toggle] - Toggle the LED state
[TOOL:led_status] - Check if the LED is on or off

Examples:
- User: "Turn on the light" -> You respond: "I'll turn on the LED now. [TOOL:led_on]"
- User: "Switch off the LED" -> You respond: "Turning off the LED. [TOOL:led_off]"
- User: "What's the LED status?" -> You respond: "Let me check. [TOOL:led_status]"

Keep your responses conversational and concise. All of your responses will be spoken out loud through a speaker, so write naturally as if you're talking."""

app = FastAPI(title="Pi Control Chat", version="2.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Store chat history per connection
chat_histories = {}

# HTML template for the chat interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Pi Control Chat</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
        }
        .header {
            background: rgba(255,255,255,0.1);
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid rgba(255,255,255,0.1);
        }
        .header h1 {
            color: #fff;
            font-size: 1.5rem;
        }
        .status {
            display: flex;
            gap: 15px;
        }
        .status-item {
            display: flex;
            align-items: center;
            gap: 5px;
            color: #aaa;
            font-size: 0.9rem;
        }
        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: #666;
        }
        .status-dot.connected {
            background: #4CAF50;
        }
        .status-dot.disconnected {
            background: #f44336;
        }
        .chat-container {
            flex: 1;
            overflow-y: auto;
            padding: 20px;
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .message {
            max-width: 80%;
            padding: 12px 16px;
            border-radius: 18px;
            line-height: 1.4;
            word-wrap: break-word;
        }
        .message.user {
            align-self: flex-end;
            background: #0084ff;
            color: white;
            border-bottom-right-radius: 4px;
        }
        .message.assistant {
            align-self: flex-start;
            background: rgba(255,255,255,0.1);
            color: #fff;
            border-bottom-left-radius: 4px;
        }
        .message.system {
            align-self: center;
            background: rgba(76, 175, 80, 0.2);
            color: #4CAF50;
            font-size: 0.9rem;
            border-radius: 8px;
        }
        .message.error {
            align-self: center;
            background: rgba(244, 67, 54, 0.2);
            color: #f44336;
            font-size: 0.9rem;
            border-radius: 8px;
        }
        .tool-result {
            margin-top: 8px;
            padding: 8px 12px;
            background: rgba(255,255,255,0.05);
            border-left: 3px solid #4CAF50;
            font-size: 0.9rem;
            color: #aaa;
        }
        .input-container {
            padding: 15px 20px;
            background: rgba(255,255,255,0.05);
            border-top: 1px solid rgba(255,255,255,0.1);
            display: flex;
            gap: 10px;
        }
        #messageInput {
            flex: 1;
            padding: 12px 16px;
            border: none;
            border-radius: 24px;
            background: rgba(255,255,255,0.1);
            color: #fff;
            font-size: 1rem;
            outline: none;
        }
        #messageInput::placeholder {
            color: #888;
        }
        #messageInput:focus {
            background: rgba(255,255,255,0.15);
        }
        button {
            padding: 12px 24px;
            border: none;
            border-radius: 24px;
            background: #0084ff;
            color: white;
            font-size: 1rem;
            cursor: pointer;
            transition: background 0.2s;
        }
        button:hover {
            background: #0073e6;
        }
        button:disabled {
            background: #555;
            cursor: not-allowed;
        }
        .quick-actions {
            padding: 10px 20px;
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        .quick-btn {
            padding: 8px 16px;
            border: 1px solid rgba(255,255,255,0.2);
            border-radius: 16px;
            background: transparent;
            color: #aaa;
            font-size: 0.85rem;
            cursor: pointer;
            transition: all 0.2s;
        }
        .quick-btn:hover {
            background: rgba(255,255,255,0.1);
            color: #fff;
        }
        .typing {
            display: none;
            align-self: flex-start;
            padding: 12px 16px;
            background: rgba(255,255,255,0.1);
            border-radius: 18px;
            color: #888;
        }
        .typing.active {
            display: block;
        }
        .typing span {
            animation: blink 1s infinite;
        }
        .typing span:nth-child(2) { animation-delay: 0.2s; }
        .typing span:nth-child(3) { animation-delay: 0.4s; }
        @keyframes blink {
            0%, 100% { opacity: 0.2; }
            50% { opacity: 1; }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Pi Control Chat</h1>
        <div class="status">
            <div class="status-item">
                <span class="status-dot" id="ollamaStatus"></span>
                <span>Ollama</span>
            </div>
            <div class="status-item">
                <span class="status-dot" id="piStatus"></span>
                <span>Pi</span>
            </div>
            <div class="status-item">
                <span class="status-dot" id="wsStatus"></span>
                <span>WebSocket</span>
            </div>
        </div>
    </div>

    <div class="quick-actions">
        <button class="quick-btn" onclick="sendQuick('Turn on the LED')">LED On</button>
        <button class="quick-btn" onclick="sendQuick('Turn off the LED')">LED Off</button>
        <button class="quick-btn" onclick="sendQuick('What is the LED status?')">LED Status</button>
        <button class="quick-btn" onclick="sendQuick('Say hello to me')">Say Hello</button>
        <button class="quick-btn" onclick="sendQuick('Tell me a joke out loud')">Tell Joke</button>
        <button class="quick-btn" onclick="clearChat()">Clear Chat</button>
    </div>

    <div class="chat-container" id="chatContainer">
        <div class="message system">
            Welcome! Chat with the AI to control your Raspberry Pi. You can control the LED and have it speak to you!
        </div>
    </div>

    <div class="typing" id="typingIndicator">
        <span>.</span><span>.</span><span>.</span>
    </div>

    <div class="input-container">
        <input type="text" id="messageInput" placeholder="Type a message..." autocomplete="off">
        <button id="sendBtn" onclick="sendMessage()">Send</button>
    </div>

    <script>
        let ws = null;
        let isConnected = false;

        function connect() {
            ws = new WebSocket(`ws://${window.location.host}/ws`);

            ws.onopen = () => {
                isConnected = true;
                document.getElementById('wsStatus').classList.add('connected');
                document.getElementById('wsStatus').classList.remove('disconnected');
                checkStatus();
            };

            ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                handleMessage(data);
            };

            ws.onclose = () => {
                isConnected = false;
                document.getElementById('wsStatus').classList.remove('connected');
                document.getElementById('wsStatus').classList.add('disconnected');
                setTimeout(connect, 3000);
            };

            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
        }

        function handleMessage(data) {
            const container = document.getElementById('chatContainer');
            const typing = document.getElementById('typingIndicator');

            if (data.type === 'status') {
                document.getElementById('ollamaStatus').classList.toggle('connected', data.ollama);
                document.getElementById('ollamaStatus').classList.toggle('disconnected', !data.ollama);
                document.getElementById('piStatus').classList.toggle('connected', data.pi);
                document.getElementById('piStatus').classList.toggle('disconnected', !data.pi);
            } else if (data.type === 'response') {
                typing.classList.remove('active');
                document.getElementById('sendBtn').disabled = false;

                const msgDiv = document.createElement('div');
                msgDiv.className = 'message assistant';
                msgDiv.textContent = data.text;

                if (data.tool_results && data.tool_results.length > 0) {
                    const toolDiv = document.createElement('div');
                    toolDiv.className = 'tool-result';
                    toolDiv.textContent = data.tool_results.join(' | ');
                    msgDiv.appendChild(toolDiv);
                }

                container.appendChild(msgDiv);
                container.scrollTop = container.scrollHeight;
            } else if (data.type === 'error') {
                typing.classList.remove('active');
                document.getElementById('sendBtn').disabled = false;

                const msgDiv = document.createElement('div');
                msgDiv.className = 'message error';
                msgDiv.textContent = data.text;
                container.appendChild(msgDiv);
                container.scrollTop = container.scrollHeight;
            } else if (data.type === 'stream') {
                // Handle streaming responses
                let lastMsg = container.querySelector('.message.assistant:last-child');
                if (!lastMsg || lastMsg.dataset.complete === 'true') {
                    lastMsg = document.createElement('div');
                    lastMsg.className = 'message assistant';
                    container.appendChild(lastMsg);
                }
                lastMsg.textContent += data.text;
                container.scrollTop = container.scrollHeight;
            } else if (data.type === 'stream_end') {
                const lastMsg = container.querySelector('.message.assistant:last-child');
                if (lastMsg) {
                    lastMsg.dataset.complete = 'true';
                    if (data.tool_results && data.tool_results.length > 0) {
                        const toolDiv = document.createElement('div');
                        toolDiv.className = 'tool-result';
                        toolDiv.textContent = data.tool_results.join(' | ');
                        lastMsg.appendChild(toolDiv);
                    }
                }
                typing.classList.remove('active');
                document.getElementById('sendBtn').disabled = false;
            }
        }

        function sendMessage() {
            const input = document.getElementById('messageInput');
            const text = input.value.trim();
            if (!text || !isConnected) return;

            const container = document.getElementById('chatContainer');
            const msgDiv = document.createElement('div');
            msgDiv.className = 'message user';
            msgDiv.textContent = text;
            container.appendChild(msgDiv);
            container.scrollTop = container.scrollHeight;

            document.getElementById('typingIndicator').classList.add('active');
            document.getElementById('sendBtn').disabled = true;

            ws.send(JSON.stringify({ type: 'message', text: text }));
            input.value = '';
        }

        function sendQuick(text) {
            document.getElementById('messageInput').value = text;
            sendMessage();
        }

        function clearChat() {
            const container = document.getElementById('chatContainer');
            container.innerHTML = '<div class="message system">Chat cleared. Start a new conversation!</div>';
            ws.send(JSON.stringify({ type: 'clear' }));
        }

        function checkStatus() {
            if (isConnected) {
                ws.send(JSON.stringify({ type: 'status' }));
            }
        }

        document.getElementById('messageInput').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') sendMessage();
        });

        connect();
        setInterval(checkStatus, 30000);
    </script>
</body>
</html>
"""


def check_ollama_connection() -> bool:
    """Check if Ollama is accessible"""
    try:
        response = requests.get(f"{OLLAMA_BASE_URL}/api/tags", timeout=5)
        return response.status_code == 200
    except:
        return False


def check_pi_connection() -> bool:
    """Check if Pi API is accessible"""
    try:
        response = requests.get(f"{PI_API_URL}/health", timeout=5)
        return response.status_code == 200
    except:
        return False


def execute_tool(tool_name: str, tool_arg: str = None) -> str:
    """Execute a tool command on the Pi"""
    try:
        if tool_name == "led_on":
            response = requests.post(f"{PI_API_URL}/led/on", timeout=5)
            data = response.json()
            return f"LED turned ON"
        elif tool_name == "led_off":
            response = requests.post(f"{PI_API_URL}/led/off", timeout=5)
            data = response.json()
            return f"LED turned OFF"
        elif tool_name == "led_toggle":
            response = requests.post(f"{PI_API_URL}/led/toggle", timeout=5)
            data = response.json()
            return f"LED toggled (now {data.get('led_state', 'unknown')})"
        elif tool_name == "led_status":
            response = requests.get(f"{PI_API_URL}/led", timeout=5)
            data = response.json()
            return f"LED is {data.get('led_state', 'unknown')}"
        elif tool_name == "speak":
            if tool_arg:
                response = requests.post(
                    f"{PI_API_URL}/speak",
                    json={"text": tool_arg},
                    timeout=30
                )
                if response.status_code == 200:
                    return f"Speaking: \"{tool_arg[:50]}...\""
                else:
                    return f"Speech failed: {response.text}"
            return "No text provided for speech"
        else:
            return f"Unknown tool: {tool_name}"
    except Exception as e:
        return f"Error: {str(e)}"


def speak_text(text: str) -> str:
    """Send text to Pi for TTS"""
    try:
        response = requests.post(
            f"{PI_API_URL}/speak",
            json={"text": text},
            timeout=30
        )
        if response.status_code == 200:
            return "Speaking..."
        else:
            return f"Speech failed: {response.text}"
    except Exception as e:
        return f"Speech error: {str(e)}"


def process_response(response: str) -> tuple:
    """Process LLM response to extract and execute tool commands, then speak the response"""
    # Pattern for LED tools: [TOOL:led_on]
    tool_pattern = r'\[TOOL:(\w+)\]'
    tool_matches = re.findall(tool_pattern, response)

    tool_results = []

    # Execute LED tool commands
    for tool_name in tool_matches:
        result = execute_tool(tool_name)
        tool_results.append(result)

    # Clean response (remove tool tags)
    cleaned = re.sub(tool_pattern, '', response).strip()

    # ALWAYS speak the cleaned response through the Pi
    if cleaned:
        speak_result = speak_text(cleaned)
        tool_results.append(speak_result)

    return cleaned, tool_results


def chat_with_ollama(prompt: str, history: list) -> str:
    """Get response from Ollama"""
    history.append(f"User: {prompt}")
    full_prompt = "\n".join(history[-10:])

    try:
        response = requests.post(
            f"{OLLAMA_BASE_URL}/api/generate",
            json={
                "model": MODEL_NAME,
                "prompt": full_prompt,
                "system": SYSTEM_PROMPT,
                "stream": False
            },
            timeout=120
        )
        response.raise_for_status()
        data = response.json()
        return data.get("response", "")
    except Exception as e:
        return f"Error communicating with Ollama: {str(e)}"


@app.get("/", response_class=HTMLResponse)
async def get_chat_page():
    """Serve the chat HTML page"""
    return HTML_TEMPLATE


@app.get("/health")
async def health():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "ollama": check_ollama_connection(),
        "pi": check_pi_connection()
    }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time chat"""
    await websocket.accept()
    connection_id = id(websocket)
    chat_histories[connection_id] = []

    try:
        while True:
            data = await websocket.receive_json()

            if data.get("type") == "status":
                await websocket.send_json({
                    "type": "status",
                    "ollama": check_ollama_connection(),
                    "pi": check_pi_connection()
                })

            elif data.get("type") == "clear":
                chat_histories[connection_id] = []

            elif data.get("type") == "message":
                text = data.get("text", "").strip()
                if not text:
                    continue

                history = chat_histories[connection_id]

                # Get LLM response
                try:
                    llm_response = chat_with_ollama(text, history)
                    cleaned, tool_results = process_response(llm_response)
                    history.append(f"Assistant: {cleaned}")

                    await websocket.send_json({
                        "type": "response",
                        "text": cleaned,
                        "tool_results": tool_results
                    })
                except Exception as e:
                    await websocket.send_json({
                        "type": "error",
                        "text": f"Error: {str(e)}"
                    })

    except WebSocketDisconnect:
        if connection_id in chat_histories:
            del chat_histories[connection_id]


if __name__ == "__main__":
    print(f"Starting Pi Control Chat on http://localhost:{PORT}")
    print(f"Ollama: {OLLAMA_BASE_URL}")
    print(f"Pi API: {PI_API_URL}")
    uvicorn.run(app, host="0.0.0.0", port=PORT)
