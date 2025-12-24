"""
Pi API Demo - FastAPI application to control LED and Text-to-Speech
Run on Raspberry Pi with: uvicorn main:app --host 0.0.0.0 --port 8000
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from enum import Enum
import logging
import subprocess
import os
import tempfile

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Try to import RPi.GPIO, fall back to mock for testing
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    logger.info("RPi.GPIO loaded successfully")
except ImportError:
    GPIO_AVAILABLE = False
    logger.warning("RPi.GPIO not available - running in mock mode")

# Configuration
LED_PIN = 21
AUDIO_DEVICE = "plughw:0,0"  # Built-in 3.5mm AUX output

app = FastAPI(
    title="Pi API Demo",
    description="LED Control and Text-to-Speech API for Raspberry Pi with RASPIAUDIO Ultra V3",
    version="2.0.0"
)

# Enable CORS for cross-origin requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Track LED state
led_state = {"on": False}

# Track TTS state
tts_state = {"speaking": False}


class LEDState(str, Enum):
    on = "on"
    off = "off"


class LEDCommand(BaseModel):
    state: LEDState


class LEDResponse(BaseModel):
    success: bool
    led_state: str
    message: str


class TTSRequest(BaseModel):
    text: str
    voice: str = "en-us"  # espeak-ng voice


class TTSResponse(BaseModel):
    success: bool
    message: str
    text: str


def setup_gpio():
    """Initialize GPIO settings"""
    if GPIO_AVAILABLE:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LED_PIN, GPIO.OUT)
        GPIO.output(LED_PIN, GPIO.LOW)
        logger.info(f"GPIO {LED_PIN} initialized as output")


def cleanup_gpio():
    """Clean up GPIO on shutdown"""
    if GPIO_AVAILABLE:
        GPIO.cleanup()
        logger.info("GPIO cleaned up")


def speak_text(text: str, voice: str = "en-us"):
    """Generate and play TTS audio using espeak-ng and sox"""
    global tts_state
    tts_state["speaking"] = True

    try:
        # Create temp files for audio processing
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_raw:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_converted:
                raw_path = tmp_raw.name
                converted_path = tmp_converted.name

        # Generate speech with espeak-ng
        espeak_cmd = [
            "espeak-ng",
            "-v", voice,
            "-w", raw_path,
            text
        ]
        subprocess.run(espeak_cmd, check=True, capture_output=True)

        # Convert to 48kHz stereo for RASPIAUDIO HAT
        sox_cmd = [
            "sox",
            raw_path,
            "-r", "48000",
            "-c", "2",
            converted_path
        ]
        subprocess.run(sox_cmd, check=True, capture_output=True)

        # Play audio through RASPIAUDIO HAT
        aplay_cmd = [
            "aplay",
            "-D", AUDIO_DEVICE,
            converted_path
        ]
        subprocess.run(aplay_cmd, check=True, capture_output=True)

        # Cleanup temp files
        os.unlink(raw_path)
        os.unlink(converted_path)

        logger.info(f"TTS completed: {text[:50]}...")

    except subprocess.CalledProcessError as e:
        logger.error(f"TTS failed: {e}")
        raise
    finally:
        tts_state["speaking"] = False


@app.on_event("startup")
async def startup_event():
    """Initialize GPIO on startup"""
    setup_gpio()
    logger.info("Pi API Demo started")


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up GPIO on shutdown"""
    cleanup_gpio()
    logger.info("Pi API Demo shutdown")


@app.get("/", response_model=dict)
async def root():
    """Root endpoint - API info"""
    return {
        "name": "Pi API Demo",
        "description": "LED Control and TTS API for Raspberry Pi",
        "version": "2.0.0",
        "gpio_available": GPIO_AVAILABLE,
        "led_pin": LED_PIN,
        "audio_device": AUDIO_DEVICE,
        "endpoints": {
            "GET /": "This info",
            "GET /health": "Health check",
            "GET /led": "Get current LED state",
            "POST /led": "Set LED state",
            "POST /led/on": "Turn LED on",
            "POST /led/off": "Turn LED off",
            "POST /led/toggle": "Toggle LED state",
            "POST /speak": "Text-to-speech (body: {\"text\": \"Hello\"})",
            "GET /speak/status": "Get TTS status"
        }
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "gpio_available": GPIO_AVAILABLE,
        "led_state": "on" if led_state["on"] else "off",
        "tts_speaking": tts_state["speaking"]
    }


# ============== LED Endpoints ==============

@app.get("/led", response_model=LEDResponse)
async def get_led_state():
    """Get current LED state"""
    return LEDResponse(
        success=True,
        led_state="on" if led_state["on"] else "off",
        message=f"LED is currently {'on' if led_state['on'] else 'off'}"
    )


@app.post("/led", response_model=LEDResponse)
async def set_led_state(command: LEDCommand):
    """Set LED to specific state"""
    if command.state == LEDState.on:
        return await turn_led_on()
    else:
        return await turn_led_off()


@app.post("/led/on", response_model=LEDResponse)
async def turn_led_on():
    """Turn LED on"""
    if GPIO_AVAILABLE:
        GPIO.output(LED_PIN, GPIO.HIGH)
    led_state["on"] = True
    logger.info("LED turned ON")
    return LEDResponse(
        success=True,
        led_state="on",
        message="LED turned on successfully"
    )


@app.post("/led/off", response_model=LEDResponse)
async def turn_led_off():
    """Turn LED off"""
    if GPIO_AVAILABLE:
        GPIO.output(LED_PIN, GPIO.LOW)
    led_state["on"] = False
    logger.info("LED turned OFF")
    return LEDResponse(
        success=True,
        led_state="off",
        message="LED turned off successfully"
    )


@app.post("/led/toggle", response_model=LEDResponse)
async def toggle_led():
    """Toggle LED state"""
    if led_state["on"]:
        return await turn_led_off()
    else:
        return await turn_led_on()


# ============== TTS Endpoints ==============

@app.post("/speak", response_model=TTSResponse)
async def speak(request: TTSRequest, background_tasks: BackgroundTasks):
    """Text-to-speech endpoint - speaks the provided text through RASPIAUDIO HAT"""
    if tts_state["speaking"]:
        raise HTTPException(status_code=429, detail="TTS is currently speaking. Please wait.")

    if not request.text.strip():
        raise HTTPException(status_code=400, detail="Text cannot be empty")

    # Run TTS in background so API responds immediately
    background_tasks.add_task(speak_text, request.text, request.voice)

    logger.info(f"TTS requested: {request.text[:50]}...")

    return TTSResponse(
        success=True,
        message="Speaking text...",
        text=request.text
    )


@app.post("/speak/sync", response_model=TTSResponse)
async def speak_sync(request: TTSRequest):
    """Synchronous text-to-speech - waits for speech to complete"""
    if tts_state["speaking"]:
        raise HTTPException(status_code=429, detail="TTS is currently speaking. Please wait.")

    if not request.text.strip():
        raise HTTPException(status_code=400, detail="Text cannot be empty")

    try:
        speak_text(request.text, request.voice)
        return TTSResponse(
            success=True,
            message="Speech completed",
            text=request.text
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"TTS failed: {str(e)}")


@app.get("/speak/status")
async def get_tts_status():
    """Get current TTS status"""
    return {
        "speaking": tts_state["speaking"]
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
