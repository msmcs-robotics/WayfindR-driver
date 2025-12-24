# Pi API Demo

Simple FastAPI application to control an LED on Raspberry Pi GPIO 14.

## Hardware Setup

Connect an LED to GPIO 14 (BCM numbering):
- LED anode (+) -> GPIO 14 (pin 8)
- LED cathode (-) -> 330Ω resistor -> GND (pin 6)

## Installation

```bash
# On Raspberry Pi
cd pi_api_demo
pip install -r requirements.txt
```

## Running

```bash
# Start the API server
uvicorn main:app --host 0.0.0.0 --port 8000

# Or run directly
python main.py
```

## API Endpoints

| Method | Endpoint      | Description          |
|--------|---------------|----------------------|
| GET    | /             | API info             |
| GET    | /led          | Get LED state        |
| POST   | /led          | Set LED state (JSON) |
| POST   | /led/on       | Turn LED on          |
| POST   | /led/off      | Turn LED off         |
| POST   | /led/toggle   | Toggle LED           |
| GET    | /health       | Health check         |

## Examples

```bash
# Check LED status
curl http://raspberrypi.local:8000/led

# Turn LED on
curl -X POST http://raspberrypi.local:8000/led/on

# Turn LED off
curl -X POST http://raspberrypi.local:8000/led/off

# Toggle LED
curl -X POST http://raspberrypi.local:8000/led/toggle

# Set state with JSON
curl -X POST http://raspberrypi.local:8000/led \
  -H "Content-Type: application/json" \
  -d '{"state": "on"}'
```

## Mock Mode

If RPi.GPIO is not available (e.g., testing on non-Pi system), the app runs in mock mode - it tracks state but doesn't control actual GPIO.
