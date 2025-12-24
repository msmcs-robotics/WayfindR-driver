# Pi API Demo

A FastAPI application running on a Raspberry Pi that provides:
- LED control on GPIO 21
- Text-to-Speech through the built-in 3.5mm AUX output

## Hardware Setup

### Requirements
- Raspberry Pi 4 (tested with Raspbian Bullseye)
- LED connected to GPIO 21
- Speaker connected to 3.5mm AUX jack (headphone output)

### LED Wiring
- LED positive (long leg) → GPIO 21
- LED negative (short leg) → GND (through appropriate resistor ~220Ω)

### Audio Output
The Pi uses the built-in bcm2835 Headphones output (3.5mm jack).

**Important**: Do NOT use HDMI audio - ensure audio is routed to the headphone jack:
```bash
sudo raspi-config nonint do_audio 1
```

## Software Setup

### Dependencies
```bash
pip3 install fastapi uvicorn pydantic
sudo apt-get install espeak-ng sox alsa-utils
```

### Deploying from Development Machine
```bash
rsync -avz /home/devel/WayfindR-driver/pi_api_demo/ dnichols4@169.254.123.217:~/Desktop/pi_api_demo/
```

### Running the Server
```bash
cd ~/Desktop/pi_api_demo
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000
```

Or run directly:
```bash
python3 main.py
```

### Running in Background
```bash
cd ~/Desktop/pi_api_demo
nohup python3 -m uvicorn main:app --host 0.0.0.0 --port 8000 > /tmp/pi_api.log 2>&1 &
```

## Configuration

In `main.py`:
```python
LED_PIN = 21                    # GPIO pin for LED (BCM numbering)
AUDIO_DEVICE = "plughw:0,0"     # Built-in 3.5mm AUX output
```

## API Endpoints

### Root
- `GET /` - API information and available endpoints

### Health
- `GET /health` - Health check with GPIO and TTS status
  ```json
  {"status": "healthy", "gpio_available": true, "led_state": "off", "tts_speaking": false}
  ```

### LED Control
- `GET /led` - Get current LED state
- `POST /led` - Set LED state (body: `{"state": "on"}` or `{"state": "off"}`)
- `POST /led/on` - Turn LED on
- `POST /led/off` - Turn LED off
- `POST /led/toggle` - Toggle LED state

### Text-to-Speech
- `POST /speak` - Speak text asynchronously (body: `{"text": "Hello"}`)
- `POST /speak/sync` - Speak text synchronously (waits for completion)
- `GET /speak/status` - Check if TTS is currently speaking

## Example Usage

### LED Control
```bash
# Turn LED on
curl -X POST http://169.254.123.217:8000/led/on

# Get LED status
curl http://169.254.123.217:8000/led

# Turn LED off
curl -X POST http://169.254.123.217:8000/led/off

# Toggle LED
curl -X POST http://169.254.123.217:8000/led/toggle
```

### Text-to-Speech
```bash
# Speak text (async - returns immediately)
curl -X POST http://169.254.123.217:8000/speak \
  -H "Content-Type: application/json" \
  -d '{"text": "Hello from the Raspberry Pi!"}'

# Speak text (sync - waits for completion)
curl -X POST http://169.254.123.217:8000/speak/sync \
  -H "Content-Type: application/json" \
  -d '{"text": "This waits until done speaking."}'

# Check if speaking
curl http://169.254.123.217:8000/speak/status
```

## Technical Details

### TTS Pipeline

```
Text Input
    ↓
espeak-ng (generates 22050 Hz mono WAV)
    ↓
sox (converts to 48000 Hz stereo)
    ↓
aplay -D plughw:0,0 (plays through AUX jack)
    ↓
Speaker Output
```

The conversion to 48kHz stereo is required because:
- espeak-ng outputs 22050 Hz mono by default
- The bcm2835 audio driver works best with 48000 Hz stereo

### GPIO Configuration
- LED is connected to GPIO 21 (BCM numbering)
- GPIO mode: BCM
- Initial state: LOW (off)
- Uses RPi.GPIO library

### Audio Device
- Device: `plughw:0,0` (bcm2835 Headphones)
- Card 0 is the built-in audio
- The `plug` prefix handles format conversion automatically

## Troubleshooting

### Audio not working

1. **Check audio devices:**
   ```bash
   aplay -l
   # Should show: card 0: Headphones [bcm2835 Headphones]
   ```

2. **Set volume to maximum:**
   ```bash
   amixer -c 0 set PCM 100%
   ```

3. **Force audio to headphone jack:**
   ```bash
   sudo raspi-config nonint do_audio 1
   ```

4. **Test audio directly:**
   ```bash
   speaker-test -D plughw:0,0 -c 2 -t sine -f 440 -l 1
   ```

5. **Test TTS pipeline manually:**
   ```bash
   espeak-ng "Hello world" -w /tmp/test.wav
   sox /tmp/test.wav -r 48000 -c 2 /tmp/test48.wav
   aplay -D plughw:0,0 /tmp/test48.wav
   ```

### GPIO not available

1. Check if RPi.GPIO is installed:
   ```bash
   python3 -c "import RPi.GPIO; print('OK')"
   ```

2. Ensure running with sufficient permissions (may need sudo or gpio group)

3. Check if GPIO is enabled in `/boot/config.txt`

### API not accessible

1. Check if server is running:
   ```bash
   ps aux | grep uvicorn
   ```

2. Check logs:
   ```bash
   tail -f /tmp/pi_api.log
   ```

3. Test locally on Pi:
   ```bash
   curl http://localhost:8000/health
   ```

## Project Structure

```
pi_api_demo/
├── main.py             # Main FastAPI application
├── docs/
│   ├── README.md       # This documentation
│   └── AUDIO_SETUP.md  # Audio troubleshooting notes
```

## Network Configuration

The Pi is accessible at:
- IP: `169.254.123.217` (link-local address via USB/Ethernet)
- Port: `8000`
- SSH: `ssh dnichols4@169.254.123.217`
