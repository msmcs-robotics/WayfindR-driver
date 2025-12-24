# Sheep Helper - Livestock Health Screening System

A visual screening system for detecting potential health issues in sheep using computer vision and audio alerts. Designed for farmers working at gates who need hands-free audio notifications about animals that may require attention.

## Project Purpose

### The Problem

Sheep can develop various health conditions that manifest as visible tissue changes, particularly necrotic (dead/dying) tissue that appears dark or black in color. Early detection of these conditions is critical for:

- Animal welfare
- Preventing spread to other animals
- Reducing treatment costs
- Improving farm productivity

However, farmers working the gate during sheep handling (shearing, dipping, sorting) have their hands full and can't easily stop to examine each animal or check a screen.

### The Solution

Sheep Helper provides:

1. **Automated Visual Screening**: Uses a camera to scan the shaved/exposed skin area of sheep as they pass
2. **Pixel Analysis**: Analyzes the darkness/lightness ratio of the scanned area to detect potential issues
3. **Audio Alerts**: Plays distinct sounds so farmers can hear which animals need attention without looking at a screen
4. **Individual Tracking**: Assigns unique IDs to each face/animal for later identification and record-keeping

### Detection Logic

The system analyzes the cropped face region (with 10% border expansion):

- **Dark Pixels** (grayscale < 80): Potential necrotic or diseased tissue
- **Light Pixels** (grayscale > 170): Healthy tissue appearance

**Classification:**
- **Case 1 - HEALTHY** (Green): More light pixels than dark → Plays sound from `case1/` folder
- **Case 2 - ALERT** (Red): More dark pixels than light → Plays warning from `case2/` folder

## Requirements

- Python 3.8+
- Windows (for webcam access - WSL2 doesn't support webcams directly)
- Webcam positioned to capture sheep faces/scan areas
- Speakers for audio alerts

## Installation

```bash
cd sheep_helper
pip install -r requirements.txt
```

## Setup

### 1. Add Audio Files

Place audio files in the appropriate folders:

```
sheep_helper/
├── case1/          # Healthy confirmation sounds
│   ├── ok.wav
│   ├── clear.mp3
│   └── ...
├── case2/          # Alert/warning sounds
│   ├── alert.wav
│   ├── warning.mp3
│   └── ...
```

Supported formats: `.wav`, `.mp3`, `.mp4`, `.ogg`, `.flac`

The system randomly selects a file from the appropriate folder to provide variety.

### 2. Position Camera

- Mount camera to capture the scan area (typically the shaved side/back of sheep)
- Ensure consistent lighting for reliable detection
- Test with `python main.py` to verify detection works

## Usage

**Run from Windows (not WSL):**

```bash
python main.py
```

### Controls

| Key | Action |
|-----|--------|
| `q` | Quit and save session log |
| `s` | Save screenshot |
| `r` | Refresh audio files (after adding new sounds) |
| `m` | Toggle mute |

### Visual Indicators

- **Green Box**: Face detected, healthy classification
- **Red Box**: Face detected, alert classification (potential issue)
- **Yellow Box**: Segmentation border (10% expanded area analyzed)
- **ID Label**: Unique identifier for this face/animal

### Console Output

```
[OK] Face 1: Healthy scan. Playing confirmation...
[ALERT] Face 2: Potential issue detected! Playing alert...
```

## Project Structure

```
sheep_helper/
├── main.py             # Main application entry point
├── camera.py           # Webcam capture with debugging
├── face_detector.py    # OpenCV Haar Cascade detection
├── pixel_analyzer.py   # Dark/light pixel ratio analysis
├── audio_player.py     # Cross-platform audio playback
├── face_tracker.py     # Unique ID tracking and image saving
├── visualizer.py       # Drawing boxes and overlays
├── requirements.txt    # Python dependencies
├── README.md           # This documentation
├── case1/              # Healthy confirmation sounds
├── case2/              # Alert warning sounds
├── known_faces/        # Saved face images with IDs
└── screenshots/        # Saved screenshots
```

## Output Files

### Known Faces (`known_faces/`)

Each detected face is automatically saved:
```
sheep_0001_20241224_143022.png
sheep_0002_20241224_143025.png
...
```

Format: `sheep_XXXX_YYYYMMDD_HHMMSS.png`

### Session Logs (`known_faces/session_*.json`)

JSON log of each session with:
- Total faces tracked
- Per-face statistics
- Case classification history

## Customization

### Adjust Detection Thresholds

Edit `main.py`:

```python
# Lower = more sensitive to dark pixels
# Higher = less sensitive
analyzer = PixelAnalyzer(
    darkness_threshold=80,   # Pixels below this are "dark"
    light_threshold=170      # Pixels above this are "light"
)
```

### Adjust Border Expansion

```python
BORDER_PERCENT = 0.10  # 10% expansion around detected face
```

### Audio Cooldown

Edit `audio_player.py`:

```python
self.cooldown_seconds = 2.0  # Minimum seconds between plays
```

## Debugging

### Camera Issues

```bash
python main.py --debug
# or
python camera.py
```

This will show:
- System information (Windows/Linux/WSL detection)
- Available video devices
- OpenCV camera probe results

### No Audio

1. Check audio files exist in `case1/` and `case2/` folders
2. Supported formats: `.wav`, `.mp3`, `.mp4`, `.ogg`, `.flac`
3. Press `r` to refresh file list after adding new sounds

## Future Improvements

- [ ] Train custom ML model on sheep-specific data for better detection
- [ ] Add multiple condition classifications (not just healthy/alert)
- [ ] Integration with farm management systems
- [ ] Remote monitoring and alerts
- [ ] Historical trend analysis per animal ID

## Technical Notes

### Why Face Detection?

The OpenCV Haar Cascade face detector is used as a starting point because:
1. It's fast and works in real-time
2. It provides consistent bounding boxes for analysis
3. It can be replaced with a sheep-specific detector later

The actual health assessment is done by the pixel analyzer, not the face detector itself. The detector just provides the region of interest.

### WSL Limitation

Windows Subsystem for Linux (WSL2) cannot access USB webcams directly. The camera module detects this and provides clear guidance to run on Windows instead.

## License

Internal/proprietary - for authorized agricultural use.
