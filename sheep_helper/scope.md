# Sheep Helper - Project Scope

## Overview

**Sheep Helper** is a livestock health screening system that uses computer vision and audio alerts to help farmers detect potential health issues in sheep during routine handling operations (shearing, dipping, sorting). The system provides hands-free notifications when animals show visual signs of necrotic tissue or other health concerns.

## Project Purpose

### The Agricultural Problem

Sheep can develop various health conditions that manifest as visible tissue changes, particularly necrotic (dead/dying) tissue that appears dark or black on the skin. Early detection is critical for:

- Animal welfare and prompt treatment
- Preventing spread to other animals in the flock
- Reducing treatment costs
- Improving overall farm productivity

However, farmers working at gates during sheep handling have their hands full with physical tasks and cannot easily stop to examine each animal or monitor a screen continuously.

### The Technical Solution

Sheep Helper addresses this by:

1. **Automated Visual Screening**: Uses a webcam to scan exposed skin areas as sheep pass by
2. **Real-time Pixel Analysis**: Analyzes the darkness/lightness ratio to detect potential tissue problems
3. **Audio Feedback System**: Plays distinct sounds (randomly selected from sound banks) so farmers can hear which animals need attention without looking at a screen
4. **Individual Animal Tracking**: Assigns unique IDs to each detected face/animal using histogram-based face matching for identification and record-keeping

## Relationship to WayfindR Project

Sheep Helper is **independent and unrelated** to the main WayfindR project. They share the same parent repository directory but serve completely different purposes:

- **WayfindR-driver**: A robotics project focused on indoor mapping, navigation, SLAM, and autonomous movement using ROS2, Raspberry Pi, and various sensors
- **Sheep Helper**: An agricultural computer vision tool for livestock health screening with no robotics components

**Why the unusual name?** "Sheep Helper" is a creative/playful name choice that fits the agricultural domain. The system helps identify sheep that may need attention.

**Repository location**: Both projects happen to be in the same development workspace (`WayfindR-driver/`) but have no technical dependencies on each other.

## Key Files and Components

### Core Application Files

| File | Purpose | Key Features |
|------|---------|--------------|
| `main.py` | Main application entry point | - Orchestrates all components<br>- Main event loop with webcam processing<br>- Keyboard controls (q/s/r/m/c)<br>- FPS calculation and display<br>- Session management |
| `camera.py` | Webcam capture and system detection | - Cross-platform webcam support<br>- WSL detection and warnings<br>- Comprehensive debugging tools<br>- DirectShow backend support (Windows)<br>- Device enumeration utilities |
| `face_detector.py` | OpenCV Haar Cascade face detection | - Uses pre-trained frontal face detector<br>- Configurable scale/neighbors parameters<br>- Bounding box expansion utility |
| `face_tracker.py` | Face identification and persistence | - Histogram-based face matching (HSV color space)<br>- Unique ID assignment and tracking<br>- Saves face images to disk<br>- Session logging (JSON format)<br>- Prevents duplicate alerting |
| `pixel_analyzer.py` | Health assessment via pixel analysis | - Counts dark pixels (< 80 grayscale)<br>- Counts light pixels (> 170 grayscale)<br>- Case classification: HEALTHY vs ALERT<br>- Configurable thresholds<br>- Border expansion for region analysis |
| `audio_player.py` | Cross-platform audio playback | - Random sound selection from folders<br>- Threaded playback (non-blocking)<br>- Windows (winsound/PowerShell Media Player)<br>- macOS (afplay)<br>- Linux (aplay/paplay/mpv/ffplay)<br>- Cooldown timer to prevent spam |
| `visualizer.py` | Drawing and overlay rendering | - Color-coded bounding boxes<br>- Face ID labels<br>- Statistics overlay<br>- Instructions and legend |

### Configuration and Data Files

| File/Folder | Purpose |
|-------------|---------|
| `requirements.txt` | Python dependencies (opencv-python, numpy) |
| `README.md` | Comprehensive user documentation |
| `convert_audio.sh` | Utility script to convert MP4 audio files to MP3 |
| `case1/` | Audio files for healthy sheep (6 MP3 files) |
| `case2/` | Audio files for alert/warning (3 MP3 files) |
| `venv/` | Python virtual environment (Windows-based) |
| `known_faces/` | Auto-created folder for saved face images |
| `screenshots/` | Auto-created folder for saved screenshots |

### Detection Logic

The system uses a simple but effective pixel-based classification:

```
Analyzed Region: Detected face bounding box + 10% border expansion

Dark Pixels: Grayscale value < 80
- Indicates potential necrotic or diseased tissue (appears black/dark)

Light Pixels: Grayscale value > 170
- Indicates healthy tissue appearance (normal shaved skin color)

Classification:
- If dark_count > light_count → CASE 2 (ALERT) → Red box → Warning sound
- If light_count > dark_count → CASE 1 (HEALTHY) → Green box → Confirmation sound
- If equal → CASE 0 (UNCERTAIN) → Yellow box
```

## Current Implementation State

### Fully Implemented Features

- **Core Detection Pipeline**
  - ✓ Real-time webcam capture at 640x480
  - ✓ Face detection using OpenCV Haar Cascades
  - ✓ Pixel analysis with configurable thresholds
  - ✓ Two-case classification (healthy vs alert)

- **Audio Alert System**
  - ✓ Cross-platform audio playback (Windows/macOS/Linux)
  - ✓ Random sound selection for variety
  - ✓ Non-blocking threaded playback
  - ✓ Cooldown timer (0.5 seconds default)
  - ✓ Runtime audio file refresh
  - ✓ Mute toggle

- **Face Tracking and Persistence**
  - ✓ Histogram-based face matching (HSV color space)
  - ✓ Unique ID assignment (incremental: sheep_0001, sheep_0002, ...)
  - ✓ Automatic face image saving with timestamps
  - ✓ Prevents duplicate alerting within same session
  - ✓ Session log export (JSON format)
  - ✓ Known faces database persistence across sessions

- **User Interface**
  - ✓ Live video display with overlays
  - ✓ Color-coded bounding boxes (green=healthy, red=alert, yellow=border)
  - ✓ Face ID labels and statistics
  - ✓ FPS counter
  - ✓ Keyboard controls (q/s/r/m/c)
  - ✓ On-screen instructions and legend

- **Developer Tools**
  - ✓ Comprehensive camera debugging mode (--debug flag)
  - ✓ WSL detection with helpful error messages
  - ✓ Platform-specific device enumeration
  - ✓ Screenshot capture functionality

### Audio Content

**Case 1 (Healthy) - 6 files**
- Confirmation sounds to indicate normal/healthy animals
- Total: ~830 KB

**Case 2 (Alert) - 3 files**
- Warning sounds to alert farmer of potential issues
- Total: ~278 KB

### Known Limitations

1. **Face Detection Method**
   - Currently uses generic human face Haar Cascade detector
   - Not optimized for sheep anatomy
   - May have false positives/negatives depending on angle and lighting
   - Note: The face detector is just used to identify a region of interest; actual health assessment is done by pixel analyzer

2. **Platform Constraints**
   - **WSL (Windows Subsystem for Linux)**: Cannot access USB webcams directly
   - Must run on native Windows, macOS, or Linux for webcam support
   - Python virtual environment appears to be Windows-based (`.exe` files in venv/Scripts/)

3. **Detection Accuracy**
   - Simple threshold-based pixel analysis (not ML-based)
   - Lighting conditions can affect accuracy
   - No validation against veterinary data
   - No support for multiple condition types (only healthy vs alert)

4. **Tracking Limitations**
   - Face matching uses color histogram similarity (may confuse similar-looking sheep)
   - No deep learning or advanced biometric identification
   - Position-based tracking to avoid duplicate IDs in same frame (100px threshold)

## Dependencies and Requirements

### System Requirements

- **Python**: 3.8 or higher
- **Operating System**:
  - Windows (native - recommended for best compatibility)
  - macOS (with webcam)
  - Linux (native with webcam access)
  - **NOT compatible with WSL/WSL2** (no webcam access)
- **Hardware**:
  - USB webcam or built-in camera
  - Speakers or audio output device
  - Sufficient lighting for reliable detection

### Python Dependencies

```
opencv-python >= 4.8.0   # Computer vision and face detection
numpy >= 1.24.0          # Array operations and pixel analysis
```

**Additional system dependencies** (automatically used if available):
- **Windows**: winsound (built-in), PowerShell (for Media Player)
- **macOS**: afplay (built-in)
- **Linux**: aplay, paplay, mpv, or ffplay (one required for audio)

### Optional Tools

- **ffmpeg**: For audio conversion (MP4 → MP3) using `convert_audio.sh`
- **v4l2-ctl** (Linux): Enhanced camera device information

## Installation and Setup

### Basic Setup

```bash
cd sheep_helper

# Create and activate virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # Linux/macOS
# OR
venv\Scripts\activate     # Windows

# Install dependencies
pip install -r requirements.txt
```

### Audio Setup

1. Add audio files to appropriate folders:
   - `case1/` - Healthy confirmation sounds
   - `case2/` - Alert/warning sounds

2. Supported formats: `.wav`, `.mp3`, `.mp4`, `.ogg`, `.flac`

3. Convert MP4 to MP3 if needed:
   ```bash
   chmod +x convert_audio.sh
   ./convert_audio.sh           # Convert only
   ./convert_audio.sh --delete  # Convert and delete MP4s
   ```

### Camera Positioning

- Mount camera to capture the scan area (typically shaved side/back of sheep)
- Ensure consistent lighting for reliable detection
- Test with `python main.py` to verify detection works

## Usage

### Running the Application

```bash
# From Windows (not WSL)
python main.py

# Debug mode (show camera diagnostics)
python main.py --debug
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `q` | Quit and save session log |
| `s` | Save screenshot to screenshots/ folder |
| `r` | Refresh audio files (after adding new sounds) |
| `m` | Toggle mute/unmute |
| `c` | Clear alert history (re-alert on same faces) |

### Output Files

**Known Faces**: `known_faces/sheep_XXXX_YYYYMMDD_HHMMSS.png`
- Automatically saved for each new detected face
- Used for face matching in future sessions

**Session Logs**: `known_faces/session_YYYYMMDD_HHMMSS.json`
- Contains detection statistics
- Per-face case classification history
- Saved on quit

**Screenshots**: `screenshots/screening_YYYYMMDD_HHMMSS.png`
- Manual captures via 's' key
- Includes all overlays and annotations

## Future Improvements (from README)

- [ ] Train custom ML model on sheep-specific data for better detection
- [ ] Add multiple condition classifications (not just healthy/alert)
- [ ] Integration with farm management systems
- [ ] Remote monitoring and alerts
- [ ] Historical trend analysis per animal ID

## Technical Notes

### Why Use Face Detection for Sheep?

The OpenCV Haar Cascade face detector is used as a pragmatic starting point:
- Fast and works in real-time
- Provides consistent bounding boxes for pixel analysis
- Can be replaced with a sheep-specific detector later

**The actual health assessment is done by the pixel analyzer, not the face detector**. The face detector just provides the region of interest to analyze.

### File Naming Convention

```
sheep_XXXX_YYYYMMDD_HHMMSS.png

Where:
- XXXX = 4-digit face ID (0001, 0002, ...)
- YYYYMMDD = Date (20241224 = Dec 24, 2024)
- HHMMSS = Time (143022 = 14:30:22)
```

### Session Tracking

The system maintains two levels of tracking:
1. **Within-session tracking**: `frame_tracks` dict - prevents duplicate alerts for same animal in current session
2. **Across-session tracking**: `known_faces_db` dict - recognizes animals from previous sessions

## Customization Points

### Adjust Detection Thresholds

Edit in `main.py`:
```python
analyzer = PixelAnalyzer(
    darkness_threshold=80,   # Lower = more sensitive to dark pixels
    light_threshold=170      # Higher = stricter for "light" classification
)
```

### Adjust Border Expansion

Edit in `main.py`:
```python
BORDER_PERCENT = 0.10  # 10% expansion around detected face
```

### Audio Cooldown

Edit in `audio_player.py`:
```python
self.cooldown_seconds = 0.5  # Minimum seconds between plays
```

### Face Matching Sensitivity

Edit in `face_tracker.py`:
```python
match_threshold=0.5,        # 0-1, higher = stricter matching
position_threshold=100      # Pixels distance for same-frame tracking
```

## License

Internal/proprietary - for authorized agricultural use only.
