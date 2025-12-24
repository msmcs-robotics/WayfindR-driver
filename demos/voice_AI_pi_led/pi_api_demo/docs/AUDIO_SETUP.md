# Audio Setup - Raspberry Pi 4

Documentation of audio setup findings for the Raspberry Pi.

## Final Configuration: Built-in 3.5mm AUX Jack

After troubleshooting the RASPIAUDIO Ultra V3 HAT, we switched to using the built-in 3.5mm headphone jack for simplicity and reliability.

### Working Configuration

```python
AUDIO_DEVICE = "plughw:0,0"  # Built-in 3.5mm AUX output
```

### Audio Devices Available

```
$ aplay -l
card 0: Headphones [bcm2835 Headphones]    ← USING THIS
card 1: vc4hdmi0 [vc4-hdmi-0]              (HDMI 0)
card 2: vc4hdmi1 [vc4-hdmi-1]              (HDMI 1)
```

### Volume Configuration

```bash
# Set volume to maximum
amixer -c 0 set PCM 100%

# Force audio output to headphone jack (not HDMI)
sudo raspi-config nonint do_audio 1
```

## TTS Pipeline

### Selected Solution: espeak-ng + sox

```bash
# 1. Generate speech (22050 Hz mono)
espeak-ng -v en-us "Hello world" -w /tmp/speech.wav

# 2. Convert to 48000 Hz stereo (required for best compatibility)
sox /tmp/speech.wav -r 48000 -c 2 /tmp/speech48.wav

# 3. Play through headphone jack
aplay -D plughw:0,0 /tmp/speech48.wav
```

### Why This Pipeline?

| Step | Reason |
|------|--------|
| espeak-ng | Fast, reliable, works offline, no dependencies |
| sox conversion | bcm2835 works best with 48kHz stereo |
| plughw:0,0 | Direct hardware access with format conversion |

### TTS Options Evaluated

| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **espeak-ng** | Simple, fast, works | Robotic voice | **CHOSEN** |
| Piper TTS | Neural quality | Binary compatibility issues | Not used |
| Festival | Moderate quality | Complex setup | Not used |

## RASPIAUDIO Ultra V3 HAT (NOT USED)

We attempted to use the RASPIAUDIO Ultra V3 HAT but encountered persistent issues.

### What We Tried

1. **HAT Detection**: The WM8960 codec was detected correctly
   ```
   card 3: wm8960soundcard [wm8960-soundcard]
   ```

2. **Config.txt Changes**:
   ```
   dtparam=i2s=on
   dtparam=audio=on
   dtoverlay=wm8960-soundcard
   ```

3. **ALSA Mixer Settings**:
   ```bash
   amixer -c 3 cset numid=51 on  # PCM Playback
   amixer -c 3 cset numid=54 on  # Left/Right Output Mixer
   ```

### Issues Encountered

1. **Clock Configuration Failures**:
   ```
   wm8960 1-001a: failed to configure clock
   snd_soc_wm8960: ASoC: error at snd_soc_dai_hw_params on wm8960-hifi
   ```

2. **No Audio Output**: Despite successful aplay commands (no errors), no sound came from the HAT speakers.

3. **Blue Button**: The HAT has a blue button between the speakers - pressing it had no effect.

### Decision

Abandoned the HAT in favor of the simpler, working 3.5mm jack solution.

## Quick Test Commands

```bash
# Test if audio works
speaker-test -D plughw:0,0 -c 2 -t sine -f 440 -l 1

# Test TTS
espeak-ng "Testing audio" -w /tmp/t.wav && \
sox /tmp/t.wav -r 48000 -c 2 /tmp/t48.wav && \
aplay -D plughw:0,0 /tmp/t48.wav

# Check volume
amixer -c 0 sget PCM
```

## Troubleshooting

### No Sound At All

1. Check speaker is connected to 3.5mm jack (not HDMI)
2. Force output to headphone: `sudo raspi-config nonint do_audio 1`
3. Set volume: `amixer -c 0 set PCM 100%`
4. Test: `speaker-test -D plughw:0,0 -c 2 -t sine -f 440 -l 1`

### Sound Goes to HDMI Instead

```bash
# Force to headphone jack
sudo raspi-config nonint do_audio 1
# Or edit /boot/config.txt and add:
# dtparam=audio=on
# Then reboot
```

### espeak-ng Not Installed

```bash
sudo apt-get install espeak-ng sox
```

## References

- [Raspberry Pi Audio Configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#audio)
- [espeak-ng Documentation](https://github.com/espeak-ng/espeak-ng)
- [SoX Audio Processing](http://sox.sourceforge.net/)
