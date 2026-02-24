# Ambot - Known Issues & Troubleshooting

> Last updated: 2026-02-24 (Session 18)

---

## Active Issues

_Currently known problems and workarounds_

### RPi Desktop Login Fails (FIXED - Session 13)

**Symptom**: Typing correct username (`pi`) and password (`erau`) at the LightDM greeter, the desktop doesn't load — it flashes and returns to the login screen.

**Root cause**: Stray `fi` at the end of `~/.profile` (line 29) and `~/.bashrc` (line 115). These were leftover from Session 10 when venv auto-activation was added then reverted. The X session sources `.profile` on login, hits the syntax error, and the session crashes.

**Error** (from `~/.xsession-errors`):
```
/etc/X11/Xsession: 29: /home/pi/.profile: Syntax error: "fi" unexpected
```

**Fix applied**:
```bash
sed -i '29d' ~/.profile    # Remove stray fi
sed -i '115d' ~/.bashrc    # Remove stray fi
sudo systemctl restart lightdm
```

**Prevention**: Always run `bash -n ~/.profile && bash -n ~/.bashrc` after editing shell config files.

---

### Motors Keep Running After Script Exits (FIXED - Session 13)

**Symptom**: After the face tracker crashes, gets killed, or the system shuts off, motors continue spinning indefinitely because GPIO pins stay in their last state.

**Root cause**: Python process exit doesn't automatically reset GPIO pins. If the process is killed with SIGKILL or crashes, the `finally` block may not execute.

**Fix applied**:
1. **Startup stop**: Motors are stopped immediately on initialization (clears previous state)
2. **Signal handlers**: SIGTERM and SIGINT trigger motor stop before exit
3. **atexit handler**: Last-resort cleanup on any Python exit
4. **Watchdog**: Motors auto-stop if no drive command for 2 seconds (handles hangs)
5. **Kill script**: `scripts/kill-hardware.sh` — emergency GPIO reset + process kill

**Emergency fix if motors are spinning**:
```bash
cd ~/ambot && source venv/bin/activate && bash scripts/kill-hardware.sh
```

---

### Motors Not Spinning (Wiring/Power)

**Symptom**: `test_motors.py` shows motors trying to spin (PWM signals sent) but wheels don't turn.

**Status**: User is rewiring motors with proper power supply.

**Likely causes**:
- Insufficient battery voltage (4xAA = ~6V, minus L298N 2V drop = ~4V to motors)
- ENA/ENB jumpers still installed (need to be removed for PWM control)
- Loose terminal screw connections
- Left motor (ENA, Pin 33) wiring issue specifically

**Workaround**: Use `--max-speed` flag to limit PWM duty cycle during testing. Motors are mounted safely so robot won't move during bench testing.

---

### Jetson: Firefox & Chromium Crash on Launch

**Symptom**: Firefox and Chromium crash immediately when opened from the Jetson desktop as the non-root user (`georgejetson`). The browser window appears briefly then closes, or doesn't appear at all.

**Status**: Not investigated yet. Needs root cause analysis.

**Possible causes**:
- GPU driver conflict (NVIDIA driver 540.4.0 may not support browser hardware acceleration properly)
- Snap/Flatpak packaging issues on JetPack/Ubuntu 22.04 for ARM64
- Insufficient shared memory (`/dev/shm`) or GPU memory for browser rendering
- AppArmor or sandboxing restrictions
- Corrupted browser profile in `/home/georgejetson/.mozilla/` or `.config/chromium/`

**Investigation steps** (for next session):
```bash
# Try launching from terminal to see errors
ssh jetson
firefox --no-remote 2>&1 | head -50
chromium-browser --no-sandbox 2>&1 | head -50

# Check if it's a GPU issue
firefox --safe-mode
chromium-browser --disable-gpu

# Check installed browser versions
dpkg -l | grep -iE 'firefox|chrom'
snap list 2>/dev/null | grep -iE 'firefox|chrom'

# Check logs
journalctl --user -u snap.firefox.firefox 2>/dev/null | tail -20
dmesg | tail -30
```

**Workaround**: Use SSH port forwarding to access web applications from the dev machine's browser instead of the Jetson's browser.

```bash
ssh -L 5000:localhost:5000 jetson
# Then open http://localhost:5000 in local browser
```

---

### Web Dashboard: Chat API Endpoint Mismatch (FIXED - Session 18)

**Symptom**: Chat returns `404 Not Found` when asking questions through the web dashboard.

**Cause**: `api_chat.py` used `/api/query` but the RAG API endpoint is `/api/ask`.

**Fix**: Changed endpoint in `web_control/routes/api_chat.py` from `/api/query` to `/api/ask`.

---

### Web Dashboard: Missing `requests` Dependency (FIXED - Session 18)

**Symptom**: Dashboard fails to start with `ModuleNotFoundError: No module named 'requests'`.

**Cause**: `api_chat.py` imports `requests` but it wasn't listed in `requirements.txt`.

**Fix**: Added `requests>=2.28` to `web_control/requirements.txt`.

---

## Resolved Issues

_Previously known issues that have been fixed_

### OpenCV GUI Not Working (FIXED - Session 10)

**Symptom**: `cv2.imshow()` throws "The function is not implemented" error.

**Cause**: pip `opencv-python-headless` has GUI=NONE. Apt `python3-opencv` provides Qt5 backend.

**Fix**: Removed pip headless version, installed apt `python3-opencv` + `opencv-data`.

### Venv Folder Hidden (FIXED - Session 10)

**Symptom**: `.venv` folder not visible in file listings, confusing to find.

**Fix**: Renamed `.venv` → `venv` across all scripts and on RPi.

### verify_all_imports.py Scanning 463 Files (FIXED - Session 10)

**Symptom**: After renaming `.venv` → `venv`, the script counted 463 Python files instead of 52.

**Cause**: `d.startswith('.')` exclusion no longer applied to `venv/`.

**Fix**: Added `d not in ('__pycache__', 'venv')` to directory exclusion.

---

## Diagnostic Commands

```bash
# Check desktop login
ssh pi@10.33.224.1 "tail -20 ~/.xsession-errors"
ssh pi@10.33.224.1 "bash -n ~/.profile && bash -n ~/.bashrc && echo OK"
ssh pi@10.33.224.1 "systemctl status lightdm"

# Check motor GPIO
ssh pi@10.33.224.1 "cd ~/ambot && source venv/bin/activate && python3 tests/test_motors.py --check"

# Check camera
ssh pi@10.33.224.1 "ls -la /dev/video*"

# Check LiDAR
ssh pi@10.33.224.1 "ls -la /dev/ttyUSB*"

# Full environment check
./deploy.sh rpi --test=env

# === Jetson ===

# Check RAG health
./deploy.sh jetson --test=rag-health

# Check web dashboard
./deploy.sh jetson --test=web-status

# Start/stop web dashboard
./deploy.sh jetson --test=web-start
./deploy.sh jetson --test=web-stop

# SSH port forward for browser access
ssh -L 5000:localhost:5000 jetson
# Then open: http://localhost:5000

# Check browser crash logs
ssh jetson "firefox --no-remote 2>&1 | head -20"
ssh jetson "chromium-browser --disable-gpu --no-sandbox 2>&1 | head -20"
```

---

*Add new issues as they're discovered. Move to "Resolved" when fixed.*
