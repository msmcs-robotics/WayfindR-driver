# Ambot - Known Issues & Troubleshooting

> Last updated: 2026-02-24 (Session 18-19)

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

**Status**: FIXED (Session 20). Installed Firefox ESR from Mozilla PPA (bypasses broken Snap).

**Root cause**: `snap-confine` capability failure on JetPack/Tegra kernel.

The Jetson's JetPack R36.4.4 kernel (`5.15.148-tegra`) does **not** have AppArmor compiled in (`CONFIG_SECURITY_APPARMOR` not set). The `snap-confine` binary (from `snapd` deb package v2.67.1) requires `cap_dac_override` capability, which either isn't granted or the kernel doesn't fully support POSIX capabilities for snap confinement.

**Error** (from `firefox --no-remote 2>&1`):
```
snap-confine is packaged without necessary permissions and cannot continue;
required permitted capability cap_dac_override not found in current capabilities:
=: Function not implemented
```

**Investigation performed** (Session 19):
```bash
# Confirmed snap-confine lacks capabilities
getcap /usr/lib/snapd/snap-confine
# (empty — no capabilities set)

# Confirmed kernel lacks AppArmor
ssh jetson "grep CONFIG_SECURITY_APPARMOR /boot/config-* 2>/dev/null"
# CONFIG_SECURITY_APPARMOR is not set

# snapd version mismatch
dpkg -l snapd | grep snapd  # deb: 2.67.1
snap version                 # snapd: 2.73

# Attempted fix: setcap (did NOT resolve the issue)
sudo setcap cap_dac_override+ep /usr/lib/snapd/snap-confine
# Capability was set but error persists — kernel may not support it properly
```

**Note**: User reports browsers may have been installed via APT, not Snap. Check with:
```bash
which firefox && dpkg -l firefox 2>/dev/null   # APT-installed?
snap list firefox 2>/dev/null                    # Snap-installed?
```

**Fix applied** (Session 20):
```bash
# Install Firefox ESR from Mozilla PPA (bypasses broken Snap)
sudo add-apt-repository -y ppa:mozillateam/ppa
sudo apt update
sudo apt install -y firefox-esr

# Set as default browser
sudo update-alternatives --set x-www-browser /usr/bin/firefox-esr
sudo update-alternatives --set gnome-www-browser /usr/bin/firefox-esr
```

Firefox ESR 140.8.0 now works. The Snap `firefox` package is still installed but broken — use `firefox-esr` instead.

**Alternative**: Use SSH port forwarding to access web applications from the dev machine's browser:
```bash
ssh -f -N -L 5123:localhost:5000 jetson
# Then open http://localhost:5123 in local browser
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

# SSH port forward for browser access (use 5123 if 5000 is occupied locally)
ssh -f -N -L 5123:localhost:5000 jetson
# Then open: http://localhost:5123
# To kill the tunnel later: pkill -f 'ssh.*5123.*jetson'

# Check browser crash logs
ssh jetson "firefox --no-remote 2>&1 | head -20"
ssh jetson "chromium-browser --disable-gpu --no-sandbox 2>&1 | head -20"
```

---

*Add new issues as they're discovered. Move to "Resolved" when fixed.*
