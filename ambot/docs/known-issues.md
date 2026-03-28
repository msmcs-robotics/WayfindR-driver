# Ambot - Known Issues & Troubleshooting

> Last updated: 2026-03-28 (Session 23)

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

### Jetson: Firefox & Chromium Crash on Launch (FULLY FIXED - Session 23)

**Symptom**: Clicking Firefox or Chromium from the GNOME dock/app menu does nothing — browser window never appears. Both snap-packaged browsers are completely broken.

**Status**: **FULLY FIXED** (Session 23). Firefox ESR installed, all desktop integration fixed.

**Root cause**: Two-layer failure:

1. **snap-confine capability failure** — JetPack R36.4.4 kernel (`5.15.148-tegra`) does NOT have AppArmor compiled in. The `snap-confine` binary requires `cap_dac_override` capability, which the Tegra kernel doesn't fully support for snap confinement. Installing `policycoreutils` (provides `matchpathcon`) was attempted but did NOT resolve the issue — this is a kernel-level limitation.

2. **GNOME dock pointed to broken snap** — The favorites dock had `firefox_firefox.desktop` pinned, which calls `/snap/bin/firefox`. Even after installing firefox-esr, clicking the dock icon still tried the broken snap version.

**Error** (from `firefox --no-remote 2>&1`):
```
WARNING: cannot create user data directory: failed to verify SELinux context
snap-confine is packaged without necessary permissions and cannot continue
required permitted capability cap_dac_override not found in current capabilities:
  =: Function not implemented
```

**Why snap is unfixable on Jetson**: The Tegra kernel was compiled without `CONFIG_SECURITY_APPARMOR`. Snap requires either AppArmor or proper POSIX capabilities for confinement. Neither is available. `setcap cap_dac_override+ep` on snap-confine doesn't work because the kernel's capabilities implementation is non-standard for the snap runtime. This is a JetPack/NVIDIA limitation — **snap browsers will never work on this kernel**.

**Complete fix** (Session 23 — apply all steps):
```bash
# Step 1: Install Firefox ESR (deb package, bypasses snap entirely)
sudo add-apt-repository -y ppa:mozillateam/ppa
sudo apt update
sudo apt install -y firefox-esr

# Step 2: Set firefox-esr as default browser
sudo update-alternatives --install /usr/bin/x-www-browser x-www-browser /usr/bin/firefox-esr 200
sudo update-alternatives --set x-www-browser /usr/bin/firefox-esr
sudo update-alternatives --set gnome-www-browser /usr/bin/firefox-esr
xdg-settings set default-web-browser firefox-esr.desktop
xdg-mime default firefox-esr.desktop text/html
xdg-mime default firefox-esr.desktop x-scheme-handler/http
xdg-mime default firefox-esr.desktop x-scheme-handler/https

# Step 3: Redirect /usr/bin/firefox to firefox-esr
sudo cp /usr/bin/firefox /usr/bin/firefox.snap-original  # backup
sudo tee /usr/bin/firefox > /dev/null << 'EOF'
#!/bin/bash
exec /usr/bin/firefox-esr "$@"
EOF
sudo chmod +x /usr/bin/firefox

# Step 4: Update GNOME favorites dock (replace snap entry)
# Run this as the desktop user (georgejetson), not root:
gsettings set org.gnome.shell favorite-apps \
  "['firefox-esr.desktop', 'org.gnome.Nautilus.desktop', 'snap-store_ubuntu-software.desktop', 'yelp.desktop']"

# Step 5: Patch snap desktop entries (so any remaining references work)
sudo sed -i 's|Exec=/snap/bin/firefox|Exec=/usr/bin/firefox-esr|g' \
  /var/lib/snapd/desktop/applications/firefox_firefox.desktop
sudo sed -i 's|Icon=/snap/firefox/current/default256.png|Icon=firefox-esr|g' \
  /var/lib/snapd/desktop/applications/firefox_firefox.desktop

# Step 6: Refresh desktop database
sudo update-desktop-database /usr/share/applications/
sudo update-desktop-database /var/lib/snapd/desktop/applications/

# Step 7: Verify
firefox --version   # Should show: Mozilla Firefox 140.8.0esr
xdg-settings get default-web-browser  # Should show: firefox-esr.desktop
```

**If GNOME dock icon doesn't update**: Press `Alt+F2`, type `r`, press Enter to restart GNOME Shell.

**Investigation commands** (for diagnosing if this recurs):
```bash
# Check what's in the GNOME favorites dock
gsettings get org.gnome.shell favorite-apps

# Check which firefox binary runs
which firefox && file $(which firefox)
firefox --version

# Check what desktop entries exist
find /usr/share/applications /var/lib/snapd/desktop/applications \
  -name '*firefox*' -o -name '*chrom*' 2>/dev/null

# Check snap status
snap list firefox 2>/dev/null
snap list chromium 2>/dev/null

# Test firefox-esr headless (works even over SSH)
timeout 5 firefox-esr --headless --screenshot /tmp/test.png about:blank 2>&1
ls -la /tmp/test.png  # Should exist if browser works

# Check if snap-confine has capabilities (it won't on Tegra)
getcap /usr/lib/snapd/snap-confine
```

**Alternative**: Use SSH port forwarding to access Jetson web apps from the dev machine's browser:
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

# Check browser (should be firefox-esr)
ssh jetson "firefox --version"
ssh jetson "gsettings get org.gnome.shell favorite-apps"

# If browser breaks again, re-run the fix script:
# scp /tmp/fix-jetson-browser-gui2.sh jetson:/tmp/ && ssh jetson 'bash /tmp/fix-jetson-browser-gui2.sh'
```

---

*Add new issues as they're discovered. Move to "Resolved" when fixed.*
