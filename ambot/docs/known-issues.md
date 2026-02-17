# Ambot - Known Issues & Troubleshooting

> Last updated: 2026-02-17 (Session 13)

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
```

---

*Add new issues as they're discovered. Move to "Resolved" when fixed.*
