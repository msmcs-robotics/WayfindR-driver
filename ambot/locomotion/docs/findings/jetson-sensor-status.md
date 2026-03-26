# Jetson Orin Nano — Sensor Status

**Date:** 2026-03-26 (Session 21)
**Target:** georgejetson@10.33.155.83

## USB Devices Connected

```
Bus 001 Device 004: ID 328f:00af  EMEET SmartCam S600    ← Camera
Bus 001 Device 005: ID 10c4:ea60  Silicon Labs CP210x     ← LiDAR LD19
Bus 001 Device 003: ID 13d3:3549  IMC Networks Bluetooth  ← Built-in
```

## Camera — EMEET SmartCam S600

- **Device:** `/dev/video0`, `/dev/video1`
- **Status:** WORKING
- **Resolution:** 640x480 confirmed via OpenCV
- **OpenCV:** GStreamer backend (built-in on Jetson)
- **Warning:** `Cannot query video position` — harmless GStreamer info message
- **Permissions:** `/dev/video0` owned by `root:video`, user in `video` group — OK

```python
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()  # OK: 640x480
```

## LiDAR — LD19 (via CP210x USB-UART bridge)

- **Device:** `/dev/ttyUSB0`
- **Baud:** 230400
- **Status:** DETECTED but needs permission fix
- **Issue:** `/dev/ttyUSB0` owned by `root:dialout`, user NOT in `dialout` group
- **Fix applied:** `sudo usermod -aG dialout georgejetson` (needs re-login to take effect)
- **pyserial:** Installed (`pip3 install pyserial`)

After re-login:
```python
import serial
s = serial.Serial("/dev/ttyUSB0", 230400, timeout=1)
data = s.read(200)  # Should get ~200 bytes of scan data
```

## GPIO / Motors — L298N (not yet wired)

- **Jetson.GPIO:** 2.1.7 installed, BOARD pins 32+33 PWM verified OK
- **Pinout doc:** `ambot/locomotion/docs/l298n-jetson-pinout-guide.md`
- **Driver code:** `ambot/locomotion/jetson_motors/driver.py`
- **Waiting on:** Physical L298N wiring to Jetson header

## Summary

| Sensor | Device | Status | Action Needed |
|--------|--------|--------|---------------|
| Camera (EMEET S600) | /dev/video0 | WORKING | None |
| LiDAR (LD19) | /dev/ttyUSB0 | DETECTED | Re-login for dialout group |
| L298N motors | GPIO 32,33 | PWM VERIFIED | Physical wiring needed |
| IMU (MPU6050) | I2C bus 1 | NOT CONNECTED | Wire to pins 3,5 |
