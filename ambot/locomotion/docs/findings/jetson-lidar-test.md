# Jetson Orin Nano LD19 LiDAR Test Results

**Date:** 2026-03-26
**Target:** Jetson Orin Nano (`ssh jetson` / georgejetson@10.33.155.83)
**Device:** LD19 LiDAR at /dev/ttyUSB0, 230400 baud (CP210x USB-UART bridge)

## Summary

All tests passed. The LD19 LiDAR works on the Jetson with zero issues. The existing
`pathfinder/lidar_ld19.py` driver runs unmodified and produces scan data identical in
quality to the RPi setup.

## Step 1: Device Verification

```
Device exists: True
Readable: True
Writable: True
Permissions: crw-rw---- 1 root dialout 188, 0
User groups: georgejetson is in 'dialout' group
pyserial version: 3.5
```

No permission issues. No udev rules needed.

## Step 2: Raw Serial Data

```
Total bytes received: 39936 (in 2 seconds)
Bytes/sec: 19577
0x54 header bytes found: 919
```

Sample raw bytes (two consecutive packets):
```
ff 96 00 ff 97 00 ff 97 00 ff 98 00 ff 98 00 ff 99 00 ff b6 2f a0 42 5e
54 2c 0c 0e f0 2f 9a 00 ff 9c 00 ff a0 00 ff a8 00 ff b1 00 ff c0 00 ...
```

The 0x54 header bytes appear at regular intervals, confirming the LD19 is streaming
its auto-broadcast protocol.

## Step 3: Packet Parsing

Parsed 4096 bytes of raw data:

```
Valid packets: 86
CRC failures: 0
Invalid/skipped bytes: ~54

Sample parsed packets:
  Packet 1: start=258.6 deg, end=266.5 deg
    Points: [(83mm, i=232), (85mm, i=231), (86mm, i=229), ...]
  Packet 2: start=267.2 deg, end=274.9 deg
    Points: [(105mm, i=237), (107mm, i=247), (114mm, i=255), ...]
  Packet 3: start=275.6 deg, end=283.5 deg
    Points: [(125mm, i=255), (128mm, i=255), (131mm, i=255), ...]
```

**Zero CRC failures** across all 86 packets -- clean serial link.

## Step 4: Scan Collection (5 seconds)

```
Complete scans: 49
Points/scan: avg=500, min=498, max=502
Total valid points: 24506
Scan rate: 9.8 scans/sec
```

This is the raw point count before the driver's distance filtering (100mm-12000mm).
The LD19 runs at approximately 10 Hz, consistent with spec.

## Step 5: Existing Driver Test (lidar_ld19.py)

The file `pathfinder/lidar_ld19.py` was rsynced to the Jetson and tested. It works
unmodified with no code changes required.

```
Info: {'model': 'LD19', 'port': '/dev/ttyUSB0', 'baudrate': 230400}
Health: ('Good', 0)
Connected: True

Scan 1: 339 pts | closest=102mm@160.1 deg | farthest=1181mm@89.2 deg
  Angle range: 9.3 - 359.9 deg
  Avg quality: 220
Scan 2: 351 pts | closest=101mm@160.1 deg | farthest=1208mm@90.8 deg
  Angle range: 0.6 - 360.0 deg
  Avg quality: 221
Scan 3: 351 pts | closest=102mm@160.0 deg | farthest=1213mm@91.0 deg
  Angle range: 0.7 - 359.4 deg
  Avg quality: 221

Scans collected: 20 (in 2.0s)
Avg points/scan: 350 (after distance filtering)
Scan rate: 10.0 scans/sec
DRIVER STATUS: WORKING
```

The ~350 filtered points per scan (vs ~500 raw) is due to the driver discarding points
with distance=0 (no reflection) and points outside 100mm-12000mm range. This is normal
and matches the RPi behavior (~467 raw, fewer after filtering depending on environment).

## Comparison: Jetson vs RPi

| Metric | Jetson | RPi (from memory) |
|---|---|---|
| Scan rate | 10.0 Hz | ~10 Hz |
| Raw pts/scan | 498-502 | ~467 |
| Filtered pts/scan | ~350 | varies |
| CRC errors | 0 | 0 |
| Driver changes needed | None | N/A (original) |
| Serial latency | No issues | No issues |

The Jetson gets slightly more raw points per scan (500 vs 467), likely due to
fewer USB scheduling delays on the faster CPU.

## Conclusion

- The LD19 LiDAR is fully operational on the Jetson Orin Nano
- The existing `pathfinder/lidar_ld19.py` driver works with zero modifications
- Serial permissions are correct (user is in dialout group)
- No additional packages or configuration needed beyond pyserial
- Ready for integration with Jetson-side navigation/SLAM code
