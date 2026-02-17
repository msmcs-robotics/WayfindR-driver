# Motor Orientation Guide

> How camera perspective maps to motor control in the face tracker.

---

## Camera Perspective

The camera feed is **not mirrored** — it shows what the camera sees from its own perspective.

- **Left on screen** = the robot's physical left (when facing the same direction as the camera)
- **Right on screen** = the robot's physical right
- The image center (crosshair) is the tracking target

## Differential Drive Steering

The face tracker uses differential drive — two independently controlled motors:

| Face Position | Desired Turn | Left Motor | Right Motor |
|---------------|-------------|------------|-------------|
| Left of center | Turn LEFT | Slower / reverse | Faster / forward |
| Right of center | Turn RIGHT | Faster / forward | Slower / reverse |
| Center (bbox overlaps origin) | STOP | 0 | 0 |
| No face detected | STOP | 0 | 0 |

**Steering formula** (before orientation adjustments):
```
steer = (face_center_x - frame_center_x) / half_width * gain
left_speed  = base_speed + steer
right_speed = base_speed - steer
```

## Center-Overlap Stop

When the frame center point falls **inside the tracked face's bounding box**, motors stop completely. This means the robot is already oriented toward the face and doesn't need further correction.

This is separate from the dead zone (which only checks horizontal pixel offset).

## Motor Orientation Adjustments

If the physical motors don't match the expected behavior, use these flags:

### CLI Flags

| Flag | Effect |
|------|--------|
| `--swap-motors` | Exchange left/right motor assignments |
| `--invert-left` | Reverse left motor direction (forward ↔ backward) |
| `--invert-right` | Reverse right motor direction (forward ↔ backward) |

### Runtime Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `x` | Toggle swap L/R motors |
| `v` | Toggle invert left motor direction |
| `b` | Toggle invert right motor direction |

Changes take effect immediately — no restart needed.

## Troubleshooting

### Robot turns the WRONG direction

**Symptom**: Face moves right, but robot turns left (or vice versa).

**Fix**: Press `x` (swap motors). The left and right motor assignments are reversed in the wiring.

### One motor always spins backward

**Symptom**: One motor runs backward when it should go forward.

**Fix**: Press `v` (invert left) or `b` (invert right) for the affected motor.

### Robot moves backward instead of forward

**Symptom**: Both motors spin backward when chasing a face.

**Fix**: Use both `--invert-left --invert-right` (or press `v` then `b`).

### Finding the right configuration

1. Start with: `python3 tests/gui_face_tracker.py --motors --max-speed 20`
2. Hold your face in front of the camera
3. Move face to the LEFT — robot should try to turn left
4. If wrong direction → press `x` to swap motors
5. If a motor spins backward when it should go forward → press `v` or `b`
6. Once correct, add the flags to your command for future runs

Example with corrections applied:
```bash
python3 tests/gui_face_tracker.py --motors --max-speed 30 --swap-motors --invert-left
```

## L298N Pin Mapping (BOARD numbering)

| Motor | IN1 | IN2 | EN (PWM) |
|-------|:---:|:---:|:--------:|
| Left (Motor A) | Pin 13 | Pin 15 | Pin 33 |
| Right (Motor B) | Pin 16 | Pin 18 | Pin 32 |
| Ground | Pin 6 | — | — |

The `offset` field in `locomotion/rpi_motors/config.py` can also correct motor direction permanently (set to `-1` to reverse a motor's wiring direction in software).

## How Motor Values Are Applied

```
Algorithm output: left_spd, right_spd (floats, -1.0 to 1.0)
     ↓
Scale to PWM: left_pwm = int(left_spd * max_speed)
     ↓
Apply --swap-motors: swap left_pwm, right_pwm
     ↓
Apply --invert-left/right: negate respective value
     ↓
robot.drive(left_pwm, right_pwm)  → DifferentialDrive → Motor.drive()
     ↓
Motor applies config offset (1 or -1) → GPIO forward/reverse
```

---

*Once you find the correct orientation, you can update `config.py` motor offsets for a permanent fix.*
