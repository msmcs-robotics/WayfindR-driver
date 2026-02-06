# Pre-SLAM Localization: IMU + Reactive Navigation

> Date: 2026-02-06
> Context: Evaluating whether the robot needs position tracking for pre-SLAM wandering demos

## Question

When the robot moves, the LiDAR map shifts. How does it know where it is? Do we need an IMU? Do we need SLAM?

## Key Finding: Reactive Wandering Does NOT Need Position Tracking

The current `NaturalWanderBehavior` is **memoryless with respect to position**. Each cycle:
1. Takes a fresh LiDAR scan (~467 points)
2. Bins into 36 angular buckets for clearance profile
3. Picks top-N open directions
4. Turns toward current target
5. Repeats (targets refresh every 5 seconds)

The robot never compares current scan to previous scans. It doesn't need to know "I was at (2,1) and now I'm at (2.5,1.2)." The environment IS the memory — walls and obstacles encode all needed information.

**Position tracking becomes necessary only when you want to:**
- Return to a specific location (charging station)
- Avoid revisiting areas (exploration coverage)
- Navigate to a goal (go to Room 204)
- Build a persistent map

None of these are requirements for natural-looking wandering.

## What an MPU6050 IMU WOULD Help With

### Gyroscope (High Value)
- **Heading tracking**: Know how much the robot actually turned (vs. hoping motor command duration was right)
- **Turn verification**: Surface friction, battery voltage affect actual turn rate. Gyro doesn't care.
- **Closed-loop turns**: Convert `turn_right(speed)` to `turn_until(heading_delta=30°)`
- **Scan de-skewing**: Correct for robot rotation during 100ms scan time

### Accelerometer (Low Value for Navigation)
- **Tilt detection**: Is robot on a slope? Tipped over?
- **Bump/collision detection**: Catch objects below LiDAR scan plane
- **Position estimation**: DO NOT ATTEMPT. Drift is catastrophic:

| Time | Position Error (1mg bias) |
|------|--------------------------|
| 1 second | ~0.5 cm |
| 10 seconds | ~0.5 m |
| 1 minute | ~17.6 m |
| 5 minutes | ~441 m |

Error grows quadratically (0.5 * a_bias * t^2). This is a fundamental sensor limitation, not a software problem.

## Localization Levels

| Level | What | IMU? | SLAM? | CPU Cost (Pi 3) |
|-------|------|------|-------|-----------------|
| **0 (Current)** | Pure reactive, no heading | No | No | 0 |
| **1 (Recommended)** | Gyro heading for turn control | Yes | No | <1% |
| 2 | LiDAR scan matching (ICP) for position | Optional | No | 10-30% |
| 3 | IMU + scan matching fusion | Yes | No | 10-30% |
| 4 | Full SLAM (ROS2 Nav2/Cartographer) | Yes | Yes | 50-80%+ |

### Level 1: IMU Heading Only (Recommended Next Step)
- Gyroscope integration for heading tracking
- Robot knows "I'm facing 47 degrees from where I started"
- Heading drifts ~1-2 degrees/minute (fine for 5-second target cycles)
- **Computational cost**: Negligible. I2C reads at 100Hz + integration.
- **What it enables**: Turn commands become heading-error-based, not time-based

### Level 2: LiDAR Scan Matching (Future, If Needed)
- ICP (Iterative Closest Point) compares consecutive scans to estimate movement
- Gives both rotation AND translation estimates
- Accuracy: few centimeters per match on structured indoor environments
- **On Pi 3**: Feasible at 10Hz with optimizations (downsample to ~100 points, KD-tree, IMU prior)
- **When to add**: Only when a specific capability requires position (return-to-base, area coverage)

### Level 4: Full SLAM (Not Now)
- ROS2 + Cartographer or SLAM Toolbox
- Pi 3 will struggle (500MB+ RAM for Cartographer)
- Save for Pi 5 or Jetson

## Gyroscope Drift Reality

MPU6050 gyro drift: ~1-2 degrees/minute after calibration.

For wandering (targets refresh every 5s):
- Drift per target cycle: ~0.17 degrees — **negligible**
- After 10 minutes: ~20 degrees — noticeable but manageable (targets refresh anyway)
- After 1 hour: ~120 degrees — unusable without correction

**For wandering behaviors, short-term heading is all we need.** Long-term heading correction options (future):
- Magnetometer (upgrade to MPU9250, same pinout)
- LiDAR scan matching periodic correction
- Heading reset at known locations

## Implementation Recommendation

### Phase 1: Add MPU6050 Driver (Pre-SLAM, Current Milestone)
1. Write I2C driver for MPU6050 (read gyro/accel at ~100Hz)
2. Gyroscope heading integration with startup calibration (keep still 2-3s, average bias)
3. Complementary filter for pitch/roll (accel corrects gyro drift on those axes)
4. Modify `NaturalWanderBehavior` to use heading for turn commands
5. Add tilt/bump detection for safety
6. Graceful degradation: system works without IMU, IMU enhances when present

### Phase 2: Scan Matching (Later, Only If Needed)
- Add ICP-based LiDAR odometry when position tracking is required
- Use IMU heading as prior for ICP convergence
- Downsample scans to ~100 points for Pi 3 performance

### Phase 3: SLAM (When Ready)
- ROS2 integration per roadmap
- All Phase 1/2 work is reusable (IMU driver, heading integration)

## Sources
- MPU6050 gyro drift: ~20-40 deg/hour bias instability (consumer MEMS)
- Accelerometer position drift: quadratic growth from bias, ~0.5*a*t^2
- ICP on ARM: 10-30ms per match with optimizations on modern ARM
- FAST-LIO2, LIO-SAM: reference architectures for LiDAR-inertial odometry
