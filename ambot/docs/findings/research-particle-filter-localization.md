# Particle Filter Localization Research for AMBOT

**Date:** 2026-02-19
**Target hardware:** Raspberry Pi 3B (Cortex-A53, 906MB RAM)
**Sensors:** LD19 LiDAR (~467 points/scan, 360 deg), MPU6050 IMU (optional gyro heading)
**Constraint:** No ROS2, standalone Python, no wheel encoders

---

## 1. How Particle Filter Localization Works with 2D LiDAR

### The Core Idea

Particle filter localization (also called Monte Carlo Localization / MCL) estimates a
robot's position by maintaining a cloud of "guesses" (particles), each representing a
possible pose (x, y, theta). At each timestep:

1. **Predict (Motion Update):** Move each particle according to a motion model. With
   wheel encoders this uses odometry; without them, you need an alternative (see
   Section 7). Add noise to account for uncertainty.

2. **Update (Sensor Model):** Compare each particle's *expected* LiDAR scan (ray-cast
   into a known map) against the *actual* LiDAR scan. Particles whose expected scans
   match reality get high weights; particles in wrong locations get low weights.

3. **Resample:** Duplicate high-weight particles and discard low-weight ones. Over
   time, particles converge to the true position.

### What It Needs

| Component | Required? | Notes |
|-----------|-----------|-------|
| Pre-built map | **YES** for pure localization (MCL) | Occupancy grid or similar |
| Motion estimate | **YES** | Encoders, IMU, scan matching, or motor commands |
| LiDAR scans | **YES** | Used to weight particles against the map |
| Known initial position | Helpful but not required | "Global localization" spreads particles everywhere |

### Key Distinction: Localization vs SLAM

- **Localization (MCL):** Robot knows the map, estimates its position within it. Simpler.
- **SLAM:** Robot builds the map AND localizes simultaneously. Harder, more memory.
- **FastSLAM:** Uses particle filter for the robot pose + per-particle EKF for landmarks.
  Each particle carries its own map. Memory scales as `particles x map_size`.

---

## 2. Available Python Implementations

### 2a. Pip-Installable Packages

| Package | What It Does | Particle Filter? | Status | Notes |
|---------|-------------|-------------------|--------|-------|
| **filterpy** | Kalman filters, EKF, UKF | **No** PF included | Last release 2018 | Good for EKF but not what we need |
| **pfilter** | Generic particle filter | **Yes** | Active (GitHub: johnhw/pfilter) | NumPy only. Generic framework, no LiDAR-specific code. You write custom observation/weight functions. |
| **breezyslam** | Full 2D LiDAR SLAM | RMHC (not traditional PF) | Active (GitHub: simondlevy/BreezySLAM) | **Best candidate.** C extensions with SIMD (ARM NEON). Custom LiDAR sensor classes. See Section 2c. |

### 2b. GitHub Repositories -- Standalone Particle Filters

| Repository | Stars | Description | Practical? |
|------------|-------|-------------|------------|
| **AtsushiSakai/PythonRobotics** | 28.7k | FastSLAM 1.0 (100 particles, landmark-based), PF localization (100 particles, RFID beacons) | Educational. Not LiDAR-ready but excellent reference code. |
| **Tinker-Twins/Particle-Filter** | 2 | MCL with RangeLibc for fast ray casting | **ROS-based** -- not standalone. But RangeLibc library is interesting. |
| **ClayFlannigan/icp** | 637 | Standalone ICP (NumPy + sklearn), m-dimensional | Excellent for scan matching. 2D ready. |
| **Davidwarchy/particle-filters** | 0 | Simple 1D/2D particle filter simulation | Educational only. Single distance sensor. |
| **malekzahedi/particle-filter-localization** | 0 | Mobile robot MCL in Python | Educational, simple. |

### 2c. BreezySLAM -- The Best Standalone Option

**BreezySLAM** (simondlevy/BreezySLAM) is the most practical choice for this project:

- **No ROS required** -- standalone Python with C extensions
- **SLAM built in** -- builds occupancy grid map AND localizes simultaneously
- **Custom LiDAR support** -- define LD19 by subclassing `Laser`:

```python
from breezyslam.sensors import Laser

class LD19(Laser):
    def __init__(self):
        Laser.__init__(self,
            scan_size=467,              # points per scan
            scan_rate_hz=10,            # scan frequency
            detection_angle_degrees=360, # full rotation
            distance_no_detection_mm=12000,  # 12m max range
            detection_margin=0,
            offset_mm=0
        )
```

- **Algorithm:** RMHC_SLAM (Random-Mutation Hill-Climbing) based on CoreSLAM
  - NOT a traditional particle filter -- uses single-point position with stochastic search
  - 1000 random position mutations per update, keeps best match
  - Faster and lower memory than particle clouds
  - Default map: 800x800 pixels over 32m (4cm/pixel)
- **Odometry:** Optional. Works without it, but better with it.
- **Performance:** C extensions with ARM NEON SIMD. Should run on RPi 3B.
- **Installation:** `cd python && sudo python3 setup.py install`

**Limitations:**
- Supported LiDARs are URG04LX, XV Lidar, RPLidar A1 -- but custom class is trivial
- Python 3 required (we have 3.13, may need compatibility testing)
- Some users report build issues on newer systems (Issue #84)
- No pip install -- must build from source (C extension compilation)

### 2d. PythonRobotics -- Educational Reference

The PythonRobotics repository provides excellent reference implementations:

- **Particle Filter Localization:** 100 particles, RFID-beacon-based, low-variance resampling
- **FastSLAM 1.0:** 100 particles, per-particle EKF landmark tracking
- **LiDAR-to-Grid-Map:** Bresenham ray casting, configurable resolution (default 2cm/pixel)
- **ICP:** Iterative closest point for scan matching

All use only NumPy/SciPy/Matplotlib. Great for understanding algorithms before deploying.

---

## 3. Does It Need a Pre-Built Map?

### Option A: Pure Localization (MCL) -- YES, needs map first

You provide an occupancy grid map. The particle filter localizes within it.
- Simpler algorithm
- Lower memory (one map + N particles with just pose)
- Faster per-update
- **Problem:** You need to build the map somehow first

### Option B: SLAM -- NO, builds map incrementally

Simultaneously builds the map and localizes.
- BreezySLAM does this out of the box
- FastSLAM: each particle maintains its own map copy (memory-heavy)
- BreezySLAM RMHC: single map, single position estimate with stochastic refinement (memory-light)

### Option C: Incremental Grid Building (simplest)

Build an occupancy grid from LiDAR scans as the robot moves:
1. Robot moves, takes LiDAR scan
2. Convert polar scan to Cartesian points
3. Ray-cast from robot position through free space to obstacles
4. Update occupancy grid cells (Bayesian update or simple counting)
5. Use ICP scan matching between consecutive scans to estimate motion

This is essentially "poor man's SLAM" and is the most practical starting point.

### Recommendation for AMBOT

**Start with Option C** (incremental grid + ICP scan matching), then consider
BreezySLAM (Option B) if you need better accuracy. Pure MCL (Option A) only makes
sense once you have a saved map of a known environment.

---

## 4. Memory Requirements

### Per-Particle Memory

A particle in MCL stores only a pose:
```
x (float64):     8 bytes
y (float64):     8 bytes
theta (float64): 8 bytes
weight (float64): 8 bytes
--------------------------
Total per particle: 32 bytes
```

For FastSLAM, each particle also carries a map:
```
800x800 occupancy grid (uint8): 640 KB per particle
+ pose + landmarks: ~1 KB per particle
```

### How Many Particles Fit in 906MB?

| Approach | Per Particle | Particles in 500MB* | Recommended |
|----------|-------------|---------------------|-------------|
| MCL (pose only) | 32 bytes | ~15 million | 100-1000 |
| FastSLAM (with map) | ~641 KB | ~780 | 10-50 |
| BreezySLAM RMHC | N/A (single point) | N/A | 1000 mutations |

*Leaving ~400MB for OS, Python, numpy, LiDAR driver, etc.

### Occupancy Grid Map Memory

| Resolution | 10m x 10m room | 20m x 20m area | 50m x 50m building |
|------------|---------------|-----------------|---------------------|
| 5 cm/cell | 200x200 = 40 KB | 400x400 = 160 KB | 1000x1000 = 1 MB |
| 2 cm/cell | 500x500 = 250 KB | 1000x1000 = 1 MB | 2500x2500 = 6.25 MB |
| 1 cm/cell | 1000x1000 = 1 MB | 2000x2000 = 4 MB | 5000x5000 = 25 MB |

### Practical Memory Budget for RPi 3B (906MB total)

```
Linux OS + services:     ~200 MB
Python 3.13 interpreter:  ~30 MB
NumPy + SciPy:            ~50 MB
LiDAR driver + camera:    ~50 MB
OpenCV (if loaded):       ~80 MB
------------------------------------
Available for SLAM:      ~400-500 MB

Occupancy grid (5cm, 20m): ~160 KB
100 MCL particles:         ~3.2 KB
LiDAR scan buffer:         ~4 KB (467 x float32 x 2)
ICP working memory:        ~100 KB
------------------------------------
SLAM working set:          <1 MB (for MCL)
                           ~65 MB (for 100 FastSLAM particles)
```

**Verdict: Memory is NOT a bottleneck.** Even 1000 MCL particles with a large
occupancy grid fits easily. FastSLAM with per-particle maps is the only approach
that could strain memory, and even that is feasible with <100 particles.

---

## 5. CPU Requirements

### Cortex-A53 Performance Profile
- 4 cores @ 1.2 GHz (RPi 3B)
- No NEON SIMD in A53 (A53 has it but limited vs A72)
- Single-core Python (GIL limits parallelism)
- NumPy can use multiple cores for large array ops

### Per-Update Computation

| Operation | Complexity | Est. Time on A53 | Notes |
|-----------|-----------|-------------------|-------|
| Motion prediction (100 particles) | O(N) | <0.1 ms | Trivial |
| Ray casting (100 particles x 467 rays) | O(N x R x map) | 10-50 ms | **Bottleneck** |
| Weight calculation | O(N x R) | 1-5 ms | Compare expected vs actual |
| Resampling | O(N) | <0.1 ms | Low-variance resampling |
| ICP scan matching (467 pts) | O(P log P x iters) | 5-20 ms | ~10 iterations typical |
| Occupancy grid update (467 rays) | O(R x cells_per_ray) | 1-5 ms | Bresenham ray casting |

### Can It Run at 5-10 Hz?

| Approach | Est. Total per Update | Achievable Rate | Verdict |
|----------|----------------------|-----------------|---------|
| ICP scan matching only | 5-20 ms | **50+ Hz** | Easy |
| MCL with 100 particles | 15-60 ms | **15-60 Hz** | Yes |
| MCL with 500 particles | 50-250 ms | **4-20 Hz** | Borderline |
| MCL with 1000 particles | 100-500 ms | **2-10 Hz** | Tight |
| BreezySLAM RMHC (C ext) | 10-30 ms | **30-100 Hz** | Easy (C code) |
| FastSLAM 100 particles | 50-200 ms | **5-20 Hz** | Yes |

**Verdict: 5-10 Hz is achievable** with 100-500 particles or BreezySLAM. The key
optimization is ray casting -- BreezySLAM's C extensions with SIMD handle this much
faster than pure Python. For pure Python, keep particles under 200.

### Optimization Tips for RPi 3B
- Use C extensions (BreezySLAM) or Cython for ray casting
- Downsample LiDAR to ~100 points for ray casting comparison (every 4th point)
- Use coarse grid resolution (5cm not 1cm)
- Pre-compute ray lookup tables
- Run LiDAR reading in a separate thread (it's I/O bound)

---

## 6. Particle Filter vs ICP Scan Matching

### ICP Scan Matching

**How it works:** Aligns consecutive LiDAR scans to compute relative motion
(translation + rotation) between them. No map needed.

**Pros:**
- Very simple (50-100 lines of Python)
- Fast (~5-20ms for 467 points)
- No pre-built map required
- Works without encoders
- NumPy + sklearn only (ClayFlannigan/icp)

**Cons:**
- **Drift accumulates** -- small errors compound over time
- No loop closure (can't recognize "I've been here before")
- Fails in featureless environments (long hallways)
- Fails with large movements between scans (needs overlap)
- Gives only relative motion, not absolute position

### Particle Filter (MCL)

**Pros:**
- Global localization (can determine position from scratch)
- No drift (always compared against absolute map)
- Handles kidnapped robot problem
- Robust to sensor noise
- Can recover from wrong estimates

**Cons:**
- Requires a pre-built map
- Ray casting is compute-heavy
- More complex to implement
- More particles = more CPU

### BreezySLAM (RMHC)

**Pros:**
- Builds map AND localizes (no pre-built map needed)
- C extensions are fast
- Works with or without odometry
- Simple API: `slam.update(scan)` / `slam.getpos()`
- Battle-tested, published algorithm (CoreSLAM)

**Cons:**
- Single-point estimate (not a full particle distribution)
- Can get stuck in local minima
- Not as robust as full MCL for global localization
- Requires C extension compilation on RPi

### Comparison Summary

| Feature | ICP Scan Match | Particle Filter (MCL) | BreezySLAM (RMHC) |
|---------|---------------|----------------------|-------------------|
| Needs pre-built map | No | Yes | No (builds it) |
| Drift | Yes (accumulates) | No | Minimal |
| CPU load | Low | Medium-High | Low (C ext) |
| Memory | Minimal | Low-Medium | Medium (map) |
| Complexity | Simple | Complex | Medium |
| Without encoders | Works (scan-to-scan) | Needs motion estimate | Works |
| Best for | Short-term motion | Known environments | **General SLAM** |

### Recommendation

**For AMBOT, the practical progression is:**

1. **Phase 1 (Now):** ICP scan matching for relative motion estimation.
   Pure Python, simple, proves the concept. Integrate with current wandering demo.

2. **Phase 2 (Next):** BreezySLAM for incremental mapping.
   Define LD19 sensor class, feed scans, get position + map. Can work without
   encoders.

3. **Phase 3 (Later):** Full MCL on saved BreezySLAM maps for robust re-localization.

---

## 7. Practical Without Wheel Encoders?

This is the critical question for AMBOT, which has no wheel encoders.

### Motion Estimation Alternatives (Without Encoders)

| Method | How It Works | Quality | Difficulty |
|--------|-------------|---------|------------|
| **ICP scan matching** | Align consecutive LiDAR scans | Good (short-term) | Easy |
| **Motor command dead reckoning** | "I commanded 40% forward for 0.1s" | Poor (no feedback) | Easy |
| **IMU integration** | Integrate gyro for heading, accel for position | Heading: Good. Position: Terrible. | Medium |
| **ICP + IMU fusion** | ICP for translation, gyro for heading | **Best available** | Medium |
| **Scan-to-map matching** | Match current scan against occupancy grid | Good (if map exists) | Hard |

### ICP as Odometry Substitute (Recommended)

This is the most practical approach for AMBOT:

```
For each new LiDAR scan:
  1. Run ICP(previous_scan, current_scan)
  2. Get transformation: (dx, dy, dtheta)
  3. This IS your odometry estimate
  4. Feed to particle filter / BreezySLAM as pose_change
  5. Update occupancy grid
  6. Store current scan as previous_scan
```

**Advantages:**
- No hardware changes needed (already have LD19)
- Works well for small movements (robot moves slowly)
- LD19 at 10Hz means ~0.1s between scans -- small displacement
- 467 points gives good feature matching

**Limitations:**
- Fails if robot spins in place in a featureless area
- Needs some structure in the environment (walls, furniture)
- Drift over long trajectories (but acceptable for room-scale)

### IMU Fusion Improvement

Adding MPU6050 gyro heading significantly helps:

```
For each new LiDAR scan:
  1. Read gyro-integrated heading change: dtheta_imu
  2. Run ICP(previous_scan, current_scan)
  3. Get ICP result: (dx_icp, dy_icp, dtheta_icp)
  4. Fuse: dtheta = 0.7 * dtheta_imu + 0.3 * dtheta_icp
  5. Use (dx_icp, dy_icp, dtheta_fused) as motion estimate
```

The gyro provides reliable rotation estimates (where ICP can struggle), while ICP
provides translation estimates (where accelerometer integration fails badly).

### Verdict

**Yes, it is practical without wheel encoders**, using ICP scan matching as the
odometry source. The LD19 at 10Hz with 467 points provides enough data for
frame-to-frame scan matching. Adding the MPU6050 gyro for heading makes it
significantly more robust.

---

## 8. Occupancy Grid Implementations Without ROS

### PythonRobotics: LiDAR-to-Grid-Map

The simplest reference implementation:
- Bresenham ray casting from sensor through free space to obstacles
- Cell values: 0.0 (free), 0.5 (unknown), 1.0 (occupied)
- Default resolution: 2cm/pixel
- Dependencies: NumPy, Matplotlib only
- Source: `AtsushiSakai/PythonRobotics/Mapping/lidar_to_grid_map/`

### Roll-Your-Own Occupancy Grid (Recommended Starting Point)

A minimal occupancy grid for AMBOT needs only ~100 lines of Python:

```python
import numpy as np

class OccupancyGrid:
    def __init__(self, size_m=20.0, resolution_m=0.05):
        """20m x 20m area at 5cm resolution = 400x400 grid = 160KB"""
        self.resolution = resolution_m
        self.size = int(size_m / resolution_m)
        self.origin = self.size // 2  # robot starts at center
        # Log-odds representation (0 = unknown, + = occupied, - = free)
        self.grid = np.zeros((self.size, self.size), dtype=np.float32)

    def update(self, robot_x, robot_y, robot_theta, scan_angles, scan_distances):
        """Update grid with one LiDAR scan using Bresenham ray casting."""
        rx = int(robot_x / self.resolution) + self.origin
        ry = int(robot_y / self.resolution) + self.origin

        for angle, dist in zip(scan_angles, scan_distances):
            if dist <= 0:
                continue
            # Endpoint in grid coordinates
            world_angle = robot_theta + angle
            ex = int((robot_x + dist * np.cos(world_angle)) / self.resolution) + self.origin
            ey = int((robot_y + dist * np.sin(world_angle)) / self.resolution) + self.origin

            # Bresenham line: mark cells as free
            for cx, cy in bresenham(rx, ry, ex, ey):
                if 0 <= cx < self.size and 0 <= cy < self.size:
                    self.grid[cy, cx] -= 0.1  # log-odds: more negative = more free

            # Mark endpoint as occupied
            if 0 <= ex < self.size and 0 <= ey < self.size:
                self.grid[ey, ex] += 0.3  # log-odds: more positive = more occupied

    def get_map(self):
        """Return occupancy probabilities [0, 1]."""
        return 1.0 / (1.0 + np.exp(-self.grid))  # sigmoid of log-odds
```

### Other Grid Mapping Resources

| Resource | Description | Useful? |
|----------|-------------|---------|
| richardos/occupancy-grid-a-star (80 stars) | A* on occupancy grids | Path planning, not mapping |
| grid-mapping-in-ROS (52 stars) | Bayes filter + Bresenham | ROS-based but algorithm is instructive |
| PythonRobotics Gaussian Grid Map | Probabilistic grid | Good reference |
| PythonRobotics Ray Casting Grid Map | Ray-cast occupancy | Good reference |

---

## 9. Practical Recommendations for AMBOT

### Immediate Next Step: ICP-Based Dead Reckoning

**Goal:** Track robot position relative to starting point using LiDAR scan matching.

**Dependencies:** `numpy`, `scikit-learn` (for KDTree in ICP)

**Implementation plan:**
1. Port/adapt ClayFlannigan/icp (50 lines, NumPy + sklearn)
2. Collect consecutive LD19 scans
3. Run ICP between scan[t-1] and scan[t]
4. Accumulate (dx, dy, dtheta) for position tracking
5. Integrate with wandering demo to show position trail

**Estimated effort:** 1-2 sessions. Pure Python. No new hardware.

### Medium-Term: Occupancy Grid Mapping

**Goal:** Build a 2D map of the environment as the robot wanders.

**Implementation plan:**
1. Create OccupancyGrid class (see Section 8, ~100 lines)
2. Feed ICP-estimated poses + LiDAR scans
3. Visualize as image (save periodically or display on Jetson)
4. Log-odds update for robustness

**Estimated effort:** 1-2 sessions. NumPy only.

### Longer-Term: BreezySLAM Integration

**Goal:** Better SLAM with C-accelerated code.

**Implementation plan:**
1. Build BreezySLAM on RPi 3B (`python3 setup.py install`)
2. Define LD19 sensor class (see Section 2c)
3. Create RMHC_SLAM instance with appropriate map size
4. Feed scans (with or without ICP-derived odometry)
5. Extract position and map

**Risk:** Build may fail on Python 3.13 / Debian 13 (not tested). Have fallback of
pure-Python approach.

### Architecture Sketch

```
LD19 LiDAR (10Hz, 467pts)
       |
       v
  [LiDAR Driver Thread]
       |
       +---> [ICP Scan Matcher] ---> (dx, dy, dtheta) ---> [Position Accumulator]
       |                                                           |
       +---> [Occupancy Grid] <--- (robot pose) -----------------+
       |          |
       |          v
       |     [Map Image / Path Planning]
       |
       +---> [Wandering Behavior] (existing NaturalWanderBehavior)
       |
  [MPU6050 IMU] ---> (gyro heading) ---> [Fuse with ICP dtheta]
```

### What NOT to Do

- **Don't start with full SLAM.** ICP + grid mapping is simpler and proves the concept.
- **Don't use >500 particles in pure Python.** Ray casting in Python is too slow.
- **Don't use 1cm grid resolution.** 5cm is plenty for a room-scale robot.
- **Don't try FastSLAM with per-particle maps.** Memory-heavy and complex.
- **Don't integrate accelerometer for position.** Gyro heading is useful; accelerometer
  position integration is practically useless (drift in seconds).

---

## 10. Key Libraries Summary

| Library | Install | Purpose | RPi 3B Ready? |
|---------|---------|---------|---------------|
| **numpy** | `pip install numpy` | Array math, grid storage | Yes (already have it) |
| **scikit-learn** | `pip install scikit-learn` | KDTree for ICP nearest-neighbor | Yes, but heavy (~100MB). Consider scipy.spatial.KDTree instead. |
| **scipy** | `pip install scipy` | KDTree alternative, SVD | Yes (already have it likely) |
| **breezyslam** | Build from source | Full SLAM | Probably (needs testing) |
| **pfilter** | `pip install pfilter` | Generic particle filter framework | Yes (NumPy only) |
| **matplotlib** | `pip install matplotlib` | Grid visualization | Yes (for debug/dev) |

### Minimal ICP with Only NumPy + SciPy (No sklearn)

```python
from scipy.spatial import KDTree
import numpy as np

def icp_2d(source, target, max_iter=20, tolerance=1e-5):
    """
    Simple 2D ICP. source/target are Nx2 arrays of (x,y) points.
    Returns: rotation angle, translation (tx, ty), transformed source.
    """
    src = source.copy()
    for i in range(max_iter):
        tree = KDTree(target)
        distances, indices = tree.query(src)
        matched = target[indices]

        # Compute centroids
        src_mean = src.mean(axis=0)
        tgt_mean = matched.mean(axis=0)
        src_centered = src - src_mean
        tgt_centered = matched - tgt_mean

        # SVD for optimal rotation
        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = tgt_mean - R @ src_mean
        src = (R @ src.T).T + t

        mean_error = distances.mean()
        if mean_error < tolerance:
            break

    # Extract angle from rotation matrix
    angle = np.arctan2(R[1, 0], R[0, 0])
    return angle, t, src
```

This is ~30 lines, needs only NumPy and SciPy, and runs in milliseconds for 467 points.

---

## Appendix: Glossary

- **MCL:** Monte Carlo Localization -- particle filter for robot position estimation
- **SLAM:** Simultaneous Localization and Mapping
- **ICP:** Iterative Closest Point -- aligns two point clouds
- **RMHC:** Random-Mutation Hill-Climbing -- stochastic optimization
- **Occupancy Grid:** 2D array where each cell stores probability of being occupied
- **Log-odds:** Numerical representation for Bayesian occupancy updates
- **Ray casting:** Simulating LiDAR beams through a map to predict expected readings
- **Resampling:** Duplicating high-weight particles, discarding low-weight ones
- **CoreSLAM:** "A SLAM Algorithm in less than 200 lines of C code" (Steux, 2010)
- **Bresenham:** Integer-arithmetic line drawing algorithm (for ray casting through grids)
