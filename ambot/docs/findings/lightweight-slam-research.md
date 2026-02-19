# Lightweight SLAM Research: No-ROS Options for RPi 3B + LD19 LiDAR

**Date:** 2026-02-19
**Context:** AMBOT robot with RPi 3B (906MB RAM, Cortex-A53 quad-core, Python 3.13), LD19 LiDAR (~467 points/scan, 360-degree, 2D), no IMU wired yet (MPU6050 available), no wheel odometry.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [BreezySLAM (RECOMMENDED)](#1-breezyslam)
3. [TinySLAM / CoreSLAM](#2-tinyslam--coreslam)
4. [Hector SLAM (Standalone)](#3-hector-slam-standalone)
5. [GMapping (Standalone)](#4-gmapping-standalone)
6. [ICP Scan Matching ("Poor Man's SLAM")](#5-icp-scan-matching-poor-mans-slam)
7. [PythonRobotics SLAM Implementations](#6-pythonrobotics-slam-implementations)
8. [NDT Scan Matching](#7-ndt-scan-matching)
9. [Custom Occupancy Grid Mapping](#8-custom-occupancy-grid-mapping)
10. [Comparison Matrix](#comparison-matrix)
11. [Memory Analysis for RPi 3B](#memory-analysis-for-rpi-3b)
12. [Recommendation for AMBOT](#recommendation-for-ambot)
13. [Implementation Plan](#implementation-plan)

---

## Executive Summary

After researching all available lightweight SLAM options that work without ROS2, **BreezySLAM** is the clear winner for the AMBOT project. It is the only mature, pip-installable Python SLAM library with C extensions for ARM performance, proven LiDAR support, and the ability to work without odometry. The LD19 LiDAR can be integrated by defining a simple custom `Laser` subclass.

The second-best option is a **custom ICP-based incremental mapping** approach using numpy/scipy, which gives more control but requires significantly more development effort.

All other options (Hector SLAM, GMapping, TinySLAM) are either ROS-dependent, unmaintained, or educational-only code not suitable for real-time embedded use.

---

## 1. BreezySLAM

**Repository:** https://github.com/simondlevy/BreezySLAM
**License:** LGPL-3.0
**Stars:** ~796 | **Forks:** ~263
**Language:** Python 3 with C extensions
**Last Activity:** Issues through 2023; core code stable

### What It Is

BreezySLAM is a Python package for 2D LiDAR-based SLAM based on the CoreSLAM/TinySLAM algorithm. It provides a clean separation between map-building and particle-filtering components. The core computation runs as a C extension with SIMD (Intel) and NEON (ARMv7) optimizations.

### Algorithm: RMHC_SLAM

Random-Mutation Hill-Climbing SLAM. Unlike traditional particle filter SLAM (which maintains hundreds of particles), RMHC_SLAM maintains a **single position estimate** and improves it through iterative random search:

1. Generate random position mutations (x, y, theta)
2. Score each against the current map using scan correlation
3. Keep mutations that improve the match score
4. Repeat for `max_search_iter` iterations (default: 1000)

This is dramatically more memory-efficient than multi-particle approaches.

### Key API

```python
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1  # or custom Laser subclass

# Create SLAM object
slam = RMHC_SLAM(
    laser=MyLD19Laser(),       # Laser sensor definition
    map_size_pixels=800,       # Map resolution (800x800 = 640KB)
    map_size_meters=20,        # Physical map extent
    map_quality=50,            # Integration speed (0-255)
    hole_width_mm=600,         # Obstacle width for mapping
    random_seed=42,            # Reproducibility
    sigma_xy_mm=100,           # Position search spread
    sigma_theta_degrees=20,    # Rotation search spread
    max_search_iter=1000       # Search iterations per update
)

# Main loop
while running:
    distances = get_lidar_scan()  # List of distances in mm

    # Update with or without odometry
    slam.update(distances)  # No odometry needed!
    # OR: slam.update(distances, velocities)  # With odometry

    # Get position
    x_mm, y_mm, theta_degrees = slam.getpos()

    # Get map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    slam.getmap(mapbytes)
```

### Defining a Custom LD19 Laser

```python
from breezyslam.sensors import Laser

class LD19Laser(Laser):
    def __init__(self):
        super().__init__(
            scan_size=467,                    # Points per scan (actual from LD19)
            scan_rate_hz=10,                  # LD19 rotation rate
            detection_angle_degrees=360,      # Full 360-degree coverage
            distance_no_detection_mm=12000,   # LD19 max range: 12m
            detection_margin=0,
            offset_mm=0
        )
```

**Note on scan_size:** BreezySLAM expects a fixed scan size. The LD19 delivers ~467 points per revolution, but this varies slightly. You will need to either:
- Resample/interpolate each scan to a fixed number of points (e.g., 360 or 480)
- Pad/truncate to a fixed size
- Use the most common count as the fixed size

Resampling to 360 points (1 per degree) is cleanest and avoids issues with variable-length scans.

### Sensor Requirements

| Sensor | Required? | Notes |
|--------|-----------|-------|
| 2D LiDAR | **YES** | Primary input. ~200+ points per scan recommended |
| Odometry | No | Optional. Improves accuracy but not required |
| IMU | No | Not used directly. Could feed into odometry model |

### Resource Requirements (Estimated for RPi 3B)

| Resource | Estimate | Notes |
|----------|----------|-------|
| **RAM (map 500x500)** | ~250 KB map + ~2 MB working | Very manageable |
| **RAM (map 800x800)** | ~640 KB map + ~3 MB working | Recommended size |
| **RAM (map 1000x1000)** | ~1 MB map + ~4 MB working | Upper practical limit |
| **CPU per update** | ~20-50ms | With C extensions on ARM |
| **Python overhead** | ~50 MB | Interpreter + numpy |
| **Total RAM** | ~60-70 MB | Well within 906 MB |

### Pros

- **Only mature Python SLAM library** that is pip-installable and standalone
- **C extensions with ARM NEON** optimizations (critical for RPi performance)
- **No odometry required** -- works with LiDAR-only scan matching
- **Simple API** -- 4 methods: constructor, update, getpos, getmap
- **Proven on similar hardware** -- designed for resource-constrained robots
- **Map output** -- generates occupancy grid as byte array
- **Deterministic mode** available for testing (Deterministic_SLAM)

### Cons

- **Maintenance status unclear** -- last issues from 2023, 56 open issues
- **No native LD19 support** -- requires custom Laser subclass (straightforward)
- **Python 3.13 compatibility unknown** -- may need to build C extension from source
- **Single-position (not particle cloud)** -- less robust than full particle filter
- **No loop closure** -- drift accumulates in large environments
- **Variable scan sizes** -- LD19's ~467 points need resampling to fixed count

### Build from Source (Required for RPi 3B + Python 3.13)

```bash
# On RPi, inside venv
git clone https://github.com/simondlevy/BreezySLAM.git
cd BreezySLAM/python
python3 setup.py install
```

If the C extension fails to build on Python 3.13, a pure-Python fallback exists (much slower, but functional for prototyping).

---

## 2. TinySLAM / CoreSLAM

**Origin:** 2010 paper "TinySLAM, a lightweight SLAM algorithm" by Bruno Steux and Oussama El Hamzaoui
**Core idea:** Simplest possible particle-filter SLAM
**Repository (educational):** https://github.com/Aramarchuk/TinySlam

### What It Is

TinySLAM (also called CoreSLAM) was designed as "the smallest viable SLAM implementation" -- approximately 200 lines of C code. BreezySLAM is actually *based on* the CoreSLAM algorithm, so using BreezySLAM is essentially using an improved TinySLAM.

### Algorithm

1. Maintain an occupancy grid map
2. For each LiDAR scan:
   - Project scan rays onto the map using current pose estimate
   - Score the match (sum of occupancy values at scan endpoints)
   - Use Monte Carlo search to find the pose that maximizes match score
   - Update the map with the new scan at the best pose

### Available Implementations

| Implementation | Language | Status | Notes |
|---|---|---|---|
| Original C (Steux) | C | Academic | ~200 lines, no Python bindings |
| BreezySLAM | Python/C | Usable | Best available implementation of this algorithm |
| Aramarchuk/TinySlam | Python | Educational | Simulation only, not for real hardware |

### Why Not Use Standalone TinySLAM

There is **no standalone Python TinySLAM implementation suitable for real robots**. The Aramarchuk repository is a course homework with simulated sensors only. The original C code has no Python bindings and no build system.

**Verdict: Use BreezySLAM instead** -- it IS TinySLAM/CoreSLAM with proper packaging.

---

## 3. Hector SLAM (Standalone)

**Original:** ROS package `hector_slam` by TU Darmstadt
**Key feature:** No odometry required -- uses scan-to-map matching
**Standalone Python:** **DOES NOT EXIST**

### What It Is

Hector SLAM uses Gauss-Newton scan matching against a multi-resolution occupancy grid. It was designed for UAVs and robots without wheel odometry, relying solely on high-frequency LiDAR data.

### Why It Matters

Hector SLAM is relevant because it shares our constraint: **no odometry available**. The algorithm works by:

1. Maintaining a multi-resolution map (e.g., 3 levels)
2. For each new scan, aligning it against the existing map using Gauss-Newton optimization
3. Starting alignment at coarse resolution, refining at fine resolution
4. Updating the map with the aligned scan

### Why We Cannot Use It

After thorough searching:
- **No standalone Python implementation exists** on GitHub
- All Python-related Hector SLAM repos are ROS wrappers
- The original C++ code is deeply intertwined with ROS message types
- Porting it would require significant effort

### Could We Port It?

The Gauss-Newton scan-to-map matching is the core innovation. In principle:
- The algorithm itself is ~500-1000 lines of C++
- It requires multi-resolution occupancy grids (memory multiplier)
- The math is well-documented in the original paper

**Effort estimate:** 2-4 weeks for a Python port. Not recommended when BreezySLAM already works without odometry.

**Verdict: Not viable** without major porting effort. BreezySLAM's RMHC approach achieves similar "no odometry" capability.

---

## 4. GMapping (Standalone)

**Original:** ROS package `slam_gmapping`
**Algorithm:** Rao-Blackwellized Particle Filter SLAM
**Standalone Python:** **DOES NOT EXIST**

### What It Is

GMapping is the classic 2D LiDAR SLAM workhorse in the ROS ecosystem. It uses a Rao-Blackwellized particle filter where each particle carries its own occupancy grid map.

### Why We Cannot Use It

- **No standalone Python implementation exists**
- The original is tightly coupled to ROS
- **Requires odometry** as a motion model input (critical limitation for AMBOT)
- **Memory intensive**: Each particle carries a full map copy. With 30 particles and a 500x500 grid, that is 30 * 250KB = 7.5MB just for maps, plus particle overhead
- On a 906MB RPi 3B, this would be marginal at best

### Resource Requirements (If It Existed)

| Resource | Estimate | Notes |
|----------|----------|-------|
| RAM (30 particles, 500x500) | ~50-100 MB | Each particle has a map |
| RAM (30 particles, 1000x1000) | ~200-400 MB | Dangerous on 906MB system |
| CPU | Heavy | Particle resampling + map updates |
| Odometry | **REQUIRED** | Cannot function without it |

**Verdict: Not viable** -- no standalone implementation, requires odometry we do not have, and memory-heavy.

---

## 5. ICP Scan Matching ("Poor Man's SLAM")

**Repository (reference):** https://github.com/AtsushiSakai/PythonRobotics (SLAM/ICPMatching/)
**Dependencies:** numpy, scipy
**Concept:** Align consecutive scans to estimate motion, accumulate into a map

### What It Is

ICP (Iterative Closest Point) is not SLAM per se, but it can be used as the foundation for incremental mapping:

1. Take scan at time T and scan at time T+1
2. Find the rotation + translation that best aligns them (ICP)
3. Use that transform as the robot's motion estimate
4. Accumulate scans into a global point cloud or occupancy grid

This is sometimes called "scan-to-scan matching" and provides a minimal localization + mapping solution.

### Algorithm (SVD-based 2D ICP)

```python
import numpy as np

def icp_step(source, target, max_iter=100, tolerance=0.0001):
    """
    Align source points to target points using ICP.
    source, target: Nx2 numpy arrays of 2D points.
    Returns: rotation matrix R (2x2), translation t (2x1)
    """
    T_total = np.eye(3)
    current = source.copy()

    for _ in range(max_iter):
        # 1. Find nearest neighbors
        indices = find_nearest(current, target)
        matched_target = target[indices]

        # 2. Compute centroids
        src_mean = current.mean(axis=0)
        tgt_mean = matched_target.mean(axis=0)

        # 3. Center the point sets
        src_centered = current - src_mean
        tgt_centered = matched_target - tgt_mean

        # 4. SVD to find rotation
        W = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(W)
        R = (U @ Vt).T
        t = tgt_mean - R @ src_mean

        # 5. Apply transform
        current = (R @ current.T).T + t

        # 6. Check convergence
        error = np.mean(np.linalg.norm(current - matched_target, axis=1))
        if error < tolerance:
            break

    return R, t
```

### Building "Poor Man's SLAM" from ICP

```python
# Pseudocode for incremental mapping with ICP
global_map = []  # Accumulated point cloud
robot_pose = np.eye(3)  # Cumulative transform

while running:
    scan = get_lidar_scan_as_xy()  # Nx2 array

    if len(global_map) > 0:
        # Match current scan against recent accumulated points
        R, t = icp_step(scan, recent_map_points)

        # Update pose
        robot_pose = update_pose(robot_pose, R, t)

    # Transform scan to global frame and add to map
    global_scan = transform(scan, robot_pose)
    global_map.extend(global_scan)

    # Periodically thin the global map (voxel grid downsampling)
    if len(global_map) > MAX_POINTS:
        global_map = downsample(global_map)
```

### Sensor Requirements

| Sensor | Required? | Notes |
|--------|-----------|-------|
| 2D LiDAR | **YES** | Only input needed |
| Odometry | No | ICP derives motion from scan alignment |
| IMU | No | Could help reject bad ICP matches |

### Resource Requirements

| Resource | Estimate | Notes |
|----------|----------|-------|
| RAM (pure numpy) | ~10-30 MB | Point arrays + working memory |
| CPU per ICP step | ~50-200ms | Depends on point count, pure Python |
| CPU with scipy KDTree | ~20-80ms | Faster nearest-neighbor lookup |
| Accumulated map | Grows over time | Must downsample periodically |

### Pros

- **Minimal dependencies** -- numpy + scipy only
- **No odometry required** -- motion comes from scan matching
- **Full control** -- can tune everything for our specific hardware
- **Educational value** -- understand SLAM from the ground up
- **Incremental complexity** -- start with scan matching, add mapping later

### Cons

- **No built-in map representation** -- must implement occupancy grid yourself
- **Drift accumulates** -- no loop closure, errors compound
- **Pure Python performance** -- slower than C-extension BreezySLAM
- **Nearest-neighbor bottleneck** -- O(n^2) naive, O(n log n) with KDTree
- **Fragile without odometry prior** -- can fail with large motions between scans
- **Significant development effort** -- 1-2 weeks for a basic working system

### Feasibility on RPi 3B

With ~467 points per scan and a KDTree for nearest neighbors:
- ICP alignment: ~50-100ms per step (scipy)
- Map accumulation: manageable if you downsample to ~5000-10000 global points
- Memory: well within 906MB budget

**Verdict: Feasible but labor-intensive.** Good as a learning exercise or fallback if BreezySLAM does not build on Python 3.13.

---

## 6. PythonRobotics SLAM Implementations

**Repository:** https://github.com/AtsushiSakai/PythonRobotics
**Stars:** ~28,700
**License:** MIT
**Maintenance:** Active (2,214+ commits, CI on Linux/macOS/Windows)

### Available SLAM Algorithms

| Algorithm | Type | Sensor Input | Notes |
|-----------|------|-------------|-------|
| **ICP Matching** | Scan alignment | 2D point clouds | Reference implementation, not full SLAM |
| **EKF SLAM** | Feature-based | Range + bearing | Landmark-based, not occupancy grid |
| **FastSLAM 1.0** | Particle filter + features | Range + bearing | Feature-based, not grid-based |
| **FastSLAM 2.0** | Improved particle filter | Range + bearing | Better proposal distribution |
| **Graph-based SLAM** | Pose graph optimization | Constraints | Backend optimization, needs frontend |

### Relevance to AMBOT

These implementations are **educational reference code**, not production libraries. They are useful for:
- Understanding algorithm internals
- Borrowing the ICP matching code as a starting point
- Learning EKF/particle filter math

They are NOT useful as drop-in SLAM solutions because:
- No real sensor interfaces
- Simulated data only
- Not optimized for real-time performance
- No occupancy grid output (except ICP-derived)

**Verdict: Reference code only.** Borrow the ICP implementation if building custom SLAM.

---

## 7. NDT Scan Matching

**Repository:** https://github.com/rsasaki0109/NormalDistributionTransform2D
**Stars:** 37
**License:** MIT

### What It Is

Normal Distribution Transform (NDT) is an alternative to ICP for scan alignment. Instead of matching individual points, it:

1. Divides the reference scan into grid cells
2. Fits a normal distribution (mean + covariance) to points in each cell
3. Optimizes the alignment by maximizing the likelihood of new scan points under these distributions

### Pros over ICP

- More robust to noise and outliers
- Smoother cost function (fewer local minima)
- Can be faster for dense point clouds

### Cons

- This repository is **localization only**, not mapping
- 100% Python (no C acceleration)
- Small community (37 stars)
- Would need to wrap it into a full SLAM pipeline yourself

**Verdict: Interesting alternative to ICP for scan matching, but not a complete SLAM solution.**

---

## 8. Custom Occupancy Grid Mapping

If none of the above libraries work, the simplest "SLAM" that provides useful navigation data is **occupancy grid mapping with known poses** (i.e., assuming ICP or BreezySLAM provides pose estimates):

```python
import numpy as np

class OccupancyGrid:
    def __init__(self, size_m=20, resolution_m=0.05):
        """
        size_m: map extent in meters
        resolution_m: cell size in meters (0.05m = 5cm)
        """
        self.resolution = resolution_m
        self.size = int(size_m / resolution_m)
        # Log-odds representation
        self.grid = np.zeros((self.size, self.size), dtype=np.float32)
        self.origin = np.array([self.size // 2, self.size // 2])

    def update(self, robot_x_m, robot_y_m, scan_points_m):
        """
        Update grid with a LiDAR scan from robot position.
        scan_points_m: Nx2 array of hit points in global frame (meters)
        """
        rx = int(robot_x_m / self.resolution) + self.origin[0]
        ry = int(robot_y_m / self.resolution) + self.origin[1]

        for px, py in scan_points_m:
            gx = int(px / self.resolution) + self.origin[0]
            gy = int(py / self.resolution) + self.origin[1]

            if 0 <= gx < self.size and 0 <= gy < self.size:
                # Mark endpoint as occupied (increase log-odds)
                self.grid[gy, gx] += 0.9

                # Raytrace free space (decrease log-odds along ray)
                for fx, fy in bresenham(rx, ry, gx, gy):
                    if 0 <= fx < self.size and 0 <= fy < self.size:
                        self.grid[fy, fx] -= 0.3

        # Clamp log-odds
        np.clip(self.grid, -5.0, 5.0, out=self.grid)

    def get_probability_map(self):
        """Convert log-odds to probability [0, 1]."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.grid))
```

### Memory for Occupancy Grid

| Grid Size | Resolution | Cells | RAM (float32) | RAM (int8) |
|-----------|-----------|-------|---------------|------------|
| 10m x 10m | 5cm | 200x200 | 160 KB | 40 KB |
| 20m x 20m | 5cm | 400x400 | 640 KB | 160 KB |
| 20m x 20m | 10cm | 200x200 | 160 KB | 40 KB |
| 50m x 50m | 10cm | 500x500 | 1 MB | 250 KB |

All sizes are trivial for the 906MB RPi 3B.

---

## Comparison Matrix

| Feature | BreezySLAM | Custom ICP | Hector SLAM | GMapping | PythonRobotics |
|---------|-----------|------------|-------------|----------|----------------|
| **Standalone (no ROS)** | YES | YES | NO | NO | YES |
| **Python package** | YES (pip/source) | DIY | N/A | N/A | Reference only |
| **C acceleration** | YES (ARM NEON) | No | N/A | N/A | No |
| **LiDAR-only (no odom)** | YES | YES | YES | NO | Varies |
| **Occupancy grid output** | YES | DIY | YES | YES | No |
| **RAM estimate** | ~60-70 MB | ~30-50 MB | N/A | ~100-400 MB | ~20-30 MB |
| **CPU per update** | ~20-50ms | ~50-200ms | N/A | N/A | N/A |
| **Development effort** | LOW (hours) | HIGH (weeks) | IMPOSSIBLE | IMPOSSIBLE | MEDIUM (days) |
| **Loop closure** | No | No | No | Yes | Varies |
| **Maintenance** | Stable/stale | N/A | Active (ROS) | Active (ROS) | Active |
| **LD19 compatibility** | Custom subclass | Native (numpy) | N/A | N/A | Native (numpy) |

---

## Memory Analysis for RPi 3B

**Total RAM:** 906 MB
**OS + services:** ~200-300 MB
**Python 3.13 interpreter:** ~30-50 MB
**Available for SLAM:** ~500-650 MB

### BreezySLAM Memory Budget

| Component | Size |
|-----------|------|
| Python interpreter + imports | ~50 MB |
| BreezySLAM C extension | ~1 MB |
| Map (800x800 bytes) | 0.64 MB |
| Scan buffer (467 * int) | ~4 KB |
| RMHC search working memory | ~1 MB |
| numpy overhead | ~10 MB |
| **TOTAL** | **~63 MB** |

**Verdict:** Extremely comfortable. Leaves ~500+ MB free for other processes (camera face tracking, motor control, etc.).

### Memory Comparison with Current Wandering Demo

The current wandering demos use:
- LiDAR scan buffer: ~4 KB
- Behavior state: negligible
- Camera face tracking (Demo 2): ~50-100 MB
- Motor control: negligible

Adding BreezySLAM would add ~60-70 MB, bringing total to ~160-220 MB. Still well within budget.

---

## Recommendation for AMBOT

### Primary: BreezySLAM

**BreezySLAM is the recommended approach** for the following reasons:

1. **Only viable option** -- it is literally the only standalone Python SLAM library that works without ROS
2. **No odometry required** -- RMHC_SLAM works with LiDAR-only, which matches our current hardware
3. **Lightweight** -- ~63 MB RAM, ~20-50ms per update cycle
4. **Simple integration** -- 4-method API, can be added to existing wandering demo architecture
5. **C extensions** -- ARM-optimized, critical for RPi 3B performance
6. **Proven algorithm** -- CoreSLAM/TinySLAM is well-studied and understood

### Fallback: Custom ICP + Occupancy Grid

If BreezySLAM fails to build on Python 3.13 (C extension compatibility), the fallback is:

1. Implement 2D ICP scan matching using numpy/scipy (borrow from PythonRobotics)
2. Use ICP-derived poses to build an occupancy grid
3. This requires ~1-2 weeks of development but has zero external dependencies

### Future Enhancement: Add IMU

When the MPU6050 is wired up, it can improve SLAM accuracy:
- **With BreezySLAM:** Feed IMU-derived rotation as part of odometry velocity input
- **With custom ICP:** Use IMU rotation as an initial alignment guess (warm-start ICP)

### What NOT to Do

- **Do NOT try to port Hector SLAM or GMapping** -- the effort is enormous and BreezySLAM already fills the gap
- **Do NOT install ROS2 on the RPi 3B** -- it would consume most of the 906MB RAM just for the middleware
- **Do NOT use map sizes larger than 1000x1000** -- diminishing returns for indoor robotics
- **Do NOT skip the scan resampling step** -- LD19's variable point count will cause issues with BreezySLAM's fixed-size expectation

---

## Implementation Plan

### Phase 1: BreezySLAM Integration (1-2 days)

1. **Build BreezySLAM on RPi 3B**
   ```bash
   cd ~/ambot
   source venv/bin/activate
   git clone https://github.com/simondlevy/BreezySLAM.git /tmp/BreezySLAM
   cd /tmp/BreezySLAM/python
   python3 setup.py install
   ```

2. **Create LD19 Laser class** in `demos_common/sensors.py`:
   ```python
   from breezyslam.sensors import Laser

   class LD19Laser(Laser):
       def __init__(self):
           super().__init__(
               scan_size=360,              # Resampled to 1 per degree
               scan_rate_hz=10,
               detection_angle_degrees=360,
               distance_no_detection_mm=12000,
               detection_margin=0,
               offset_mm=0
           )
   ```

3. **Add scan resampling function**:
   ```python
   import numpy as np

   def resample_scan(raw_angles, raw_distances, target_count=360):
       """
       Resample variable-length LD19 scan to fixed-size array.
       Returns list of distances at evenly-spaced angles.
       """
       target_angles = np.linspace(0, 360, target_count, endpoint=False)
       resampled = np.zeros(target_count)

       for i, ta in enumerate(target_angles):
           # Find nearest raw measurement
           diffs = np.abs(np.array(raw_angles) - ta)
           diffs = np.minimum(diffs, 360 - diffs)  # Handle wrap-around
           nearest_idx = np.argmin(diffs)
           if diffs[nearest_idx] < 5.0:  # Within 5 degrees
               resampled[i] = raw_distances[nearest_idx]
           # else: 0 means no detection

       return resampled.astype(int).tolist()
   ```

4. **Create SLAM wrapper** in `demos_common/slam.py`:
   ```python
   from breezyslam.algorithms import RMHC_SLAM

   class SLAMMapper:
       def __init__(self, laser, map_pixels=800, map_meters=20):
           self.slam = RMHC_SLAM(laser, map_pixels, map_meters)
           self.map_pixels = map_pixels
           self.mapbytes = bytearray(map_pixels * map_pixels)
           self.trajectory = []

       def update(self, distances):
           self.slam.update(distances)
           x, y, theta = self.slam.getpos()
           self.trajectory.append((x, y, theta))
           return x, y, theta

       def get_map(self):
           self.slam.getmap(self.mapbytes)
           return self.mapbytes

       def save_map(self, filename):
           self.get_map()
           # Save as PGM image
           with open(filename, 'wb') as f:
               f.write(f'P5\n{self.map_pixels} {self.map_pixels}\n255\n'.encode())
               f.write(self.mapbytes)
   ```

### Phase 2: Test with Recorded Data (1 day)

1. Record LiDAR scans to a log file while running the wandering demo
2. Replay the log through BreezySLAM to verify map generation
3. Tune parameters (map_quality, hole_width, sigma values)

### Phase 3: Real-time Integration (1-2 days)

1. Integrate SLAMMapper into the wandering demo loop
2. Run SLAM update alongside behavior decisions
3. Add map saving on shutdown
4. Optional: Live map visualization (if GUI available)

### Phase 4: Future -- SLAM-Informed Navigation

1. Use the occupancy grid for path planning
2. Replace reactive wandering with goal-directed navigation
3. Implement simple A* or wavefront planning on the occupancy grid

---

## References

- BreezySLAM: https://github.com/simondlevy/BreezySLAM
- CoreSLAM paper: B. Steux, O. El Hamzaoui, "tinySLAM: A SLAM algorithm in less than 200 lines C-language code" (2010)
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics
- NDT 2D: https://github.com/rsasaki0109/NormalDistributionTransform2D
- TinySLAM educational: https://github.com/Aramarchuk/TinySlam
- Hector SLAM paper: S. Kohlbrecher et al., "A Flexible and Scalable SLAM System with Full 3D Motion Estimation" (2011)
