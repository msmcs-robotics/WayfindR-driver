# Lightweight 2D LiDAR SLAM & Localization Options

**Research date:** 2026-04-09
**Target hardware:**
- Raspberry Pi 3B (Cortex-A53, 906 MB RAM, Python 3.13)
- Jetson Orin Nano (6x A78AE, 7.4 GB RAM, Python 3.10)
**Sensors:** LD19 LiDAR (360 deg, ~467 pts/scan, 10 Hz, 0.02-12 m), optional MPU6050 gyro
**Drive:** Differential drive, **no wheel encoders / no odometry**
**Goal:** Indoor wandering robot — need pose estimate and/or occupancy map, must run
real-time at 10 Hz scan rate, ideally under 100 MB RAM on the Pi.

---

## TL;DR — Ranked Recommendations

| Rank | Option | Platform fit | Why |
|------|--------|-------------|-----|
| 1 | **BreezySLAM (RMHC_SLAM)** | RPi 3B + Jetson | Python-native, particle filter, runs without odometry, CoreSLAM C core, proven on RPi 3 |
| 2 | **Hector-SLAM port / custom Gauss-Newton scan matcher** | Jetson (Python) / RPi (if lean) | Works *without* odometry by design; gyro-friendly |
| 3 | **PythonRobotics ICP + log-odds grid** | RPi 3B | Tiny, pure Python, maximum hackability, no pip wheels needed |
| 4 | **Custom occupancy grid + ICP scan matching (gyro-assisted)** | Both | Minimal deps, ideal if wandering only needs "what's around me" |
| 5 | **Cartographer (ROS2)** | Jetson only | Best accuracy but heavy; not realistic on RPi 3B |
| 6 | **RTAB-Map** | Jetson only | Overkill for 2D-only indoor wandering |
| 7 | **slam_toolbox** | Jetson only | Great features but designed for ROS2 ecosystem |

**Bottom line:** On the RPi 3B, **BreezySLAM** is the only proven pure-Python SLAM
that fits in 906 MB and gives both map + pose without odometry. For more experimental
setups, a **custom ICP + occupancy grid + gyro** pipeline built on top of
`demos_common` keeps full control and very low RAM. Full SLAM stacks
(Cartographer, RTAB-Map, slam_toolbox) should be reserved for the Jetson.

---

## 1. BreezySLAM (recommended primary)

**Description:** Python wrapper around CoreSLAM/tinySLAM (Steux & El Hamzaoui, 2010
— a full SLAM in <200 lines of C). Uses Monte-Carlo (particle) localization on a
2D occupancy grid. Distinct classes: `RMHC_SLAM` (random-mutation hill-climbing,
no odometry required) and `Deterministic_SLAM` (needs odometry).

- **Repo:** https://github.com/simondlevy/BreezySLAM
- **Algorithm core:** C + SIMD/NEON extensions called from Python
- **Memory footprint:**
  - Grid size is tunable. Defaults (e.g. 800 px @ 35 mm/px) ≈ 640 KB for the
    map itself. Process RSS typically 40-80 MB on RPi 3 with NumPy + rplidar.
  - Well under the 100 MB target.
- **CPU on RPi 3B:** CoreSLAM is famously cheap. Community reports:
  "~75 scan samples at 1 Hz update with artifacts on Pi 3"; cleaner at reduced
  grid size. With the LD19's 467 pts/scan at 10 Hz a 4-5 Hz update loop is
  realistic with `map_quality` tuning. On Jetson Orin Nano it will run at
  the full 10 Hz trivially.
- **Required inputs:** Raw scans (distances + angles). Odometry is
  **optional**. Velocity input (mm/s, deg/s dt) helps but is not mandatory.
- **Outputs:** Robot pose (x, y, theta) + occupancy grid (bytes 0-255).
- **Python availability:** Yes — pure Python API over C extensions. Builds
  via `python setup.py install`. No official wheels.
- **Known issues / Py 3.13:**
  - Last release pre-dates Py 3.13; uses C-API extensions. Likely needs
    `Py_ssize_t` / `PyObject_HEAD` shims or a small patch for 3.13 builds.
    Test with `pip install .` on the Pi first; if it fails, pin Py 3.11 via
    `pyenv` or use the Jetson (Py 3.10 works out of the box).
  - Uses `distutils` — need to install via a `setup.py` shim or patch to
    `setuptools` on modern Python.
  - No built-in loop closure; long wanders drift.
  - `rplidar` example in repo — we would need an LD19 adapter that feeds
    `(dist_mm, angle_deg)` lists to `slam.update(scans, ...)`.
- **Pros for AMBOT:**
  - Purpose-built for pure-Python robotics on limited hardware
  - Handles no-odometry case natively (`RMHC_SLAM`)
  - Tiny footprint, clean API: `slam.update(); x, y, theta = slam.getpos()`
  - Can optionally plug the MPU6050 yaw in as a velocity hint
  - Matches current `pathfinder/` Python-only philosophy
- **Cons:**
  - C extension build on RPi Py 3.13 is the main unknown
  - No loop closure — maps drift on long runs (OK for wandering precursor)
  - Particle filter is non-deterministic: multiple runs give slightly different maps
- **Integration sketch:**
  ```python
  from breezyslam.algorithms import RMHC_SLAM
  from breezyslam.sensors import Laser
  ld19 = Laser(scan_size=467, scan_rate_hz=10, detection_angle=360,
               distance_no_detection_mm=12000)
  slam = RMHC_SLAM(ld19, map_size_pixels=800, map_size_meters=20)
  while True:
      scan = read_ld19()  # list of distances in mm
      slam.update(scan)
      x_mm, y_mm, theta_deg = slam.getpos()
  ```

---

## 2. Hector-SLAM (ROS) / custom Gauss-Newton scan matcher

**Description:** Uses multi-resolution occupancy grids and Gauss-Newton
minimization to match each new scan to the current map. Designed **specifically**
for robots without odometry — Kohlbrecher et al., IROS 2011. Relies on high
LiDAR update rate and good scan quality.

- **Repo:** https://github.com/tu-darmstadt-ros-pkg/hector_slam (ROS C++)
- **Memory footprint:** ROS package ~100-250 MB RAM in practice (ROS node +
  multi-res maps). A Python re-implementation of the core (scan matcher +
  single grid) is far smaller (20-40 MB).
- **CPU on RPi 3B:** Native C++ version is fast but ROS overhead is painful on
  906 MB. Community has reported "worst memory usage of 2D SLAMs" vs
  Cartographer/Gmapping in ROS studies, mostly due to ROS + multi-res grid
  storage. A trimmed standalone port could be comfortable.
- **Required inputs:** Scans only. Optionally IMU for roll/pitch (irrelevant
  for 2D). Yaw from gyro improves initial pose guess.
- **Outputs:** Pose + occupancy grid.
- **Python availability:** No first-class Python binding. The algorithm is
  simple enough to reimplement: bilinear-interpolated occupancy grid + gradient
  descent on (dx, dy, dtheta). ~300-500 LoC in Python/NumPy.
- **Known issues / Py 3.13:** N/A — would be custom code.
- **Pros:**
  - Built for "no odometry" use case — literally the answer to your constraint
  - MPU6050 yaw rate integrates cleanly as a seed for the Gauss-Newton solver
    (turns a rotation-heavy problem into a small delta solve)
  - Deterministic (unlike particle filters)
- **Cons:**
  - Running the real ROS package on RPi 3B is unrealistic
  - A Python port is 2-3 days of work; no off-the-shelf option
  - No loop closure → accumulates drift
  - Bad in structureless corridors (like Cartographer)

---

## 3. PythonRobotics ICP Matching + lidar_to_grid_map

**Description:** The AtsushiSakai/PythonRobotics repo has minimal, didactic
NumPy implementations of exactly the building blocks we need:
- `SLAM/iterative_closest_point/iterative_closest_point.py` — SVD-based 2D ICP
- `Mapping/lidar_to_grid_map/lidar_to_grid_map.py` — Bresenham + log-odds grid
- `SLAM/EKFSLAM/ekf_slam.py` — EKF SLAM (landmark-based, less useful for raw
  LiDAR but still a reference)
- `SLAM/FastSLAM1/fast_slam1.py` — particle filter SLAM example

- **Repo:** https://github.com/AtsushiSakai/PythonRobotics
- **Memory footprint:** Just NumPy + matplotlib; well under 50 MB per component.
- **CPU:** Pure Python/NumPy; 2D ICP on ~500 pts easily runs at 10 Hz on RPi 3
  with KD-tree (`scipy.spatial.cKDTree`).
- **Required inputs:** Raw scans only for ICP and grid mapping.
- **Outputs:** Incremental pose transforms (ICP); grid map (log-odds).
- **Python availability:** Pure Python, minimal deps (numpy, scipy,
  matplotlib). **Works on Python 3.13.**
- **Known issues / Py 3.13:** None for the SLAM modules; only matplotlib is
  needed for visualization and is already installed.
- **Pros:**
  - Copy/paste friendly — single-file examples, BSD-style license
  - Zero C extensions, zero ROS, matches `pathfinder/` style
  - Author publishes textbook-grade docstrings
  - Can assemble a "DIY SLAM" in a few files: ICP for inter-scan rotation,
    fuse with gyro, log-odds grid
- **Cons:**
  - Not a drop-in SLAM; it's a parts bin, not an assembled algorithm
  - ICP alone drifts quickly without a map or loop closure
  - Builder is responsible for the assembly; more work than BreezySLAM
- **Recommended stack (build-your-own SLAM):**
  1. Gyro-assisted ICP for scan-to-scan motion (IMU seeds rotation)
  2. Log-odds occupancy grid (Bresenham) from Mapping module
  3. Periodic scan-to-map correction via correlative search (small window)

---

## 4. ICP Scan Matching (Python implementations)

Supporting references and reusable libraries:

- **richardos/icp** (https://github.com/richardos/icp): Pure Python 2D point
  cloud ICP, based on Lu & Milios 1997 ("Robot Pose Estimation in Unknown
  Environments by Matching 2D Range Scans"). Tiny, educational, works with
  NumPy only.
- **libpointmatcher** (https://github.com/norlab-ulaval/libpointmatcher): C++
  library, has Python bindings. Much heavier; overkill for RPi.
- **PythonRobotics ICP** (see item 3): the minimal SVD solver version.

**Memory:** <20 MB (numpy only) for pure-Python ICP.
**Py 3.13:** Works.
**Pros:** No deps, easy to integrate. **Cons:** Scan-to-scan ICP drifts
unbounded; needs to be fused with a map or gyro.

---

## 5. Occupancy Grid Mapping (pure Python)

- **PythonRobotics lidar_to_grid_map**: numpy array, 0/0.5/1 encoding, Bresenham
  line drawing, queue-based unknown-cell expansion. Example on the docs site:
  https://atsushisakai.github.io/PythonRobotics/modules/3_mapping/lidar_to_grid_map_tutorial/lidar_to_grid_map_tutorial.html
- **NekSfyris/occupancy_grid_mapping_2DLidar** (GitHub): proper log-odds
  update formulation, good reference for probabilistic updates.
- **Swepz/LidarBasedGridMapping**: uses Orebro dataset, matplotlib viz.

**Memory:** A 20 m x 20 m room at 5 cm resolution = 400x400 float32 = 640 KB.
**Py 3.13:** Works (numpy only).
**Ideal as the "map" backend for any custom SLAM we build.**

---

## 6. Particle Filter / FastSLAM

- **PythonRobotics FastSLAM 1.0 / 2.0** — landmark-based, not ideal for raw
  360 deg scans but a reference implementation.
- **xiaofeng419/SLAM-2D-LIDAR-SCAN** — FastSLAM with 2D laser scan matching
  in Python. Combines particle filter with ICP-like scan matching. Closest
  analog to BreezySLAM but pure Python.
- **yashv28/Particle-Filter-SLAM** — integrates IMU + odometry + 2D LiDAR for
  occupancy grid SLAM; useful as a *reference* for how to wire in the gyro.
- **yahsiuhsieh/particle-filter-slam** — another didactic implementation.

**Memory:** Particle filters scale with N particles x grid copy. ~50 MB
comfortable with 50-100 particles.
**CPU:** Expensive at high particle counts; 20-50 particles typically enough
for small indoor maps.
**Py 3.13:** These are reference/research repos, not packaged libraries;
mostly pure Python/NumPy. Expect to port, not `pip install`.
**Bottom line:** BreezySLAM already *is* a production particle filter SLAM in
Python; these are study references and alternatives if BreezySLAM's C
extension won't build on Py 3.13.

---

## 7. Scan Matching with Gyro (IMU-assisted ICP)

The canonical approach:

1. Integrate MPU6050 yaw gyro between scans → `dtheta_imu`
2. Pre-rotate the new scan by `dtheta_imu` before running ICP
3. ICP only has to solve a small residual (dx, dy, small dtheta)
4. Convergence improves dramatically, especially in rotation-heavy motions

Found references:
- **LiLi-OM / LIO-SAM / FAST-LIO** — full LiDAR-inertial fusion; far too heavy
  for 2D indoor use, but the math (gyro pre-integration / de-skew) maps
  directly onto our case.
- **ros-sensor-fusion-tutorial** — simple explanation of ekf + imu fusion
  patterns.
- Generalized-ICP (GICP) — extends ICP with point-to-plane and is more robust,
  but the basic point-to-point ICP with gyro seed is sufficient for indoor
  wandering.

**Takeaway:** For the AMBOT, the simplest useful addition is a **gyro-seeded
ICP** inside whichever SLAM we pick. With the MPU6050, wire up:

```python
dtheta_imu = gyro_z * dt  # radians
rot = Rot2D(dtheta_imu)
rotated_scan = rot @ new_scan_xy
# then run ICP(prev_scan, rotated_scan) starting from identity
```

This is ~30 lines of code and massively improves scan matching reliability
during turns — which is the failure mode of plain scan-to-scan ICP.

---

## 8. g2o / GTSAM / Pose Graph Optimization

- **g2o-python:** `pip install g2o-python` — Python binding, installs via pip.
  Useful if we ever want global optimization after collecting a pose graph.
- **gtsam:** `pip install gtsam` — heavy C++ library with Python bindings;
  ~50-100 MB install. Overkill for the RPi; reasonable on the Jetson.
- **miniSAM:** Lightweight alternative to GTSAM with Python bindings.
  Factor-graph non-linear least-squares.

**Memory:** g2o-python ~30 MB; GTSAM ~100 MB.
**Py 3.13:** g2o-python has recent wheels on PyPI; GTSAM lags on bleeding-edge
Python versions.
**Pros:** Real global optimization; essential for large maps with loop
closures.
**Cons:** Overkill for an indoor wanderer that isn't doing loop closure yet.
*Recommend deferring until we actually need loop closure.*

---

## 9. Loop Closure Detection

Found references (all heavy for RPi, realistic on Jetson):

- **ScanContext** (Kim et al.) — global descriptor for LiDAR scans; works in
  3D primarily, 2D variants exist.
- **PyICP-SLAM** (https://github.com/gisbi-kim/PyICP-SLAM) — full Python SLAM
  that combines ICP + ScanContext for loop closure. ~100-200 MB.
- **FastLCD** — 3D point-cloud loop closure; not a fit here.
- **Cartographer's branch-and-bound scan matcher** — gold standard for 2D
  loop closure but locked inside the Cartographer codebase.

**Recommendation:** Skip loop closure entirely for v1. For wandering, periodic
drift is acceptable. Revisit when a long-duration mapping mission comes up.

---

## 10. Heavy-Weight SLAM Stacks (Jetson only)

These are documented for completeness but **not recommended for RPi 3B**.

### Google Cartographer (2D)
- **Language:** C++ (ROS/ROS2 package)
- **Memory on Pi 3B:** Confirmed by community that Pi 3B with 1 GB RAM
  "struggles to run Cartographer for more than a minute in a small hallway
  before map TF rates drop below acceptable levels". **Not viable on the RPi.**
- **Memory on Jetson:** ~500 MB-1 GB depending on map size; fits comfortably.
- **Accuracy:** Best-in-class 2D SLAM; has branch-and-bound loop closure.
- **Inputs:** Works with scans only (no odometry) if you tune
  `use_online_correlative_scan_matching=true`.
- **Py binding:** None official; used via ROS topics.
- **Verdict:** Use on Jetson if/when we move to ROS2. Do not attempt on RPi.

### RTAB-Map
- **Language:** C++
- **Memory:** Has explicit memory management for large maps (that's its
  strength). ~500 MB baseline, grows with map size.
- **Best for:** Visual + LiDAR combined SLAM. Primarily RGB-D/stereo focused.
- **Verdict:** Overkill for pure 2D LiDAR indoor wandering. Reserve for later
  if we add cameras + need global re-localization.

### slam_toolbox
- **Language:** C++ (ROS2)
- **Memory:** Moderate; "more reliable than gmapping in small spaces", better
  features (lifelong mapping, offline mode, interactive mode).
- **Verdict:** Good choice *if* we move the AMBOT to ROS2 on the Jetson. Not
  a pure-Python option.

### GMapping
- **Language:** C++ (OpenSLAM / ROS wrapper)
- **Memory:** Community benchmark ~64 MB RAM, ~68% CPU on Pi-class hardware.
  Surprisingly light.
- **Cons:** Requires odometry — we don't have encoders. Would need to fake
  odometry from scan matching first, defeating the purpose.
- **Verdict:** Skip — the no-odometry constraint rules it out unless we add
  a scan-matcher-based odometry publisher.

---

## Cross-Cutting Observations

### Python 3.13 compatibility landscape
- Pure Python/NumPy solutions (PythonRobotics, richardos/icp, custom ICP+grid): **Works**
- BreezySLAM (C extensions, distutils-era): **Likely needs patching**
- g2o-python: **Works** (wheels on PyPI)
- GTSAM: **Lagging** — last-known Python bindings target 3.10-3.11
- ROS2 stack (Cartographer, slam_toolbox): **N/A** — ROS2 ties you to the
  system Python (Ubuntu's 3.10 on Jetson)

### Without wheel odometry
Only these work natively:
- **Hector-SLAM** (designed for it)
- **BreezySLAM RMHC_SLAM** (particle filter fills in motion model)
- **Cartographer** (with correlative matcher enabled)
- **Custom ICP-based SLAM** (treat scan matching *as* odometry)

Every other option (GMapping, Deterministic_SLAM, slam_toolbox default
configs) assumes odometry is available.

### MPU6050 utility
- The gyro alone adds no position info, but its yaw-rate integration is
  extremely useful as an **initial guess** for scan matching.
- All of the top 4 options can be improved by wiring up MPU6050 yaw as a
  pre-rotation step before ICP/scan-match.
- This is the single biggest ROI improvement we can make on any custom stack.

### 10 Hz real-time budget on RPi 3B
- 100 ms budget per scan
- Naive Python ICP with 467 points + brute-force NN = too slow
- Python ICP with scipy cKDTree ≈ 20-40 ms ✓
- BreezySLAM (C core) ≈ 10-30 ms ✓
- Log-odds grid update (Bresenham in Python) ≈ 15-25 ms ✓
- Comfortable headroom if we skip matplotlib during runtime

---

## Recommendation for AMBOT

### Phase 1 (RPi 3B, current hardware)
**Try BreezySLAM first.** Clone the repo, patch for Py 3.13 if needed (likely
just `distutils` → `setuptools`), wire in LD19 scans via `demos_common`. This
gets us map + pose in a single afternoon if the build works.

**Fallback:** Build a minimal custom SLAM from PythonRobotics parts:
1. Copy `lidar_to_grid_map.py` for the occupancy grid
2. Copy `iterative_closest_point_matching.py` for scan matching
3. Add gyro pre-rotation from MPU6050 (`demos_common.sensors.setup_imu`)
4. Wire into `NaturalWanderBehavior` as an optional pose provider

Either option respects the "pure Python, < 100 MB, no ROS" constraint and
works without wheel odometry.

### Phase 2 (Jetson, when moving to LLM-integrated wandering)
**Use Cartographer via ROS2** (or `slam_toolbox` if we want lifelong mapping).
The Jetson has plenty of RAM, runs Ubuntu 22.04, and ROS2 Humble is the
natural fit. This is deferred work — only tackle when Phase 1 limits are
actually hit.

### Phase 3 (future — only if needed)
Add loop closure via PyICP-SLAM (ScanContext) or Cartographer's built-in
branch-and-bound. Only worth doing if maps are large enough for drift to
become a real problem.

---

## Next Actions

1. Attempt `git clone https://github.com/simondlevy/BreezySLAM && pip install .`
   on the RPi in its venv. Document any Py 3.13 build errors.
2. If BreezySLAM builds, write `pathfinder/slam.py` wrapping it behind a
   `SlamProvider` interface that `demos_common` can consume.
3. Add gyro integration from `pathfinder/imu.py` into the scan-update path.
4. Log `(timestamp, x, y, theta, map_bytes)` to a replay file so we can
   regression-test SLAM output off-robot.
5. If BreezySLAM fails, implement the PythonRobotics-based fallback (ICP +
   log-odds grid + gyro pre-rotation) directly in `pathfinder/`.

---

## Sources

### BreezySLAM / CoreSLAM / tinySLAM
- https://github.com/simondlevy/BreezySLAM
- https://github.com/AdroitAnandAI/SLAM-on-Raspberry-Pi
- https://towardsdatascience.com/indoor-robot-localization-with-slam-f8b447bcb865/
- https://www.diyrobocars.com/2018/08/04/lidar-slam-without-ros-on-cheap-computing-boards/
- https://prometeo.blog/en/practical-case-2d-slam-on-rpi-5-with-rplidar-a1-tb6612fng/
- https://www.semanticscholar.org/paper/CoreSLAM-:-a-SLAM-Algorithm-in-less-than-200-lines-Steux-Hamzaoui/1c7e7af4388b17b137badc6ec19e9724e6bd91e4
- https://github.com/WestTeam/CoreSLAM

### Hector-SLAM
- https://github.com/tu-darmstadt-ros-pkg/hector_slam (reference)
- https://github.com/NickL77/RPLidar_Hector_SLAM
- https://github.com/avs2805/hector_slam_quickstart
- https://answers.ros.org/question/35924/slam-without-odometry-gmapping-or-hector_slam/
- http://library.isr.ist.utl.pt/docs/roswiki/hector_mapping.html

### PythonRobotics
- https://atsushisakai.github.io/PythonRobotics/
- https://github.com/AtsushiSakai/PythonRobotics
- https://atsushisakai.github.io/PythonRobotics/modules/3_mapping/lidar_to_grid_map_tutorial/lidar_to_grid_map_tutorial.html
- https://atsushisakai.github.io/PythonRobotics/modules/4_slam/iterative_closest_point_matching/iterative_closest_point_matching.html
- https://github.com/AtsushiSakai/PythonRobotics/blob/master/SLAM/EKFSLAM/ekf_slam.py
- https://github.com/AtsushiSakai/PythonRobotics/blob/master/Mapping/lidar_to_grid_map/lidar_to_grid_map.py

### ICP Implementations
- https://github.com/richardos/icp
- https://github.com/norlab-ulaval/libpointmatcher
- http://andrewjkramer.net/lidar-odometry-with-icp/
- https://learnopencv.com/iterative-closest-point-icp-explained/
- https://www.roboticsproceedings.org/rss05/p21.pdf (Generalized-ICP)

### Occupancy Grid Mapping
- https://github.com/NekSfyris/occupancy_grid_mapping_2DLidar
- https://github.com/Swepz/LidarBasedGridMapping
- https://deepnote.com/publish/ff1841ef-b2dd-4a84-87ee-9cdab9f3cf6b

### Particle Filter / FastSLAM
- https://github.com/xiaofeng419/SLAM-2D-LIDAR-SCAN
- https://github.com/yashv28/Particle-Filter-SLAM
- https://github.com/yahsiuhsieh/particle-filter-slam
- https://github.com/kiran-mohan/PF_FastSLAM
- https://patrickyoussef.com/projects/pf-slam/

### IMU-assisted / Scan matching research
- https://isas.iar.kit.edu/pdf/RAL21_Li.pdf
- https://arxiv.org/pdf/1710.07104
- https://senseable.mit.edu/papers/pdf/20201020_Shan-etal_LIO-SAM_IROS.pdf
- https://github.com/methylDragon/ros-sensor-fusion-tutorial
- https://april.eecs.umich.edu/pdfs/olson2009icra.pdf (Real-time correlative scan matching)

### Pose-graph optimization / loop closure
- https://github.com/RainerKuemmerle/g2o
- https://github.com/uoip/g2opy
- https://gtbook.github.io/gtsam-examples/Pose2SLAMExample_g2o.html
- https://arxiv.org/pdf/1909.00903 (miniSAM)
- https://github.com/gisbi-kim/PyICP-SLAM
- https://research.google.com/pubs/archive/45466.pdf (Cartographer loop closure)

### Heavy-weight SLAM comparisons
- https://pmc.ncbi.nlm.nih.gov/articles/PMC9506160/ (2D SLAM characterization)
- https://arxiv.org/html/2501.09490v1 (SLAM comparison indoor)
- https://arxiv.org/html/2403.06341v1 (RTAB-Map open-source SLAM)
- https://github.com/cartographer-project/cartographer_ros/issues/276 (Pi config)
- https://medium.com/robotics-weekends/2d-mapping-using-google-cartographer-and-rplidar-with-raspberry-pi-a94ce11e44c5
- https://github.com/SteveMacenski/slam_toolbox
- https://adityakamath.github.io/2021-09-05-comparing-slam-methods/
- https://openslam-org.github.io/gmapping.html
