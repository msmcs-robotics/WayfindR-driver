# ROS LiDAR Processing Algorithms: Research Findings

**Context:** 2D 360-degree spinning LiDAR (LDRobot LD19, ~350-500 points/scan, 10 Hz)
on an indoor wheeled differential-drive robot. Targets: Raspberry Pi 3B (memory
constrained, 906 MB RAM, Cortex-A53) and Jetson Orin Nano. Current use case is
**wandering / obstacle avoidance** — NOT full SLAM. Any Python implementations
are strongly preferred for prototyping.

**Date:** 2026-04-09
**Scope:** Built-in ROS filters, denoising, downsampling, localization, local
planning, and 2D SLAM options.

---

## 1. `laser_filters` — Built-in ROS LaserScan Filter Plugins

The `laser_filters` package (`ros-perception/laser_filters`) provides a set of
filter plugins that consume and emit `sensor_msgs/LaserScan`. They are stacked
into a `FilterChain<sensor_msgs::LaserScan>` and executed inside the
`scan_to_scan_filter_chain` node. Each filter below is a C++ plugin; the chain
is configured via YAML parameter files (no coding required). Some filters run
on raw scans only; all of them work WITHOUT odometry.

### 1.1 LaserScanRangeFilter
- **What it does:** Replaces range readings outside `[lower_threshold, upper_threshold]`
  with NaN (or a configurable replacement). Used to drop "returns" from the
  robot chassis or spurious max-range values.
- **Computational cost:** O(N) per scan with N ~450 points. Negligible on Pi 3B.
- **Package:** `laser_filters` (C++)
- **Needs odometry?** No — pure scan filter.
- **Pros for wandering:** Eliminates self-returns (near wheels/LiDAR mount) and
  distant garbage beyond LD19's reliable ~8 m range. Trivial to tune.
- **Cons:** Doesn't address noise, only geometry.
- **Python equivalent:** Trivial — ~5 lines of NumPy (`ranges[ranges < lo] = np.nan`).

### 1.2 LaserScanSpeckleFilter
- **What it does:** Removes isolated "speckle" returns — points that have no
  neighbors within a distance threshold. Two variants exist: `Distance` mode
  (Euclidean distance to adjacent beam endpoints) and `RadiusOutlier` mode
  (PCL-style radius check).
- **Computational cost:** O(N) with a small constant (examines ±k neighbors).
  Real-time on Pi 3B.
- **Package:** `laser_filters`
- **Needs odometry?** No.
- **Pros for wandering:** LD19 occasionally produces single-beam fliers on glass,
  shiny surfaces, and at grazing angles. Removing these reduces false obstacle
  triggers.
- **Cons:** Can erase legitimate thin obstacles (e.g., chair legs at long range).
- **Python:** Straightforward to reimplement in NumPy (~20 lines).

### 1.3 LaserScanMedianFilter / LaserScanMedianSpatialFilter
- **What it does:** 1D median filter across either angle (spatial) or time
  (temporal across consecutive scans). Smooths range jitter without blurring
  sharp edges like a mean filter would.
- **Computational cost:** Low. Spatial median over a 5-element window on 450
  points is ~microseconds.
- **Package:** `laser_filters`
- **Needs odometry?** Temporal median technically assumes the robot hasn't moved
  much, but doesn't require a TF/odom source.
- **Pros:** Good first-line denoiser that preserves edges; catches single-point
  glitches without erasing narrow obstacles.
- **Cons:** 1D-only; won't catch 2-point clusters of noise.
- **Python:** `scipy.signal.medfilt(ranges, kernel_size=5)`.

### 1.4 LaserScanIntensityFilter
- **What it does:** Drops beams whose return intensity is outside
  `[lower, upper]`. Useful for rejecting over-saturated reflections from mirrors,
  retroreflectors, or very-low-signal returns from dark fabric.
- **Computational cost:** Negligible, O(N).
- **Package:** `laser_filters`
- **Needs odometry?** No.
- **Pros:** Effective on hardware that reports calibrated intensity.
- **Cons:** **LD19 does NOT report reliable intensity values** in its message —
  it reports a confidence byte per point which doesn't map directly to the ROS
  `intensities[]` convention. Marginally useful for our hardware.
- **Python:** Trivial.

### 1.5 LaserScanAngularBoundsFilter / LaserScanAngularBoundsFilterInPlace
- **What it does:** Keeps only beams within an angular window (e.g., front 180°).
  "InPlace" variant NaNs outside-window beams, normal variant shrinks the scan.
- **Computational cost:** Trivially cheap.
- **Package:** `laser_filters`
- **Needs odometry?** No.
- **Pros for wandering:** Our current `NaturalWanderBehavior` bins the full 360°
  into 36 buckets. If we decide front-biased wandering is safer, a
  90°-to-270° angular crop is one line of config.
- **Cons:** Loses rear-proximity safety info.
- **Python:** Array slicing.

### 1.6 LaserScanFootprintFilter
- **What it does:** Removes beams that hit the robot's own footprint polygon.
  Requires a TF from `base_link` to the scan frame and a footprint parameter.
- **Computational cost:** Low.
- **Needs odometry?** No, but needs TF.
- **Pros:** The right way to kill self-hits when the LiDAR sits inboard of the
  chassis perimeter.
- **Cons:** Extra TF plumbing. A simple `LaserScanRangeFilter` with
  `lower_threshold=0.15 m` is usually enough for a small bot.

### 1.7 ScanShadowsFilter
- **What it does:** Removes "veiling" points that occur at object edges — beams
  that arrive at grazing angles produce points that don't represent a real
  surface but a mix of foreground/background.
- **Computational cost:** O(N) with neighbor angular checks.
- **Needs odometry?** No.
- **Pros:** Important for SLAM map quality; valuable before converting to an
  occupancy grid.
- **Cons:** Less critical for bang-bang wandering.

---

## 2. `scan_to_scan_filter_chain` — The Pipeline Runner

- **What it does:** A minimal ROS node that wraps a `FilterChain<LaserScan>`
  and lets you stack any of the above filters via YAML config. Subscribes to
  `/scan`, publishes `/scan_filtered`.
- **Computational cost:** The sum of the chain + some overhead. You pay one
  de/serialization cost and multiple filter passes are in-process (no extra ROS
  traffic).
- **Package:** `laser_filters`
- **Needs odometry?** No (unless a filter in the chain specifies a TF target,
  e.g., FootprintFilter).
- **Configuration pattern:** YAML list of `{name, type, params}` entries; order
  matters (range -> shadow -> speckle -> median is a common recipe).
- **Pros:** Best practice — compute filters once, multiple consumers subscribe
  to the filtered topic. Easy to A/B test chains at runtime.
- **Cons:** Requires ROS; Python equivalent is just a single node subscribing
  and publishing.
- **Python port:** We could ship a `pathfinder.scan_filter_chain` module that
  applies an ordered list of callables to a NumPy `ranges` array.

---

## 3. Denoising Beyond `laser_filters`

### 3.1 Statistical Outlier Removal (SOR) — PCL / pcl_ros
- **What it does:** For each point, computes mean distance to its K nearest
  neighbors. If that mean distance is more than `(mu + std_dev_mul * sigma)`
  across all points, the point is dropped. Originally a PCL 3D point-cloud
  filter, commonly applied to 2D LiDAR by converting scan to a (x, y) point
  cloud first.
- **Computational cost:** Requires a KD-tree or nearest-neighbor structure
  over N points. For N=450, still sub-millisecond on Pi 3B. If run in PCL C++,
  essentially free; in Python via `scipy.spatial.cKDTree`, ~1-3 ms.
- **Package:** `pcl` / `pcl_ros` (C++). Python: `open3d`, or a custom
  scipy implementation.
- **Needs odometry?** No.
- **Pros:** Considered state-of-the-art for robust LaserScan denoising when the
  built-in `SpeckleFilter` isn't enough. Particularly effective before
  clustering.
- **Cons:** Full PCL on Pi 3B has a memory footprint; prefer a small scipy
  re-implementation for us.
- **Advanced variants:** DSOR (Dynamic SOR) and IDSOR are designed for snow/rain
  on outdoor LiDAR — not relevant for indoor use.

### 3.2 Radius Outlier Removal (ROR) — PCL
- **What it does:** Drops a point if it has fewer than `min_neighbors` within a
  fixed radius. Simpler than SOR.
- **Computational cost:** Same order as SOR.
- **Package:** `pcl`.
- **Needs odometry?** No.
- **Pros:** Simple to tune (two parameters). Good for sparse scans.
- **Cons:** No adaptation to local point density.
- **Python:** ~15 lines with `cKDTree`.

---

## 4. Downsampling Techniques

### 4.1 Angular Slicing / Decimation
- **What it does:** Keep every k-th beam. Reduces 450 points to, e.g., 72.
- **Computational cost:** Free (array slicing).
- **Package:** None required.
- **Needs odometry?** No.
- **Pros:** Dirt simple. Our current `NaturalWanderBehavior` 36-bucket binning
  is basically a more sophisticated version of this.
- **Cons:** Aliasing: a small obstacle between kept beams is completely missed.

### 4.2 Angular Binning with min-range reduction (what we already do)
- **What it does:** Bin beams into angular sectors; in each sector, reduce to
  min (closest obstacle), mean, or median range.
- **Computational cost:** O(N). Trivially fast.
- **Needs odometry?** No.
- **Pros:** Anti-aliasing (every beam is considered); preserves safety-critical
  closest-obstacle info. This is our current approach.
- **Cons:** Fixed bucket count doesn't adapt to scene.

### 4.3 PCL VoxelGrid Filter (2D)
- **What it does:** Projects the scan into (x, y), overlays a regular grid
  (voxel size e.g. 5 cm), and replaces all points in a cell with their centroid.
- **Computational cost:** O(N) with hashing. Negligible for N=450.
- **Package:** `pcl` / `pcl_ros`.
- **Needs odometry?** No.
- **Pros:** Spatially uniform downsampling (unlike angular decimation, which is
  dense near the sensor and sparse far away). Standard SLAM preprocessing step.
- **Cons:** Only a modest win for 450-point scans; designed for 3D clouds of
  hundreds of thousands of points.
- **Python:** Easy with `numpy.unique` over integer-quantized (x, y) pairs.

### 4.4 Random Sampling / Farthest Point Sampling
- **What it does:** Randomly or greedily pick K points. FPS gives better
  coverage than random.
- **Computational cost:** Random is O(N). FPS is O(K*N) — for K=50, N=450,
  still fine.
- **Pros:** Fixed output size regardless of scan density.
- **Cons:** Not standard in ROS; better suited to ML feature extraction.

---

## 5. Localization (AMCL)

### 5.1 AMCL — Adaptive Monte Carlo Localization
- **What it does:** Particle filter that tracks robot pose on a known
  occupancy map by resampling particles based on how well simulated laser scans
  match the real scan. "Adaptive" = KLD sampling dynamically adjusts particle
  count (fewer when confident, more when lost).
- **Computational cost:** Dominated by the number of particles and the number
  of beams used per particle. Typical config: 500-2000 particles, 30-60
  beams/scan after subsampling. On Pi 3B it is feasible with a tuned
  configuration (`max_particles` 1000, `laser_max_beams` 30, `update_min_d`
  0.2 m) — several Pi-based robotics tutorials (UGV Rover PI, Waveshare) run
  Nav2 AMCL on a Pi 4. Pi 3B will struggle at high particle counts.
- **Package:** `nav2_amcl` (ROS2), `amcl` (ROS1). C++ only.
- **Needs odometry?** **YES — AMCL is a tracking filter, not a SLAM
  system. It REQUIRES wheel or visual odometry** to propagate particles between
  scan updates. It also requires a prior map.
- **Pros:** Battle-tested, the de facto 2D localization in ROS.
- **Cons:** Not relevant to us right now — **we have no odometry and no map**.
  Listed here for completeness as a future option once encoders/IMU-integration
  exist and we run SLAM offline to build a map.
- **Python:** None of note; AMCL is always used as a packaged C++ node.

---

## 6. Local Planners / Obstacle Avoidance

### 6.1 DWA (Dynamic Window Approach) — `dwa_local_planner` / `nav2_dwb_controller`
- **What it does:** At each control step, samples a "dynamic window" of
  admissible (v, omega) pairs reachable within one cycle given accel limits,
  rolls each out for a short horizon, scores each trajectory against a cost
  function combining (a) obstacle distance from the costmap, (b) goal progress,
  (c) path following, and picks the winner.
- **Key parameters:** `path_distance_bias` (default 32.0), `goal_distance_bias`
  (24.0), `occdist_scale` (0.01 — raise this to make obstacle avoidance more
  aggressive), `forward_point_distance` (0.325), `stop_time_buffer` (0.2),
  `sim_time`, `vx_samples`, `vth_samples`.
- **Computational cost:** Moderate. O(vx_samples * vth_samples * horizon_steps)
  per cycle. Real-time on Pi 3B with modest sample counts (6x20 grid,
  1.7 s horizon). Runs fine on Pi 4/Jetson at full config.
- **Package:** `dwa_local_planner` (ROS1), `dwb_controller` (Nav2 / ROS2). C++.
- **Needs odometry?** Yes — needs current velocity estimate and costmap
  updates. The costmap itself needs TF to localize the scan.
- **Pros:** Standard and well understood; obstacle avoidance "just works" once
  tuned.
- **Cons:** Full Nav2 stack is overkill for wandering. Needs a global goal and
  costmap — we would have to stub both.
- **Python:** No canonical Python DWA in ROS. Several educational
  implementations on GitHub (AtsushiSakai/PythonRobotics) that are <200 lines
  and easy to port.

### 6.2 Vector Field Histogram (VFH / VFH+ / VFH*)
- **What it does:** Builds a polar histogram of obstacle density around the
  robot, binarizes it against thresholds to find "valleys" of free directions,
  then (in VFH+) applies kinematic constraints to pick a steerable heading.
- **Computational cost:** Very low. O(N) scan conversion + O(bins) valley
  search. Real-time on Pi 3B trivially.
- **Package:** Not in mainline ROS. MATLAB has `controllerVFH`. Open source
  C++/Python ports exist (PythonRobotics has a clean Python VFH).
- **Needs odometry?** No — a pure reactive scheme. Optional odom for global
  goal-directed variants.
- **Pros:** **Excellent fit for our current use case.** Takes raw LaserScan,
  outputs a heading — no map, no odometry, no global planner. Computationally
  cheap. Historically the "obstacle avoidance for dense clutter" algorithm.
  Very close in spirit to our `NaturalWanderBehavior`.
- **Cons:** Can get stuck in local minima / U-traps (VFH+T variant addresses
  this). No explicit velocity planning — just heading.
- **Python:** Yes — `PythonRobotics/PathPlanning/VectorFieldHistogram/`.
  Suitable for direct integration into `pathfinder/behaviors.py`.

### 6.3 TEB Local Planner (mentioned for completeness)
- **What it does:** Time-Elastic Band optimization over continuous trajectories.
- **Cost:** Heavier than DWA. Not recommended for Pi 3B.
- **Python:** No.

---

## 7. 2D SLAM Options (Future Work)

### 7.1 Hector SLAM (`hector_mapping`)
- **What it does:** Scan-matching (Gauss-Newton) against a multi-resolution
  occupancy map. Estimates 2D pose at the sensor scan rate.
- **Computational cost:** Moderate — needs high-rate LiDAR scans (LD19's 10 Hz
  is borderline; 30-40 Hz preferred). Runs on Pi 3B+ for small rooms.
- **Package:** `hector_slam` (ROS1 primarily; community ROS2 ports). C++.
- **Needs odometry?** **NO — this is Hector's killer feature.** Laser-only
  SLAM that does not require wheel/IMU odometry. Perfectly matches our current
  hardware state (no encoders yet).
- **Pros:** No-odometry operation. Simple to set up. Handles roll/pitch.
- **Cons:** No explicit loop closure (drifts in long corridors). Struggles in
  feature-poor environments (long blank halls). Lower update rate amplifies
  drift — the LD19's 10 Hz is not ideal.
- **Python:** No Python implementation; always used as C++ node.

### 7.2 GMapping (`slam_gmapping`)
- **What it does:** Rao-Blackwellized particle filter SLAM with scan matching.
- **Cost:** Moderate. Feasible on Pi 3B/Pi 4 for small maps.
- **Package:** `slam_gmapping`. C++.
- **Needs odometry?** **YES.** Gmapping is not a no-odom option.
- **Pros:** Produces better maps than Hector in benchmarks.
- **Cons:** Requires odometry we don't have. Legacy; mostly superseded by
  `slam_toolbox` in ROS2.

### 7.3 SLAM Toolbox (`slam_toolbox`)
- **What it does:** Pose-graph SLAM with Karto scan matcher underneath.
  Supports online (async/sync), offline mapping, lifelong mapping, and
  localization-only modes.
- **Computational cost:** Lightweight at mapping time; more memory for the
  pose graph. Runs acceptably on Pi 4; on Pi 3B 906 MB RAM is very tight.
- **Package:** `slam_toolbox`. C++.
- **Needs odometry?** Yes (strongly recommended). Has scan-matching-only fallback
  but quality degrades.
- **Pros:** The currently-maintained ROS2 standard for 2D SLAM. Best feature
  set of the three. Has a `localization` mode analogous to AMCL (uses a saved
  pose graph).
- **Cons:** Heavyweight for a Pi 3B; odom-required in practice.
- **Python:** No; C++ node.

### 7.4 Google Cartographer
- **What it does:** Submap-based pose-graph SLAM with very good scan matching
  and loop closure.
- **Computational cost:** Highest of this group. **Pi 3B cannot run it
  comfortably** — map TF rate drops below tolerance within minutes per reports.
  Workarounds: enlarge voxel filter, lower submap resolution, reduce
  optimization frequency. Practical recipe is to run Cartographer on Jetson or
  a remote desktop while the Pi just streams `/scan`.
- **Package:** `cartographer_ros`. C++.
- **Needs odometry?** Optional. Cartographer is one of the better "laser-only"
  SLAM options, though odom helps.
- **Pros:** Best mapping accuracy in benchmarks. Robust loop closure.
- **Cons:** Heavy. Will choke a Pi 3B. Complex config.
- **Python:** No.

### 7.5 Benchmark Summary (from literature)
| Algorithm | Accuracy | CPU | RAM | No-odom | ROS2 maintained |
|-----------|----------|-----|-----|---------|-----------------|
| Hector SLAM | Medium | Low | Low | **Yes** | Community ports |
| GMapping | Medium | Low-Med | Low | No | Legacy |
| SLAM Toolbox | High | Medium | Medium | Partial | **Yes, primary** |
| Cartographer | Highest | High | High | Yes | Yes |

For our Pi 3B + LD19 + no-odometry setup, **Hector SLAM is the only practical
choice** today. Once we add wheel encoders or IMU-based odometry and move SLAM
work to the Jetson, SLAM Toolbox becomes the right answer.

---

## 8. LD19 ROS2 Drivers

### 8.1 `ldlidar_stl_ros2` (official)
- **Maintainer:** `ldrobotSensorTeam/ldlidar_stl_ros2`
- **Supports:** LD06, LD19, LD14, STL-19P, STL-27L
- **Launch:** `ros2 launch ldlidar_stl_ros2 ld19.launch.py`
- **Pros:** Official, actively maintained, supports Foxy/Humble/Jazzy.
- **Cons:** Vendor code quality; some users prefer alternatives.

### 8.2 `ldlidar_ros2` (newer unified driver)
- **Maintainer:** `ldrobotSensorTeam/ldlidar_ros2`
- Newer unified package. Supports the same model range.

### 8.3 `ldrobot-lidar-ros2` by Myzhar
- **Maintainer:** `Myzhar/ldrobot-lidar-ros2`
- **Key feature:** Uses ROS2 Lifecycle nodes (configure/activate/deactivate).
- **Pros:** Cleaner architecture; better for production systems with managed
  startup.
- **Cons:** Third-party.

### 8.4 `richardw347/ld19_lidar`
- Minimal single-file ROS2 node for LD19 only.
- **Pros:** Simple, easy to understand and modify.
- **Cons:** Single-model, less polished.

### 8.5 Our current driver — `pathfinder/lidar_ld19.py`
- Pure Python, no ROS dependency, direct UART at 230400 baud.
- **Pros:** No ROS overhead, integrates with our existing
  `demos_common.sensors.create_lidar()` factory. Works on the Pi 3B today.
- **Cons:** Not interoperable with the ROS ecosystem. If we ever move to ROS2
  on the Jetson, we should adopt `ldlidar_stl_ros2` or `Myzhar/ldrobot-lidar-ros2`
  and treat our Python driver as the non-ROS fallback.

---

## 9. Recommendations for WayfindR / AMBOT

Given: no odometry yet, Pi 3B memory constraints, wandering use case, Python
preferred, and Jetson Orin Nano available for heavier work.

### Short term (Pi 3B, no ROS)
1. **Port the `laser_filters` chain concept into Python.** A tiny
   `pathfinder/scan_filters.py` module applying in order:
   - Range filter (drop self-hits < 0.15 m, drop > 8 m)
   - Median filter (scipy.signal, k=5) — replaces
     isolated jitter
   - Speckle / radius-outlier filter — NumPy neighbor check
   - Angular binning (already present in `NaturalWanderBehavior`)
2. **Consider replacing the current peak-picking wander heuristic with a VFH+
   implementation** from `PythonRobotics`. VFH is the closest textbook match
   to what we're doing and would give us a well-understood tuning surface.

### Medium term (still Pi, still no odom)
3. **Stand up Hector SLAM on the Pi** (or more realistically, Jetson) just
   to build a room map offline. No odom required. Use the map as a reference
   even if not for live navigation.

### Long term (Jetson-first, with odometry)
4. **Migrate scan processing to ROS2 on the Jetson.** Use
   `ldlidar_stl_ros2` as the driver, `scan_to_scan_filter_chain` as the
   pipeline, `slam_toolbox` for mapping, and either `nav2_amcl` or
   `slam_toolbox localization mode` for live pose.
5. **Once encoders land**, DWA or DWB becomes the obstacle avoidance layer with
   SLAM Toolbox providing the map.

### Things explicitly NOT recommended now
- Cartographer on the Pi 3B (memory-starved, will miss real-time).
- AMCL before we have both odometry and a map.
- Full Nav2 stack for simple wandering.
- pip `opencv-python-headless`-style PCL installs on the Pi — too heavy.

---

## 10. Relevant Code in Our Repo

- `/home/devel/WayfindR-driver/ambot/pathfinder/lidar_ld19.py` — LD19 driver
- `/home/devel/WayfindR-driver/ambot/pathfinder/lidar.py` — generic wrapper
- `/home/devel/WayfindR-driver/ambot/pathfinder/behaviors.py` —
  `NaturalWanderBehavior` (36-bin peak picker), the thing VFH would replace.
- `/home/devel/WayfindR-driver/ambot/pathfinder/obstacle_detector.py` —
  obstacle logic to potentially rebuild on top of a filter chain.
- `/home/devel/WayfindR-driver/ambot/demos_common/sensors.py` —
  `create_lidar()` factory where a filter chain hook could live.

---

## Sources

- [laser_filters ROS Package Overview](https://index.ros.org/p/laser_filters/)
- [laser_filters GitHub (ros-perception)](https://github.com/ros-perception/laser_filters)
- [laser_filters_plugins.xml (ros2 branch)](https://github.com/ros-perception/laser_filters/blob/ros2/laser_filters_plugins.xml)
- [range_filter_example.yaml](https://github.com/ros-perception/laser_filters/blob/ros2/examples/range_filter_example.yaml)
- [Filter Laser Scans — Stretch Documentation](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_2/)
- [The Ultimate ROS1 Guide to Laser Filters — JohnTGZ](https://johntgz.github.io/2022/01/09/the_ultimate_guide_to_laser_filters/)
- [scan_to_scan_filter_chain source (ros2)](https://github.com/ros-perception/laser_filters/blob/ros2/src/scan_to_scan_filter_chain.cpp)
- [ROS Package Index: sensor_filters](https://index.ros.org/p/sensor_filters/)
- [DSOR: Scalable Statistical Filter for LiDAR](https://arxiv.org/pdf/2109.07078)
- [PCL Statistical Outlier Removal tutorial](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/)
- [Clustering Denoising of 2D LiDAR Scanning (MDPI Sensors)](https://www.mdpi.com/1424-8220/23/1/18)
- [PCL VoxelGrid Tutorial](https://pointclouds.org/documentation/tutorials/voxel_grid.html)
- [AMCL — Robotics Knowledgebase](https://roboticsknowledgebase.com/wiki/state-estimation/adaptive-monte-carlo-localization/)
- [Nav2 Mapping and Localization](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [UGV Rover PI ROS2 Auto Navigation (Waveshare)](https://www.waveshare.com/wiki/UGV_Rover_PI_ROS2_6._Auto_navigation)
- [dwa_local_planner ROS Wiki](http://library.isr.ist.utl.pt/docs/roswiki/dwa_local_planner.html)
- [ros-nav-5days LOCAL PLANNER guide](https://github.com/rwbot/ros-nav-5days/blob/master/LOCAL%20PLANNER.md)
- [ROS-Based Navigation and Obstacle Avoidance (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12300016/)
- [Vector Field Histogram — Wikipedia](https://en.wikipedia.org/wiki/Vector_Field_Histogram)
- [VFH+T Trap Avoidance paper](https://www.sciencedirect.com/science/article/pii/S2590123024008806)
- [2D Mapping with Cartographer + RPLidar on Raspberry Pi (Medium)](https://medium.com/robotics-weekends/2d-mapping-using-google-cartographer-and-rplidar-with-raspberry-pi-a94ce11e44c5)
- [Cartographer Lightweight Configuration for Raspberry Pi (issue #276)](https://github.com/googlecartographer/cartographer_ros/issues/276)
- [Comparing different SLAM methods — Aditya Kamath](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/)
- [Benchmarking of 2D SLAM Algorithms (ACRO PDF)](http://www.acro.be/downloadvrij/Benchmark_2D_SLAM.pdf)
- [2D SLAM Algorithms Characterization (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC9506160/)
- [Comparison of Various SLAM Systems (arXiv 2501.09490)](https://arxiv.org/html/2501.09490v1)
- [hector_mapping Wiki](http://library.isr.ist.utl.pt/docs/roswiki/hector_mapping.html)
- [RPLidar_Hector_SLAM (NickL77)](https://github.com/NickL77/RPLidar_Hector_SLAM)
- [hector_slam_quickstart (avs2805)](https://github.com/avs2805/hector_slam_quickstart)
- [SLAM Toolbox (Steve Macenski)](https://github.com/SteveMacenski/slam_toolbox)
- [slam_toolbox docs (Humble)](https://docs.ros.org/en/humble/p/slam_toolbox/)
- [Nav2 Navigating While Mapping tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)
- [ldlidar_stl_ros2 (LDRobot official)](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
- [ldlidar_ros2 (LDRobot unified)](https://github.com/ldrobotSensorTeam/ldlidar_ros2)
- [ldrobot-lidar-ros2 (Myzhar, Lifecycle nodes)](https://github.com/Myzhar/ldrobot-lidar-ros2)
- [ld19_lidar (richardw347)](https://github.com/richardw347/ld19_lidar)
- [LD19 Development Manual v2.3 (Elecrow PDF)](https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf)
- [pointcloud_to_laserscan (ros-perception)](https://github.com/ros-perception/pointcloud_to_laserscan)
- [How to read LaserScan data (The Construct)](https://www.theconstruct.ai/read-laserscan-data/)
- [laser_geometry ROS Package](https://index.ros.org/p/laser_geometry/)
