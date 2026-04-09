# Video Game Development Algorithms Applied to LiDAR Point Cloud Processing

**Date:** 2026-04-09
**Target Hardware:** Raspberry Pi 3B (constrained), Jetson Orin Nano (GPU capable)
**Sensor:** LD19 2D 360-degree LiDAR, 350-500 points/scan at 10 Hz
**Use Case:** Wandering / obstacle avoidance for wheeled robot

---

## Executive Summary

Game engines have spent decades optimizing real-time processing of large numbers of moving points (particles, projectiles, entities). Many of these techniques map directly to 2D LiDAR processing:

- **Spatial hashing** and **quadtrees** give O(log n) or near-O(1) neighbor queries instead of O(n^2) brute force.
- **LOD** reduces processing for far-away points — LiDAR's "far = less safety-critical" mirrors games' "far = less visually important".
- **Temporal coherence** (reusing prior frame data) is standard in both rendering denoisers and particle rendering, and directly enables temporal median filtering for LiDAR.
- **Frustum culling** maps 1:1 to "only process points in direction of travel".
- **DBSCAN** is the de-facto clustering method for both particle-based content classification and LiDAR object detection.
- **Kalman filtering** is already used in both game tracking and LiDAR odometry.
- **GPU particle systems** demonstrate that thousands of points can be processed in parallel — the Jetson Orin Nano can do this trivially for 500 LiDAR points.

For our 500-points-at-10-Hz workload, **we're not computationally bound on the Jetson**, and even on the Pi 3B most of these techniques will fit easily in budget. The real wins come from algorithmic simplicity, memory locality, and enabling higher-level behaviors (object tracking, temporal denoising).

---

## 1. Particle System Optimization (General)

**Original use case:** Games render 10k-1M+ particles per frame (smoke, fire, sparks, debris). Unreal, Unity, and custom engines aggressively optimize this through LOD, culling, batching, and GPU offload.

**Core ideas that transfer to LiDAR:**
- Avoid per-point work when possible; batch operations.
- Use structure-of-arrays (SoA) memory layout over array-of-structures (AoS) for cache efficiency (matters more when point counts climb).
- Do expensive computation in chunks or amortized over frames.
- Prefer fixed-size buffers over dynamic allocation in the hot loop.

**Applies to LiDAR how:**
LD19 gives ~467 points per scan. Python `list` of tuples is AoS — switching to numpy arrays (SoA) for `angles[]`, `ranges[]`, `x[]`, `y[]` gives vectorized operations and better cache behavior.

**Complexity:** O(n) per scan regardless; constant-factor improvement.
**Pi 3B:** Trivially runs. numpy is already used.
**Jetson GPU:** Unnecessary at this scale.
**Code complexity:** Simple Python / numpy.
**Pros:** Free performance, cleaner code.
**Cons:** None material.

---

## 2. Spatial Hashing (2D)

**Original use case:** Broad-phase collision detection in 2D games. Divide world into fixed-size grid cells, hash each object by its cell coordinates. Neighbor queries check only the current cell and 8 adjacent cells instead of all N objects.

**Algorithm:**
```
cell = (floor(x / cell_size), floor(y / cell_size))
hash_table[cell].append(point_index)
# Query: look up cell + 8 neighbors, check distance
```

**Applies to LiDAR how:**
After converting polar-to-cartesian, bin points into a fixed spatial grid. This makes neighbor queries (needed for DBSCAN, outlier detection, point density checks) go from O(n^2) to roughly O(n) total for all-pairs queries with reasonable cell size.

**Complexity:**
- Build: O(n)
- Single neighbor query: O(k) where k is points in local cells (typically small)
- All-pairs: O(n * k)

**Pi 3B:** Fine. For 500 points this is microseconds. Can be pure Python dict or numpy-based.
**Jetson GPU:** GPU spatial hashing is a solved problem (see Green's CUDA particles paper) but overkill here.
**Code complexity:** Simple Python — maybe 50 lines.
**Pros:**
- Drastically speeds up any per-point neighbor algorithm.
- Cell size is the only tuning parameter (make it ~2x expected cluster radius).
- Trivially supports dynamic updates (one scan per frame).
**Cons:**
- For only 500 points the brute-force O(n^2) = 250k ops is already fast enough on numpy. Spatial hashing matters more for DBSCAN's repeated neighbor queries.

**Recommendation:** **Worth implementing** if we add DBSCAN clustering or statistical outlier removal, because those perform many neighbor queries.

---

## 3. Quadtree / Octree Spatial Partitioning

**Original use case:** Hierarchical space subdivision used in game engines for broad-phase collision, visibility culling, nearest-neighbor lookup. Quadtree for 2D, octree for 3D. Splits space recursively when cell point count exceeds a threshold.

**Applies to LiDAR how:**
Build a quadtree per scan (or incrementally across scans) for fast range and nearest-neighbor queries. Useful for:
- "What's the nearest obstacle to my planned path point?"
- "How many points are within radius r of robot center?"
- Map-building (occupancy grid alternative).

**Complexity:**
- Build: O(n log n)
- Point query: O(log n)
- Range query: O(log n + k)

**Pi 3B:** OK but Python's function-call overhead makes quadtrees slow unless you use a C-backed library (scipy.spatial.cKDTree is a better alternative — KD-tree, similar behavior).
**Jetson GPU:** GPU quadtrees exist but are complex and unnecessary at 500 points.
**Code complexity:** Moderate if written from scratch (~150 lines Python). **Just use `scipy.spatial.cKDTree`** — already in scipy, C-implemented, supports radius and k-NN queries.
**Pros:**
- Best asymptotic complexity for neighbor queries.
- scipy provides a zero-effort C implementation.
- Reusable for path planning (nearest obstacle along trajectory).
**Cons:**
- Rebuild cost each scan (though O(n log n) for 500 points is negligible).
- For only 500 points, spatial hashing is equally fast and simpler.

**Recommendation:** **Use scipy.spatial.cKDTree** — one-liner and faster than writing our own. It's the default answer for "I need neighbor queries on a point cloud."

---

## 4. Point Cloud Denoising (Filter-Based, from Game + Research)

**Original use cases:**
- Game rendering: bilateral filters for edge-preserving denoising of rendered frames (TAA, SVGF).
- Point cloud research: bilateral filters extended to 3D points, median filters for salt-and-pepper noise, statistical outlier removal.

**Applies to LiDAR how:**
Three classes we care about:
1. **Statistical outlier removal** (PCL/Open3D style): For each point, compute mean distance to k-NN; reject points whose mean distance is > std_ratio * global_mean.
2. **Radius outlier removal**: Reject any point with fewer than N neighbors within radius r.
3. **Median/Bilateral filter on range**: Treat the 1D angle->range function as a signal, apply 1D median to eliminate single-point spikes.

**Complexity:**
- Statistical outlier: O(n log n) with KD-tree, O(n^2) brute force.
- Radius outlier: same.
- 1D median filter on range array: O(n * window_size), trivially fast.

**Pi 3B:** All three are fine for 500 points. 1D median filter is the cheapest — `scipy.ndimage.median_filter(ranges, size=3)` in a single call.
**Jetson GPU:** Not needed.
**Code complexity:** Trivial (1D median is one line). Statistical outlier needs KD-tree but scipy provides it.
**Pros:**
- 1D median filter is literally a one-liner and kills isolated spike points instantly.
- Statistical outlier removal is standard for LiDAR and well-tested.
**Cons:**
- Median filter on the polar range signal ignores the 360->0 degree wraparound — handle explicitly with `mode='wrap'` or manual padding.
- Aggressive filters destroy small-object returns.

**Recommendation:** **Start with 1D median filter on ranges** (simplest, biggest win) and add statistical outlier removal via cKDTree if we see residual noise.

---

## 5. Level of Detail (LOD)

**Original use case:** Render distant objects with fewer polygons/particles than nearby ones. Continuous LOD (CLOD) varies resolution smoothly; discrete LOD switches between pre-built versions. Modern: Nanite's cluster-based approach.

**Applies to LiDAR how:**
Points far from the robot are less safety-critical and can be processed at reduced fidelity:
- **Distance-based subsampling**: Keep all points within 1m, every 2nd point within 3m, every 4th point beyond.
- **Angular LOD**: Use high angular resolution in the direction of travel (front 90 degrees) and coarse resolution elsewhere.
- **Reduced precision**: Use int16 range (cm) instead of float32 for far points.

**Complexity:** O(n) one-pass subsample.
**Pi 3B:** Helps if later stages are slow — free. Reduces work for all downstream steps.
**Jetson GPU:** Not needed.
**Code complexity:** Trivial, a few lines of numpy boolean indexing.
**Pros:**
- Reduces downstream cost for every subsequent algorithm.
- Matches the physical reality that close obstacles are more urgent.
**Cons:**
- Can hide dynamic obstacles at range (a person appearing at 5m has fewer hits).
- Wandering demo doesn't really need this — LD19 at 500 points is already coarse.

**Recommendation:** **Skip for now.** 500 points is already low; LOD doesn't help. Revisit if we ever use a denser 3D LiDAR.

---

## 6. Real-Time Clustering (DBSCAN)

**Original use cases:**
- Games: grouping player footprints, detecting swarms, procedural content generation clustering.
- LiDAR: the standard method for obstacle segmentation in 2D scans.

**Algorithm:** For each point, check if it has >= minPts neighbors within eps; if so, it's a core point. Connected core points form a cluster; points with no cluster membership are "noise."

**Applies to LiDAR how:**
Convert scan to (x,y) cartesian, run DBSCAN with eps ~10-30cm and minPts=3-5. Each resulting cluster is a candidate "object" (wall segment, person, chair leg, etc). Gives us a much richer world model than raw points — enables behaviors like "approach the nearest object" or "avoid the thing that's moving."

**Complexity:**
- Naive: O(n^2)
- With KD-tree / spatial hash: O(n log n)

**Pi 3B:** sklearn's DBSCAN with `algorithm='kd_tree'` handles 500 points in low single-digit milliseconds. Very feasible at 10 Hz.
**Jetson GPU:** RAPIDS cuML provides GPU DBSCAN but it's massive overkill for 500 points.
**Code complexity:** `sklearn.cluster.DBSCAN` is a one-liner. No custom code needed.
**Pros:**
- Well-understood, widely used for LiDAR.
- Handles arbitrary shapes (unlike k-means).
- Noise points automatically flagged as outliers — doubles as denoising.
- Only two tunable parameters (eps, minPts).
**Cons:**
- Parameter sensitivity: eps depends on expected obstacle spacing.
- Treats the 360->0 wraparound as discontinuous in cartesian (but since DBSCAN is density-based in 2D cartesian space, this is automatically handled — it's only the polar representation that wraps).
- Doesn't track clusters across frames (needs pairing with Kalman/Hungarian for object tracking).

**Recommendation:** **High value, easy to add.** sklearn DBSCAN on cartesian points gives us object segmentation for free. This unlocks much smarter wandering behaviors than the current 36-bucket approach.

---

## 7. Frustum Culling (2D)

**Original use case:** Skip drawing/processing objects outside the camera's view frustum. In 2D games (side-scrollers), reduces to "is the object inside the visible rectangle?"

**Applies to LiDAR how:**
For obstacle avoidance during forward motion, only the front ~120-180 degree arc matters. Points behind the robot don't affect immediate pathing. Cull them early.

**Algorithm:**
```python
heading = robot.heading
relative_angles = (scan_angles - heading + pi) % (2*pi) - pi
mask = np.abs(relative_angles) < (fov / 2)
```

**Complexity:** O(n), one vectorized pass.
**Pi 3B:** Free.
**Jetson GPU:** Not needed.
**Code complexity:** 3 lines of numpy.
**Pros:**
- Cuts downstream work roughly in half.
- Matches how humans/animals navigate — you don't stare behind you while walking forward.
- Compose with LOD for even more savings.
**Cons:**
- Loses rear-facing safety info. For a 2D robot that may reverse or rotate in place, you still need the full 360 occasionally.
- Current NaturalWanderBehavior uses the full 360 scan to pick "longest gap" targets — culling would break it.

**Recommendation:** **Conditional use.** Use narrow FOV for collision checks in the immediate path, but keep full 360 for target selection. Don't apply globally.

---

## 8. Temporal Coherence / Previous Frame Data

**Original use case:** Real-time rendering denoisers (TAA, SVGF, Intel Open Image Denoise 3) reuse the previous denoised frame, reprojected by motion vectors, to accumulate samples across time. Massive quality improvement with minimal per-frame cost. Particle rendering similarly reuses prior frame particle states.

**Applies to LiDAR how:**
LD19 at 10 Hz gives us 100ms between scans. Over 3 frames (300ms) the robot has moved at most a few cm at walking speed. We can:
1. **Temporal median filter**: Keep a ring buffer of the last N scans (indexed by angle bin). For each angular bin, output the median of the last N range values. Eliminates transient noise and one-off spikes.
2. **Scan accumulation**: Merge multiple scans into a denser map (especially useful since LD19 is sparse).
3. **Moving obstacle detection**: Points that are consistent across frames = static. Points that appear/disappear = moving or noise.

**Complexity:** O(n * N) where N is buffer depth. For N=5 and n=500, that's 2500 ops — trivial.
**Pi 3B:** Easy. Ring buffer of numpy arrays.
**Jetson GPU:** Not needed.
**Code complexity:** Simple — ~30 lines of Python.
**Pros:**
- Temporal median filtering is the single most effective denoising method for rotating LiDAR.
- Also gives us "motion detection" for free (static vs. transient hits).
- Already proven on LD19 (see GitHub Lidar_filter repo).
**Cons:**
- Adds latency: a 5-frame median delays obstacle detection by ~250ms. Tune buffer size to balance lag vs. smoothness.
- Assumes slow robot motion; fast turns blur the accumulated map. Can be mitigated by transforming prior scans by odometry (requires IMU integration).

**Recommendation:** **High value.** Temporal median filter with N=3 frames is probably the best single improvement we can make. 30 lines of code and dramatically cleaner scans.

---

## 9. Kalman Filter / Smoothing

**Original use case:** Game AI uses Kalman filters for tracking moving targets (missile lock-on, enemy position prediction) and for smoothing noisy input (mouse, joystick, networked player positions). Core technique in satellite nav and robotics.

**Applies to LiDAR how:**
Not for raw point smoothing — that's what median filters are for. Kalman shines for **object tracking across frames**:
1. Run DBSCAN to get cluster centroids.
2. Associate centroids between frames (nearest-neighbor or Hungarian).
3. Run a Kalman filter per tracked cluster to estimate position and velocity.
4. Robot now knows "that blob is moving left at 0.3 m/s" — enables predictive avoidance.

Also useful for **IMU fusion**: Kalman-filter fuse wheel odometry + MPU6050 gyro + LiDAR scan matching to get smooth robot pose.

**Complexity:** O(k) per frame where k is number of tracked objects (typically <20).
**Pi 3B:** Fine. `filterpy` is a pure Python library that works here. scipy has no built-in.
**Jetson GPU:** Not needed.
**Code complexity:** Moderate. Constant-velocity Kalman is ~50 lines. Data association (pairing clusters frame-to-frame) is the tricky part.
**Pros:**
- Principled uncertainty estimates.
- Enables prediction (where will obstacle be in 200ms?).
- Well-studied, tons of examples.
**Cons:**
- Requires clustering first (chicken-and-egg with DBSCAN).
- Data association is error-prone; wrong association = garbage tracks.
- Linear Gaussian assumption; for fast-turning robots, use Extended or Unscented variants.

**Recommendation:** **Future work.** Add after DBSCAN clustering is in place and we want predictive behaviors. Not needed for basic wandering.

---

## 10. GPU Particle System Algorithms (Parallel)

**Original use case:** Modern games simulate 100k-1M particles per frame on GPU using compute shaders. CUDA/DirectCompute kernels do force calculation, collision, integration, and binning in parallel. Green's NVIDIA CUDA particles sample is the canonical reference.

**Applies to LiDAR how:**
On Jetson Orin Nano (8 SMs, Ampere, CUDA 12.6), we could GPU-accelerate:
- Polar to cartesian (trivial, per-point kernel).
- Spatial hash binning.
- DBSCAN (cuML has GPU version).
- All-pairs distance matrices.

**Complexity:** O(n/p) where p is parallel lanes. For 500 points and 8 SMs * 128 lanes = 1024 lanes, literally one op per lane.

**Pi 3B:** No GPU; not applicable.
**Jetson GPU:** Totally doable via cuML, cuPy, or Numba CUDA.
**Code complexity:** Moderate to high. Numba CUDA JIT is easiest in Python but has startup cost; cuPy is numpy-drop-in but requires array shape discipline.
**Pros:**
- Scales to massive point counts (if we ever move to a 3D LiDAR with 100k pts/frame).
- Jetson GPU is sitting idle most of the time.
**Cons:**
- **Massive overkill for 500 points.** GPU kernel launch overhead (~20us) is already larger than the CPU compute time for the whole scan.
- Adds deployment complexity (CUDA dependencies, driver versions).
- Data transfer CPU->GPU->CPU costs more than the work itself.

**Recommendation:** **Don't use GPU for LiDAR processing.** 500 points is far below the GPU break-even point. Reserve the Jetson GPU for the LLM (Ollama) and any camera-side ML (face detection, semantic segmentation). LiDAR stays CPU.

---

## Summary Table

| Technique | Complexity | Pi 3B OK? | Jetson GPU? | Code effort | Recommendation |
|---|---|---|---|---|---|
| SoA / numpy vectorization | O(n) | Yes | No | Trivial | **Do it** |
| Spatial hashing 2D | O(n) build, O(k) query | Yes | Overkill | Simple | Conditional |
| Quadtree / KD-tree | O(n log n) | Yes (scipy) | Overkill | Trivial (scipy) | **Use scipy.spatial.cKDTree** |
| Statistical outlier removal | O(n log n) | Yes | Overkill | Simple | Conditional |
| 1D median filter (range) | O(n * w) | Yes | No | 1 line | **Do it first** |
| LOD (distance subsample) | O(n) | Yes | No | Trivial | Skip for now |
| DBSCAN clustering | O(n log n) | Yes (sklearn) | Overkill | 1 line sklearn | **Do it** |
| Frustum culling 2D | O(n) | Yes | No | 3 lines | Conditional |
| Temporal median filter | O(n * N) | Yes | No | ~30 lines | **Do it — biggest win** |
| Kalman tracker per cluster | O(k) | Yes | No | Moderate | Future work |
| GPU parallel processing | O(n/p) | No | Yes | Moderate | **Skip — overkill** |

---

## Recommended Implementation Order

For the current wandering demo on LD19, the highest-value changes in order:

1. **Temporal median filter (N=3 scans)** — 30 lines, kills transient noise, leverages the 10Hz repetition rate.
2. **1D spatial median filter on ranges per scan** — 1 line with `scipy.ndimage.median_filter` with `mode='wrap'` to handle the 360/0 seam.
3. **DBSCAN clustering on cartesian points** — 1 line with sklearn, gives us object-level perception instead of raw points.
4. **scipy.spatial.cKDTree** for any neighbor queries needed by downstream algorithms.
5. **Frustum-culled "emergency stop" check** — narrow 60-degree forward arc, tighter threshold — run alongside the full-360 wandering logic.
6. *(Later)* **Kalman tracker per DBSCAN cluster** for predictive avoidance of moving obstacles.

All of this fits comfortably on the Pi 3B. The Jetson GPU is not useful for LiDAR at this scale — save it for LLM inference and camera ML.

---

## Key Insight

**The LD19 at 500 points / 10 Hz is a tiny workload by game dev standards.** Modern games routinely process 10,000 to 1,000,000 particles per frame. Any technique that works in a 2015-era indie game will run comfortably on the Pi 3B for our 500 points. The interesting question is not "can we do this fast enough?" but "which technique gives the best signal quality and robot behavior?" — which points toward temporal filtering and clustering rather than raw parallelization.

---

## Sources

### Particle systems and optimization
- Unreal Engine particle optimization: https://dev.epicgames.com/documentation/en-us/unreal-engine/core-optimization-concepts-for-particle-systems
- Building a Million-Particle System (Game Developer): https://www.gamedeveloper.com/programming/building-a-million-particle-system
- Real-Time Particle Systems on the GPU (SIGGRAPH 2007): https://advances.realtimerendering.com/s2007/Drone-Real-Time_Particles_Systems_on_the_GPU_in_Dynamic_Environments(Siggraph07%20Course%20Notes).pdf

### Spatial hashing
- Spatial Hashing for Dynamic Object Management (peerdh.com): https://peerdh.com/blogs/programming-insights/implementing-spatial-hashing-for-dynamic-object-management-in-2d-game-engines
- Spatial hashing implementation (conkerjo): https://conkerjo.wordpress.com/2009/06/13/spatial-hashing-implementation-for-fast-2d-collisions/
- Spatial Hashing (GameDev.net): https://www.gamedev.net/articles/programming/general-and-gameplay-programming/spatial-hashing-r2697/

### Quadtree / spatial partitioning
- Quadtrees for 2D Games with Moving Elements (Medium): https://medium.com/@bpmw/quadtrees-for-2d-games-with-moving-elements-63360b08329f
- Spatial Partition (Game Programming Patterns): https://gameprogrammingpatterns.com/spatial-partition.html
- An interactive intro to quadtrees: https://growingswe.com/blog/quadtrees

### Point cloud denoising
- Point cloud denoising review (ScienceDirect): https://www.sciencedirect.com/science/article/abs/pii/S1524070322000170
- Awesome 3D Point Cloud Denoising: https://github.com/agarnung/awesome-3d-point-cloud-denoising
- Score-Based Point Cloud Denoising (ICCV 2021): https://openaccess.thecvf.com/content/ICCV2021/papers/Luo_Score-Based_Point_Cloud_Denoising_ICCV_2021_paper.pdf

### LOD
- Level of detail (Wikipedia): https://en.wikipedia.org/wiki/Level_of_detail_(computer_graphics)
- GPU-Accelerated LOD Generation for Point Clouds (Schutz 2023): https://onlinelibrary.wiley.com/doi/full/10.1111/cgf.14877
- LOD in Game Dev (Sloyd): https://www.sloyd.ai/blog/mastering-level-of-detail-lod-balancing-graphics-and-performance-in-game-development

### DBSCAN and clustering
- DBSCAN (Wikipedia): https://en.wikipedia.org/wiki/DBSCAN
- scikit-learn DBSCAN: https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
- An Improved DBSCAN Method for LiDAR Data Segmentation (MDPI): https://www.mdpi.com/1424-8220/19/1/172
- Grid-Based DBSCAN Clustering Accelerator for LiDAR: https://www.mdpi.com/2079-9292/13/17/3395
- Adaptive DBSCAN LiDAR Point Cloud Clustering (IEEE): https://ieeexplore.ieee.org/document/9814025/

### Frustum culling
- LearnOpenGL Frustum Culling: https://learnopengl.com/Guest-Articles/2021/Scene/Frustum-Culling
- A fast 2D frustum culling approach (IEEE): https://ieeexplore.ieee.org/document/5485876/
- 2D Frustum Culling Tutorial: https://lcmccauley.wordpress.com/2014/04/24/2d-frustum-culling-tutorial-p1/

### Temporal coherence
- Exploiting temporal and spatial coherence (Real-Time Rendering): https://www.realtimerendering.com/blog/exploiting-temporal-and-spatial-coherence/
- Temporal Coherence Methods in Real-Time Rendering (Scherzer 2012): https://onlinelibrary.wiley.com/doi/abs/10.1111/j.1467-8659.2012.03075.x
- Intel Open Image Denoise 3 temporal denoising: https://www.cgchannel.com/2026/01/open-image-denoise-3-will-support-temporal-denoising/

### Kalman filtering
- Kalman Filter Explained: https://kalmanfilter.net/
- Kalman filter (Wikipedia): https://en.wikipedia.org/wiki/Kalman_filter
- A Tutorial on Particle Filtering and Smoothing: https://www.stats.ox.ac.uk/~doucet/doucet_johansen_tutorialPF2011.pdf

### GPU particle systems
- Particle Simulation using CUDA (Simon Green, NVIDIA): https://developer.download.nvidia.com/assets/cuda/files/particles.pdf
- Efficient GPU Implementation of Particle Interactions: https://arxiv.org/html/2406.16091v1

### LiDAR filtering (Python)
- Lidar_filter (range + temporal median, Python): https://github.com/menglaili/Lidar_filter
- Open3D statistical outlier removal: https://www.open3d.org/docs/latest/tutorial/Advanced/pointcloud_outlier_removal.html
- PCL StatisticalOutlierRemoval: https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html
- An Efficient Adaptive Noise Removal Filter on Range Images for LiDAR (MDPI): https://www.mdpi.com/2079-9292/12/9/2150
- CNN-based Lidar Point Cloud De-Noising in Adverse Weather: https://arxiv.org/pdf/1912.03874
