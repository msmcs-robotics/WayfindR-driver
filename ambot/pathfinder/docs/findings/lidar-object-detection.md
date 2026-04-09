# 2D LiDAR Object Detection & Classification — Research Findings

**Target platform**: LD19 LiDAR (~350-500 points/scan @ 10 Hz) on differential-drive indoor robot
**Use case**: Indoor wandering + people-following. Identify walls, chairs, people, doorways.
**Constraint**: Runs on RPi (906 MB RAM, Cortex-A53) or Jetson Orin Nano. CPU-friendly algorithms preferred.
**Date**: 2026-04-09

---

## Summary Table

| Technique | Identifies | Cost | Python lib | Pros | Cons |
|-----------|-----------|------|-----------|------|------|
| Distance-jump segmentation | Discrete objects | O(n) | numpy | Trivially fast, works on raw polar | Misses soft edges, needs adaptive threshold |
| Adaptive Breakpoint Detector (ABD) | Object boundaries | O(n) | numpy (custom) | Range-adaptive, robust at long distances | Single tuning constant (λ) needs calibration |
| DBSCAN | Arbitrary-shape clusters | O(n log n) | scikit-learn | No k needed, handles noise, finds arbitrary shapes | eps sensitive to range (far points sparser) |
| K-means | Clusters (fixed k) | O(n·k·i) | scikit-learn | Fast, simple | Needs k a priori, convex-shape bias — bad for 2D laser |
| RANSAC line fitting | Walls / straight edges | O(n·iter) | scikit-learn, sklearn.linear_model.RANSACRegressor | Robust to outliers, great for walls | Needs iterative extraction per line |
| Convex hull + OBB | Object extent / bbox | O(n log n) | scipy.spatial.ConvexHull | Gives width/length/orientation | Only shape summary, not classification |
| Polar-grid / sector bins | Coarse object presence | O(n) | numpy | Already used in your safety zones | Lose angular fidelity, not discrete objects |
| Leg detection (random forest) | People/legs | O(n) per cluster | leg_tracker (ROS), sklearn | Proven for people-following | Trained on older scanners, needs retraining |
| Ray-tracing / visibility | Occlusions, dynamic obstacles | O(grid) | numpy | Finds dynamic vs static | Requires local map |
| Occupancy grid | Local free/occupied map | O(n) per scan | numpy / nav_msgs | Good for planning, integrates time | Not object-level, needs pose |
| Hampel / median sliding window | Noise outliers | O(n·w) | scipy.signal | Kills spurious returns | Pre-processing only |

---

## 1. Distance-Jump Segmentation (Euclidean breakpoints)

**What**: Walk the scan in angular order. If `|r[i] - r[i-1]| > Dthresh`, start a new segment. Result: each segment = one candidate object.

**Pros for us**:
- Operates directly on the raw LD19 polar stream (no coordinate conversion needed)
- O(n), sub-millisecond on 500 points
- Naturally handles occluded gaps

**Cons**:
- Constant threshold fails: a 5 cm gap at 1 m is one object, at 5 m is noise
- Oblique walls (angled surface) produce large radial jumps — false splits

**Reference**: Jump edges are one of the three canonical boundary forms in laser scans (jump, crease, smooth). For 2D indoor LiDAR, jump edges are dominant.

**Code sketch**:
```python
def segment_by_jump(ranges, angles, d_thresh=0.15):
    segs = [[]]
    for i, (r, a) in enumerate(zip(ranges, angles)):
        if i > 0 and abs(r - ranges[i-1]) > d_thresh:
            segs.append([])
        segs[-1].append((r, a))
    return [s for s in segs if len(s) >= 3]
```

---

## 2. Adaptive Breakpoint Detector (ABD) — Dietmayer

**What**: Range-adaptive version of (1). The allowable distance between consecutive points grows with range and angular resolution:

```
Dmax = r[i-1] * sin(Δφ) / sin(λ - Δφ) + 3·σ
```

where `Δφ` = angular step (~0.72° for LD19), `λ` ≈ 10° (auxiliary angle, single tunable), `σ` = noise std.

**Pros**:
- Handles the "far points are sparser" problem that trips fixed-threshold segmentation
- Standard in 2D-lidar robotics literature since 2001
- Single intuitive parameter (λ)

**Cons**:
- Still fooled by surfaces nearly parallel to the laser ray
- Requires knowing LD19 angular resolution (LD19 ≈ 0.72°/step)

**Use for**: Primary object segmentation step before clustering/classification. Good replacement for fixed-threshold jump detection.

---

## 3. DBSCAN (Density-Based Spatial Clustering)

**What**: Group points where each has at least `min_samples` neighbors within radius `eps`. Everything else is noise.

**Parameters**:
- `eps` — neighborhood radius (meters). Typical indoor: 0.08-0.20 m
- `min_samples` — density floor. LD19 is sparse, so 3-5 is reasonable

**Cost**: O(n log n) with KD-tree; for 500 points ≈ 2-5 ms on RPi.

**Accuracy**: Comparative studies rank DBSCAN ahead of K-means for LiDAR because it doesn't assume convex clusters and tolerates noise.

**Improved variants**: Adaptive-eps DBSCAN computes eps from k-distance plot automatically, helping with the sparse-at-range problem.

**Pros for us**:
- Ready in scikit-learn (already a dep)
- No `k` needed
- Noise points labeled `-1` → free outlier rejection

**Cons**:
- Uniform `eps` misclassifies distant objects (LD19 points get farther apart with range). Workaround: convert to Cartesian first, or use polar-adapted eps.
- ~5× slower than pure jump segmentation

**Code**:
```python
from sklearn.cluster import DBSCAN
import numpy as np

xy = np.column_stack([ranges*np.cos(angles), ranges*np.sin(angles)])
labels = DBSCAN(eps=0.15, min_samples=3).fit_predict(xy)
# labels[i] == -1 → noise; else cluster ID
```

---

## 4. K-Means on Laser Scans

**What**: Partition points into `k` clusters minimizing within-cluster variance.

**Verdict for our use case: NOT recommended.**
- Must know `k` upfront — impossible with unknown scenes
- Convex bias: a long wall becomes several fake clusters
- Studies show it outperforms DBSCAN only on pre-segmented canopy-type data, not indoor scans

**When it might help**: After distance-jump segmentation has split the scan, K-means with `k=2` can sub-split a cluster that's "obviously two people close together" — but DBSCAN + careful eps handles this better.

---

## 5. RANSAC Line Fitting — Walls

**What**: Randomly sample 2 points, fit a line, count inliers within ε. Repeat; keep best. Remove inliers. Repeat for next line. (Sequential / iterative RANSAC)

**Identifies**: Walls, long flat surfaces, door edges (straight features generally).

**Cost**: O(iter × n). With 100 iterations × 500 points ≈ 5-15 ms/line. For indoor, 3-6 lines typical → ~30-90 ms. OK at 10 Hz if budgeted.

**Parameters**:
- `residual_threshold` — inlier distance, 0.02-0.05 m for indoor walls
- `min_samples=2` for line
- `max_trials=100-500`

**Python**: `sklearn.linear_model.RANSACRegressor` (1D fit), or custom 2D implementation (pyRANSAC-3D has 2D variant).

**Pros**:
- Ignores people/chairs cleanly (they become outliers)
- Gives wall equations useful for localization, corner detection, doorway detection

**Cons**:
- Iterative removal is slow if there are many lines
- Doesn't give endpoints natively — must post-process inliers for start/end

**Use for**: Wall extraction layer running at ~2-5 Hz (subsampled). Feed wall lines to corner/doorway detector.

**Code**:
```python
from sklearn.linear_model import RANSACRegressor
ransac = RANSACRegressor(residual_threshold=0.03, min_samples=2, max_trials=100)
ransac.fit(xy[:, 0:1], xy[:, 1])
inlier_mask = ransac.inlier_mask_
```

---

## 6. Door Detection

**What indoor doors look like in 2D LiDAR**:
- A gap (2 parallel line ends) of ~0.7-1.0 m in an otherwise continuous wall
- Two 90° corners flanking the gap
- Often jambs = short perpendicular line segments

**Heuristic pipeline**:
1. RANSAC line extraction (walls)
2. Detect endpoints of adjacent collinear walls
3. Compute gap between them; if 0.6-1.2 m → candidate door
4. Check perpendicular jambs (optional)
5. Classify as `open_door` if passable in front gap, `closed_door` if a short line spans the gap

**References**:
- Laser-based door localization work (see sources) treats door detection as line-fitting + jamb heuristic + (for opening) circular arc fit of the swing
- Semantic feature extraction for localization: extract corners, match to map — same features double as doorway candidates

**Limitation**: LD19 at 500 pts covers doors with ~15-20 pts at 2 m range. Marginal but workable.

---

## 7. Convex Hull + Oriented Bounding Box

**What**: After clustering, compute `scipy.spatial.ConvexHull(cluster_xy)`. From the hull points, find the minimum-area rotated rectangle → gives `(cx, cy, w, h, yaw)`.

**Cost**: O(n log n) per cluster; milliseconds for typical indoor clusters (30-80 pts each).

**Pros**:
- Gives object footprint dimensions usable for classification rules:
  - `w < 0.2 m, h < 0.2 m` → leg / pole / table leg
  - `0.25 < w < 0.45, aspect ≈ 1` → person
  - `w > 1 m, thin` → wall segment
  - `large + convex` → chair/clutter
- Works for tracking (centroid + size-invariant)

**Cons**:
- Hull is sensitive to 1-2 outlier points — run after DBSCAN noise rejection
- Only a shape summary; doesn't discriminate between classes with similar footprint

**Library**: `scipy.spatial.ConvexHull` (already available), `cv2.minAreaRect` for OBB.

```python
from scipy.spatial import ConvexHull
import cv2
hull = ConvexHull(cluster_xy)
rect = cv2.minAreaRect(cluster_xy[hull.vertices].astype(np.float32))
(cx, cy), (w, h), yaw = rect
```

---

## 8. People / Leg Detection

**Primary reference**: Leigh, Pineau et al. ICRA 2015 "Person Tracking and Following with 2D Laser Scanners" — the basis for ROS `leg_tracker` and `leg_detector`.

**Pipeline**:
1. Cluster scan points (Euclidean distance, small threshold ~0.13 m)
2. Compute ~14 features per cluster: width, circularity, linearity, number of points, radius, std, mean curvature, boundary length, boundary regularity, etc.
3. OpenCV Random Forest (trained offline) scores each cluster P(leg)
4. Pair leg candidates within ~0.5 m → person
5. Kalman/particle filter for tracking and velocity estimation

**Cost**: Cluster + features: ~2 ms. RF inference: ~1 ms. Total well under 10 ms per scan.

**Accuracy**: ~85-95% precision in published results. False positives on table/chair legs are the main issue — visual confirmation (your existing camera face tracker!) helps a lot.

**ROS availability**:
- `leg_tracker` (angusleigh) — ROS1
- `ros2_leg_detector` (mowito) — ROS2 port
- `cob_leg_detection` — alternative

**For our non-ROS stack**:
- Extract the feature set (Python, ~50 lines)
- Train a random forest with `sklearn.ensemble.RandomForestClassifier` on hand-collected labeled scans
- Combine with face detection: if camera sees a face AND a leg-shaped cluster in the same bearing, confidence → high

**Caveat from the literature**: "tuned for 7.5-15 Hz laser scanners; downsample if faster". LD19 is 10 Hz — matches perfectly.

**Fusion with existing face tracker**: The `demos_common/sensors.py` face tracker gives bearing but no depth. LiDAR leg detection gives depth but weaker identity. Fusing them at the same bearing is the strongest indoor person detector you can build without ML.

---

## 9. Scan Segmentation — Jump/Edge Detection (detailed)

**Three canonical boundary types in laser data**:
1. **Jump edges** — discontinuity in range (what we use most in 2D indoor)
2. **Crease edges** — first-derivative break (corner between two walls)
3. **Smooth edges** — curvature threshold (rounded objects)

**Crease / corner detection**: After RANSAC line extraction, intersections of adjacent lines with small gap = corners. Corners are the anchors for door detection and topological mapping.

**Dynamic threshold**: For obstacle safety, define the distance threshold as a function of returned range rather than a fixed value — the ABD approach handles this.

---

## 10. Polar-Grid / Sector-Based Methods

**Current usage (yours)**: `NaturalWanderBehavior` bins ~467 points into 36 angular buckets (10°), picks top 10 peaks ≥30° apart. This is a polar-grid maximum filter — cheap and robust for choosing a travel direction, but not for identifying objects.

**Literature comparison**:
- Polar-grid Gaussian-Mixture Model: models background per angular bin, flags foreground as anything deviating → works well for static-LiDAR people tracking (e.g. roadside). Could be adapted for your static-base human presence detection.
- PolarNet (CVPR 2020): deep-learning segmentation on polar grids. Overkill for LD19/RPi.
- Grid-based DBSCAN accelerator: skip the KD-tree by bucketing into polar cells first — ~5-10× speedup.

**Recommendation**: Keep sector binning for **wander direction selection** (it's the right tool) and layer ABD + DBSCAN + convex hull on top for **object identification**.

---

## 11. Local Occupancy Grid

**What**: A small ego-centric 2D grid (e.g. 6 m × 6 m at 5 cm resolution = 120×120 cells). Each cell holds `log-odds(occupied)`. Each scan ray updates cells along its path (free) and the endpoint (occupied).

**Cost**: ~10 k cells updated per scan; few ms in numpy vectorized form.

**Uses**:
- Temporal smoothing — a single noisy return doesn't create a phantom
- Local planning — feed costmap to dynamic window approach
- Dynamic vs static discrimination — compare scans over time per cell

**Library**: No need for ROS; you can implement in ~100 lines of numpy. Alternative: wrap `nav2_costmap_2d` semantics without the ROS runtime.

**Bresenham ray tracing**:
```python
def raytrace(grid, x0, y0, x1, y1, free_logodds=-0.4, occ_logodds=0.85):
    # Bresenham from sensor origin to endpoint
    ...  # mark intermediates free, endpoint occupied
```

---

## 12. Ray-Tracing / Occlusion / Shadow Detection

**Core insight** (visibility principle): if two points on the same bearing are observed at different ranges over time, the nearer one must be the dynamic object. If an expected background point suddenly returns at a shorter range, the new point is a dynamic intrusion.

**M-detector approach**: Use ray tracing against the last-known static map; any point whose ray passes through previously-occupied space indicates motion.

**For people-following**:
- Maintain a short (~1-2 s) rolling static occupancy grid
- Current scan points that occlude previously-free cells → moving targets
- Current scan rays that pass through previously-occupied cells → the previously-detected object has moved
- Combined with leg detection, this gives robust moving-person candidates

**Shadow/void detection**: Check for gaps in the expected scan (void region in a BEV) — could be an occluded hidden object or a doorway. Useful to prompt "look closer" behavior.

**Cost**: Grid ray tracing O(grid diagonal) per ray. Scan-to-scan differencing O(n). ~5-10 ms on RPi.

---

## 13. Sliding-Window Outlier Rejection (pre-processing)

**Hampel filter** (recommended): sliding window of width `w`, replaces any point differing from window median by > `k·σ` with the median. For polar LiDAR, run on the range array directly:

```python
from scipy.ndimage import median_filter
def hampel(r, w=5, k=3.0):
    med = median_filter(r, size=w)
    mad = median_filter(np.abs(r - med), size=w) * 1.4826
    mask = np.abs(r - med) > k * mad
    out = r.copy()
    out[mask] = med[mask]
    return out
```

**Use**: Apply before any segmentation step. Kills:
- Single-sample sensor flickers
- Specular/glass double-bounce returns
- Dust motes

**Related filters**:
- Dynamic Statistical Outlier Removal (DSOR) — density-aware variant
- Dynamic Radius Outlier Removal (DROR) — range-adaptive (similar idea to ABD)
- PCL `StatisticalOutlierRemoval` — available via `pclpy` but heavyweight

**Cost**: O(n·w), w=5, 500 pts ≈ 0.5 ms.

---

## Recommended Pipeline for AMBOT

Ordered, cheap → expensive. Early stages cover most uses; later stages only run when needed.

```
Raw LD19 scan (ranges[n], angles[n], n≈467)
        │
        ▼
[1] Hampel sliding-window filter (1 ms)
        │
        ▼
[2] Polar→Cartesian (0.5 ms)
        │
        ▼
[3] Adaptive Breakpoint Detector → segments (1 ms)
        │
        ▼
[4] Per-segment convex hull + OBB → {centroid, w, h, yaw, n_pts} (2 ms)
        │
        ├─ Rule-based classification:
        │     • w,h < 0.15  →  pole/leg
        │     • 0.15-0.45, ~square → candidate person
        │     • long+thin → wall/furniture edge
        │     • large+irregular → clutter
        │
        ▼
[5a] RANSAC line fit on "wall-like" segments → wall list (10-30 ms, subsample to 2 Hz)
        │
        ▼
[5b] For candidate person segments: run leg feature extractor + RF classifier (2 ms)
        │   ↕ fuse with face tracker bearing → person confidence
        │
        ▼
[6] Update local occupancy grid (5 ms) → dynamic obstacle mask via ray diff
        │
        ▼
[7] Keep existing polar-sector maxima for wander direction selection
```

**Budget estimate on RPi**: ~15-25 ms per scan in the hot path; SRAM footprint trivial. Easy headroom at 10 Hz.

**Budget on Jetson**: Negligible (<5 ms). Room to add LLM-based classification of cluster descriptors as a supplementary layer if needed.

---

## Key Libraries to Install

Already on RPi venv (assumed): `numpy`, `scipy`, `scikit-learn`, `opencv-python` (apt).

Additionally helpful:
- `shapely` — polygon ops on convex hulls if you want room polygons
- No PCL / Open3D needed for 2D work

---

## Open Questions / Follow-ups

1. Calibrate ABD's `λ` constant against LD19 at known ranges (0.3 m, 1 m, 3 m) indoors
2. Collect a few labeled scans (walk, stand, chair-only, doorway) for leg-classifier training
3. Decide whether to use DBSCAN or ABD-only for primary segmentation: benchmark both on 5 minutes of recorded scans
4. The face+leg fusion is the most promising person-follower signal — prototype it as a first experiment
5. Occupancy grid needs odometry — MPU6050 gives heading only. Differential drive kinematics + IMU yaw fusion → adequate local pose for 1-2 s rolling grid

---

## Sources

- [An Improved DBSCAN Method for LiDAR Data Segmentation with Automatic Eps Estimation (MDPI Sensors)](https://www.mdpi.com/1424-8220/19/1/172)
- [DBSCAN — scikit-learn documentation](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html)
- [LiDAR Clustering and Shape Extraction for Automotive Applications (Chalmers thesis)](https://publications.lib.chalmers.se/records/fulltext/253622/253622.pdf)
- [Clustering LiDAR Data with K-means and DBSCAN (SciTePress)](https://www.scitepress.org/Papers/2023/116670/116670.pdf)
- [Adaptive Clustering for Point Cloud (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10857180/)
- [Person Tracking and Following with 2D Laser Scanners — Leigh/Pineau ICRA 2015](https://www.cs.mcgill.ca/~jpineau/files/leigh-icra15.pdf)
- [angusleigh/leg_tracker (GitHub)](https://github.com/angusleigh/leg_tracker)
- [mowito/ros2_leg_detector (GitHub)](https://github.com/mowito/ros2_leg_detector)
- [cob_leg_detection — ROS Wiki](http://wiki.ros.org/cob_leg_detection)
- [Deep Person Detection in 2D Range Data (arXiv 1804.02463)](https://arxiv.org/pdf/1804.02463)
- [Tracking People in a Mobile Robot From 2D LIDAR Scans Using FCNs (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6332292/)
- [RANSAC Line Fitting for Simulated LiDAR in Gazebo (GitHub)](https://github.com/Rotvie/ransac-lidar-ros)
- [pyRANSAC-3D (GitHub)](https://github.com/leomariga/pyRANSAC-3D)
- [scikit-learn RANSACRegressor](https://scikit-learn.org/stable/auto_examples/linear_model/plot_ransac.html)
- [The Ultimate Guide to the RANSAC Algorithm](https://www.thinkautonomous.ai/blog/ransac-algorithm/)
- [Laser-Based Door Localization for Autonomous Mobile Service Robots (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10256011/)
- [Autonomous Navigation System of Indoor Mobile Robots Using 2D Lidar (MDPI)](https://www.mdpi.com/2227-7390/11/6/1455)
- [LiDAR 2D wall line detection experiment of indoor mobile robot](https://www.sciopen.com/article/10.16791/j.cnki.sjg.2024.05.011)
- [Localization System Through 2D LiDAR-based Semantic Features (IEEE)](https://ieeexplore.ieee.org/document/9826250/)
- [Estimation of 2D Bounding Box Orientation with Convex-Hull Points (IEEE IV 2020)](https://ieeexplore.ieee.org/document/9304788/)
- [Convex hull based bounding box fitting for 3D LiDAR (GitHub CH-MOA-ROS)](https://github.com/HMX2013/CH-MOA-ROS)
- [AutoLidarPerception/object_builders_lib (GitHub)](https://github.com/AutoLidarPerception/object_builders_lib)
- [LiDAR data processing for object detection (Blickfeld blog)](https://www.blickfeld.com/blog/lidar-data-processing-for-object-detection/)
- [Adaptive Obstacle Detection for Mobile Robots Using Downward-Looking 2D LiDAR (MDPI Sensors)](https://www.mdpi.com/1424-8220/18/6/1749)
- [Development of Adaptive Line Tracking Breakpoint Detection (IJACSA)](https://thesai.org/Downloads/Volume13No7/Paper_32-Development_of_Adaptive_Line_Tracking_Breakpoint_Detection.pdf)
- [kostaskonkk/datmo — Detection and Tracking of Moving Objects (GitHub)](https://github.com/kostaskonkk/datmo)
- [Adaptive Polar-Grid Gaussian-Mixture Model for Foreground Segmentation (MDPI Remote Sensing)](https://www.mdpi.com/2072-4292/14/11/2522)
- [PolarNet: An Improved Grid Representation (CVPR 2020)](https://arxiv.org/abs/2003.14032)
- [Grid-Based DBSCAN Clustering Accelerator for LiDAR (MDPI Electronics)](https://www.mdpi.com/2079-9292/13/17/3395)
- [Systematic Review of Clustering and Multi-Target Tracking for LiDAR (MDPI Sensors)](https://www.mdpi.com/1424-8220/23/13/6119)
- [Mapping and Localization — Nav2 docs](https://docs.nav2.org/setup_guides/sensors/mapping_localization.html)
- [Mapless Navigation Based on 2D LIDAR in Complex Unknown Environments (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC7602247/)
- [Object Semantic Grid Mapping with 2D LiDAR and RGB-D Camera (MDPI)](https://www.mdpi.com/2076-3417/10/17/5782)
- [Outlier Detection with Hampel Filter (Towards Data Science)](https://towardsdatascience.com/outlier-detection-with-hampel-filter-85ddf523c73d/)
- [An Efficient Adaptive Noise Removal Filter for LiDAR Point Clouds (MDPI Electronics)](https://www.mdpi.com/2079-9292/12/9/2150)
- [Adaptive Group of Density Outlier Removal Filter — Snow Particle Removal (MDPI)](https://www.mdpi.com/2079-9292/11/19/2993)
- [PCL StatisticalOutlierRemoval tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html)
- [LiDAR-based Real-Time Object Detection and Tracking in Dynamic Environments (arXiv 2407.04115)](https://arxiv.org/html/2407.04115v1)
- [Using 3D Shadows to Detect Object Hiding Attacks (arXiv 2204.13973)](https://arxiv.org/pdf/2204.13973)
- [LV-DOT: LiDAR-visual Dynamic Obstacle Detection and Tracking (arXiv 2502.20607)](https://arxiv.org/html/2502.20607v1)
- [Occlusion detection of traffic signs by voxel-based ray tracing (ScienceDirect)](https://www.sciencedirect.com/science/article/pii/S1569843222002059)
