# Waypoint Navigation System

## Overview

This document describes the waypoint management and pathfinding system built on top of AMCL localization.

---

## Waypoint Concept

A **waypoint** is a named location on the map with:
- **Name**: Human-readable identifier (e.g., "office1", "kitchen")
- **Position**: (x, y, z) in meters, relative to map origin
- **Orientation**: Quaternion or yaw angle (which way robot should face)
- **Tolerance**: How close is "close enough" (position and orientation)

---

## Waypoint File Format

Waypoints are stored in YAML format:

```yaml
metadata:
  created_by: add_office_waypoints.py
  map_yaml: first_map.yaml
  frame_id: map

waypoints:
- name: office1
  description: First office - left side of map
  position:
    x: -1.58
    y: -0.49
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
  yaw_degrees: 0
  tolerance:
    position: 0.3
    orientation: 0.2

routes:
  office_tour:
  - office1
  - office2
  - office3
```

### Quaternion to Yaw Conversion

```python
# Yaw (degrees) to quaternion
yaw_rad = math.radians(yaw_degrees)
qz = math.sin(yaw_rad / 2)
qw = math.cos(yaw_rad / 2)

# Quaternion to yaw (degrees)
yaw_rad = 2 * math.atan2(qz, qw)
yaw_degrees = math.degrees(yaw_rad)
```

---

## A* Pathfinding Algorithm

### Overview

A* finds the shortest path between two points while avoiding obstacles.

### Algorithm Steps

1. **Initialize**: Create open set with start node, closed set empty
2. **Loop**:
   - Pop node with lowest f = g + h from open set
   - If goal reached, reconstruct path
   - Add to closed set
   - For each neighbor:
     - Skip if in closed set or obstacle
     - Calculate tentative g score
     - If better than existing, update parent and add to open set
3. **Return**: Path or None if no path exists

### Cost Functions

- **g(n)**: Actual cost from start to node n
- **h(n)**: Heuristic estimate from n to goal (Euclidean distance)
- **f(n) = g(n) + h(n)**: Total estimated cost

### Grid Movement Costs

| Direction | Cost |
|-----------|------|
| Cardinal (N, S, E, W) | 1.0 |
| Diagonal (NE, NW, SE, SW) | 1.414 (√2) |

---

## Obstacle Detection

### Map Values

In ROS occupancy grid maps (PGM format):
- **0**: Occupied (black)
- **205**: Unknown (gray)
- **254**: Free (white)

### Inflation Radius

We check a radius around each cell:
```python
def is_free(gx, gy, inflation=2):
    for dy in range(-inflation, inflation + 1):
        for dx in range(-inflation, inflation + 1):
            if grid[ny][nx] < 100:  # Obstacle threshold
                return False
    return True
```

This keeps paths 2+ cells away from obstacles.

---

## Path Simplification

Raw A* paths have many points (one per grid cell). We simplify using **Ramer-Douglas-Peucker**:

1. Draw line from start to end
2. Find point with maximum perpendicular distance
3. If distance > tolerance, recursively simplify both halves
4. Otherwise, replace segment with straight line

**Result**: 82 points → 2-4 points (much easier to follow)

---

## Coordinate Transformations

### World to Grid
```python
gx = int((world_x - origin_x) / resolution)
gy = int(height - ((world_y - origin_y) / resolution))
```

### Grid to World
```python
world_x = origin_x + (gx * resolution)
world_y = origin_y + ((height - gy) * resolution)
```

Note: Image Y-axis is inverted (0 at top).

---

## Routes

A **route** is a named sequence of waypoints:

```yaml
routes:
  office_tour: [office1, office2, office3]
  patrol: [entrance, hallway, office1, hallway, entrance]
```

Routes can:
- Be executed in sequence
- Loop continuously
- Skip waypoints that can't be reached

---

## Usage Examples

### List Waypoints
```bash
python3 navigate_to_office.py --list
```

### Plan Path (Dry Run)
```bash
python3 navigate_to_office.py office2 --dry-run
```

### Plan Full Route
```bash
python3 navigate_to_office.py --route office_tour --dry-run
```

### Direct Pathfinding
```bash
python3 simple_pathfinder.py \
    --map ~/ros2_ws/maps/first_map.yaml \
    --waypoints ~/ros2_ws/maps/first_map_offices.yaml \
    --from office1 --to office3
```

---

## Test Results

### Office Tour Route

```
office1 → office2 → office3
Total: 6.92 meters, 3 segments

Segment 1 (office1): 1.84m, 3 points
Segment 2 (office2): 2.75m, 4 points
Segment 3 (office3): 2.33m, 4 points
```

### Direct Path (office1 → office3)
```
4.05 meters, 2 points (straight line - no obstacles)
```

---

## Integration with Localization

The `localization_with_waypoints.py` script combines:
1. **AMCL pose subscription** - current robot position
2. **Waypoint distances** - how far to each waypoint
3. **Bearing calculation** - direction to each waypoint
4. **Path planning** - route to nearest waypoint

Output example:
```
Robot Position: (0.50, 0.30) @ 45.0°
Uncertainty: ±0.05m (x), ±0.05m (y)
Quality: ★★★ EXCELLENT

WAYPOINTS:
Name            Distance    Bearing   Direction
office2            0.25m       +20°          ↗
office1            2.10m      -150°          ↙
office3            2.05m       +30°          ↗
```

---

## Future Improvements

1. **Dynamic replanning**: Replan when obstacles detected
2. **Smooth paths**: Use Bezier curves or splines
3. **Orientation planning**: Plan robot heading at each waypoint
4. **Time-optimal paths**: Account for acceleration/deceleration
5. **Multi-floor support**: Elevator waypoints

---

**Last Updated:** 2025-12-22
