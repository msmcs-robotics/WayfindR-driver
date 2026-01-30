# Robot Wandering Algorithms (No SLAM Required)

> Research findings for pathfinder component
> Date: 2026-01-29

## Overview

This document covers reactive navigation algorithms that use LiDAR data for obstacle avoidance and room exploration **without requiring SLAM or mapping**. These algorithms are ideal for demo robots that need to wander around safely.

---

## Implemented Algorithms

The following are implemented in `ambot/pathfinder/behaviors.py`:

### 1. MaxClearanceBehavior (Follow the Gap)

**How it works:** Find the direction with the longest distance reading and move toward it.

```python
from ambot.pathfinder import MaxClearanceBehavior

behavior = MaxClearanceBehavior(
    forward_speed=0.3,
    turn_speed=0.5,
    alignment_threshold=15.0,  # degrees
    min_clearance=0.5  # meters
)
```

**Pros:**
- Computationally lightweight
- Works purely reactively without any map
- Excellent for corridors and open spaces

**Cons:**
- Can oscillate in symmetric environments
- May U-turn unexpectedly in complex areas

### 2. WallFollowerBehavior

**How it works:** Keep a wall on one side (left or right) at a set distance.

```python
from ambot.pathfinder import WallFollowerBehavior, WallSide

behavior = WallFollowerBehavior(
    wall_side=WallSide.LEFT,
    target_distance=0.5,  # meters
    forward_speed=0.25,
    correction_gain=0.5
)
```

**Pros:**
- Very predictable behavior
- Guarantees finding exit in simply-connected mazes
- Low computational requirements

**Cons:**
- Can get stuck in loops with isolated obstacles
- Not efficient for general exploration

### 3. RandomWanderBehavior

**How it works:** Move forward when clear, random turn when obstacle detected, occasional random turns for exploration.

```python
from ambot.pathfinder import RandomWanderBehavior

behavior = RandomWanderBehavior(
    forward_speed=0.3,
    turn_speed=0.4,
    turn_probability=0.05  # 5% chance per step
)
```

**Pros:**
- Extremely simple
- Will eventually explore most reachable areas
- Robust - no complex state to corrupt

**Cons:**
- Inefficient - revisits same areas frequently
- May get stuck in corners for extended periods

### 4. AvoidAndGoBehavior

**How it works:** Priority-based reactive behavior - stop if obstacle too close, slow down if getting close, otherwise go forward.

```python
from ambot.pathfinder import AvoidAndGoBehavior

behavior = AvoidAndGoBehavior(
    forward_speed=0.3,
    turn_speed=0.5,
    stop_threshold=0.3,   # meters
    slow_threshold=0.8    # meters
)
```

**Pros:**
- Very simple and predictable
- Easy to debug
- Safe default behavior

**Cons:**
- Limited exploration capability
- Can get stuck in corners

---

## Algorithm Comparison

| Algorithm | Complexity | Exploration | Stuck Risk | Best Use |
|-----------|------------|-------------|------------|----------|
| MaxClearance | Low | High | Low | Open spaces, demo |
| WallFollower | Very Low | Low | Medium | Maze, perimeter |
| RandomWander | Very Low | Medium | High | Simple coverage |
| AvoidAndGo | Very Low | Low | Medium | Safety-first |

---

## Other Algorithms (Not Implemented)

### Subsumption Architecture
Layered behaviors where higher priority behaviors override lower ones. Good for combining multiple behaviors.

### Vector Field Histogram (VFH)
Creates polar histogram of obstacles and steers toward valleys. More complex but handles sensor noise well.

### Braitenberg Vehicles
Direct sensor-to-motor connections. Very simple but limited behavior repertoire.

### Potential Field
Virtual repulsive forces from obstacles. Smooth trajectories but can get stuck in local minima.

---

## Recommended for Demo Robot

For the Ambot demo, we recommend **MaxClearanceBehavior** with **AvoidAndGoBehavior** as a safety fallback:

```python
from ambot.pathfinder import (
    MaxClearanceBehavior,
    AvoidAndGoBehavior,
    BehaviorSelector
)

selector = BehaviorSelector()
selector.register("wander", MaxClearanceBehavior())
selector.register("avoid", AvoidAndGoBehavior())

# Use wander normally, switch to avoid if stuck
selector.select("wander")
```

---

## References

- F1TENTH Follow the Gap: https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleB/lecture05.html
- Wall Following Robot: https://github.com/omkarchittar/Wall_Following_Robot
- Subsumption Architecture: https://en.wikipedia.org/wiki/Subsumption_architecture
- VFH: https://en.wikipedia.org/wiki/Vector_Field_Histogram
- Braitenberg Vehicles: https://en.wikipedia.org/wiki/Braitenberg_vehicle
