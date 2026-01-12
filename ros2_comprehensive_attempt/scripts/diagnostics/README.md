# WayfindR Diagnostics and Monitoring Tools

Comprehensive diagnostic and monitoring system for the WayfindR navigation system.

## Overview

This diagnostic suite provides real-time monitoring, health checks, and performance profiling for autonomous navigation:

- **System Diagnostics**: Monitor TF tree, topic health, localization quality, and system resources
- **Monitoring Dashboard**: Real-time terminal-based status display
- **TF Tree Visualizer**: Analyze and visualize transform tree structure
- **Topic Checker**: Monitor topic frequencies and message rates
- **Map Quality Analyzer**: Analyze occupancy grid maps for issues
- **Localization Quality Checker**: Assess AMCL localization performance
- **Performance Profiler**: Measure end-to-end latency and resource usage

## Installation

### Prerequisites

```bash
# ROS2 packages (should already be installed)
sudo apt install ros-humble-diagnostic-msgs

# Python packages
pip3 install rich psutil numpy pillow pyyaml matplotlib
```

### Make Scripts Executable

```bash
cd scripts/diagnostics/
chmod +x *.py
```

## Tools

### 1. System Diagnostics Node

Continuously monitors all critical system components and publishes diagnostics.

**Features:**
- TF transform health monitoring
- Topic frequency checking
- AMCL localization quality assessment
- System resource monitoring (CPU, memory, network)
- Publishes to `/diagnostics` topic

**Usage:**
```bash
# Run the diagnostics node
python3 system_diagnostics.py

# Or with ROS2
ros2 run <package> system_diagnostics.py
```

**What it monitors:**
- `/scan` - Laser scan frequency (expected: 10 Hz)
- `/cmd_vel` - Velocity commands (expected: 20 Hz)
- `/odom` - Odometry (expected: 20 Hz)
- `/amcl_pose` - Localization (expected: 2 Hz)
- TF transforms: map→odom, odom→base_footprint, base_footprint→base_link, base_link→laser
- CPU and memory usage
- Network bandwidth

### 2. Monitoring Dashboard

Real-time terminal-based dashboard with live status updates.

**Features:**
- Color-coded health indicators
- Real-time metrics display
- Alert system for warnings and errors
- Robot position and velocity tracking
- Obstacle detection display

**Usage:**
```bash
# Start the dashboard (requires 'rich' library)
python3 monitoring_dashboard.py

# The dashboard will auto-refresh every 0.5 seconds
```

**Dashboard Sections:**
- Header: System uptime and timestamp
- Diagnostics: Component health status
- Robot Status: Position, velocity, obstacles
- Alerts: Recent warnings and errors

### 3. TF Tree Visualizer

Visualizes and analyzes the TF transformation tree.

**Features:**
- Hierarchical tree display
- Transform age and staleness detection
- Disconnected frame detection
- Cycle detection
- Health check with issue reporting

**Usage:**
```bash
# Display TF tree
python3 tf_tree_visualizer.py

# Export to file
python3 tf_tree_visualizer.py --export tf_tree.txt

# Health check (returns exit code 0 if healthy)
python3 tf_tree_visualizer.py --check

# Watch continuously
python3 tf_tree_visualizer.py --watch

# Specify root frame
python3 tf_tree_visualizer.py --root odom
```

**Example Output:**
```
======================================================================
TF TREE STRUCTURE
======================================================================

map [ROOT]
└── odom [OK] 0.05s
    └── base_footprint [OK] 0.02s
        └── base_link [OK] 0.01s
            ├── laser [OK] 0.01s
            └── imu_link [OK] 0.01s
```

### 4. Topic Checker

Monitors ROS2 topics for frequency and content.

**Features:**
- Frequency (Hz) measurement
- Message echo functionality
- Multi-topic monitoring
- Dynamic topic type detection

**Usage:**
```bash
# Check single topic frequency
python3 topic_checker.py /scan --hz

# Echo topic messages
python3 topic_checker.py /cmd_vel --echo

# Monitor multiple topics
python3 topic_checker.py /scan /cmd_vel /odom --hz

# Monitor all critical topics
python3 topic_checker.py --all
```

**Example Output:**
```
================================================================================
TOPIC STATISTICS (uptime: 30.5s)
================================================================================
Topic                          Count      Hz         Status
--------------------------------------------------------------------------------
/scan                          305        10.03      OK
/cmd_vel                       610        20.01      OK
/odom                          609        19.97      OK
/amcl_pose                     61         2.00       OK
================================================================================
```

### 5. Map Quality Analyzer

Analyzes occupancy grid maps for quality and navigation issues.

**Features:**
- Map coverage analysis (free/occupied/unknown)
- Connected region detection
- Narrow passage identification
- Dead end detection
- Visualization with matplotlib

**Usage:**
```bash
# Analyze map
python3 map_quality_analyzer.py --map ../maps/office.yaml

# Analyze and visualize
python3 map_quality_analyzer.py --map ../maps/office.yaml --visualize

# Save visualization
python3 map_quality_analyzer.py --map ../maps/office.yaml --visualize --output map_analysis.png
```

**What it checks:**
- **Coverage**: Percentage of free, occupied, and unknown space
- **Connectivity**: Disconnected regions that robot cannot navigate between
- **Narrow Passages**: Bottlenecks that may be difficult to navigate
- **Dead Ends**: Areas where robot could get stuck

**Example Output:**
```
======================================================================
MAP QUALITY ANALYSIS
======================================================================

1. MAP COVERAGE:
   Total cells: 384,000
   Free space: 230,400 (60.0%)
   Occupied: 76,800 (20.0%)
   Unknown: 76,800 (20.0%)

2. CONNECTIVITY:
   Connected regions: 1
   ✓ Map is fully connected

3. NARROW PASSAGES:
   Found 45 narrow passages
   ℹ Some narrow passages exist (robot clearance: check robot radius)

4. DEAD ENDS:
   Found 12 dead end cells
   ℹ Dead ends detected (may be normal for alcoves/corners)

======================================================================
OVERALL ASSESSMENT:
✓ Map quality is GOOD
======================================================================
```

### 6. Localization Quality Checker

Monitors and analyzes AMCL localization quality.

**Features:**
- Position uncertainty tracking
- Particle filter convergence analysis
- Pose stability measurement
- Real-time quality assessment
- Recommendations for improvement

**Usage:**
```bash
# Quick snapshot (5 seconds)
python3 localization_quality.py

# Monitor for specific duration
python3 localization_quality.py --monitor 30

# Export report
python3 localization_quality.py --monitor 20 --export quality_report.txt
```

**Quality Levels:**
- **EXCELLENT**: < 0.1m uncertainty
- **GOOD**: < 0.3m uncertainty
- **FAIR**: < 0.5m uncertainty
- **POOR**: < 1.0m uncertainty
- **VERY_POOR**: > 1.0m uncertainty

**Example Output:**
```
======================================================================
LOCALIZATION QUALITY REPORT
======================================================================

Quality: GOOD (4/5)

Metrics:
  Position Uncertainty:    0.2156 m
  Orientation Uncertainty: 3.45°
  Particle Count:          500
  Particle Spread:         0.1823 m
  Pose Stability:          0.0234 m
  Convergence Rate:        12.3%

Statistics:
  Runtime:             30.2s
  Total Drift:         0.456m
  Max Uncertainty:     0.3421m
  Min Uncertainty:     0.1234m

Recommendations:
  ✓ Localization quality is good
======================================================================
```

### 7. Performance Profiler

Measures end-to-end system performance and latency.

**Features:**
- Sensor-to-control latency measurement
- Message rate monitoring
- CPU and memory profiling
- Navigation success rate tracking
- Percentile analysis (P50, P95, P99)

**Usage:**
```bash
# Profile for 30 seconds (default)
python3 performance_profiler.py

# Profile for specific duration
python3 performance_profiler.py --duration 60

# Export report
python3 performance_profiler.py --duration 60 --export profile_report.txt

# Profile individual nodes
python3 performance_profiler.py --profile-nodes
```

**Metrics:**
- **Latency**: Time from sensor data to control command
- **Message Rates**: Actual vs expected frequencies
- **System Resources**: CPU and memory usage
- **ROS Nodes**: Active node count

**Example Output:**
```
======================================================================
PERFORMANCE PROFILE REPORT
======================================================================

Runtime: 60.5s

1. LATENCY (Sensor to Control):
   Samples:     1205
   Min:         12.34ms
   Mean:        45.67ms
   Max:         98.23ms
   P50 (median): 43.21ms
   P95:         67.89ms
   P99:         82.45ms
   ✓ Good latency

2. MESSAGE RATES:
   ✓ /scan                : 10.02 Hz (expected: 10.0)
   ✓ /cmd_vel             : 19.98 Hz (expected: 20.0)
   ✓ /odom                : 20.01 Hz (expected: 20.0)
   ✓ /amcl_pose           :  2.00 Hz (expected: 2.0)

3. SYSTEM RESOURCES:
   CPU Usage:    Mean: 45.3%  Max: 67.8%
   Memory Usage: Mean: 52.1%  Max: 58.9%
   ✓ CPU usage is reasonable

4. ROS NODES:
   Active nodes: 15
     - /amcl
     - /bt_navigator
     - /controller_server
     - /planner_server
     ... and 11 more

======================================================================
```

## Troubleshooting Workflows

### Workflow 1: Robot is Not Localizing

1. **Check AMCL pose topic:**
   ```bash
   python3 topic_checker.py /amcl_pose --hz
   ```

2. **Assess localization quality:**
   ```bash
   python3 localization_quality.py --monitor 10
   ```

3. **Check TF tree:**
   ```bash
   python3 tf_tree_visualizer.py --check
   ```

4. **Actions based on results:**
   - High uncertainty: Set initial pose in RViz
   - Missing TF: Check if AMCL is running
   - Stale transforms: Restart localization

### Workflow 2: Robot is Not Moving

1. **Check cmd_vel topic:**
   ```bash
   python3 topic_checker.py /cmd_vel --echo
   ```

2. **Check overall system health:**
   ```bash
   python3 system_diagnostics.py
   ```

3. **Actions:**
   - No cmd_vel: Check Nav2 stack
   - High latency: Check performance profile
   - TF issues: Check transform tree

### Workflow 3: Navigation is Slow/Jerky

1. **Profile system performance:**
   ```bash
   python3 performance_profiler.py --duration 60
   ```

2. **Check message rates:**
   ```bash
   python3 topic_checker.py --all
   ```

3. **Actions:**
   - High latency: Reduce planner frequency or increase CPU
   - Low message rates: Check sensor connections
   - High CPU: Optimize parameters or reduce sensor resolution

### Workflow 4: Robot Gets Lost

1. **Check localization quality:**
   ```bash
   python3 localization_quality.py --monitor 30
   ```

2. **Analyze map quality:**
   ```bash
   python3 map_quality_analyzer.py --map ../maps/office.yaml --visualize
   ```

3. **Actions:**
   - Poor localization: Re-localize with 2D Pose Estimate
   - Map issues: Re-create map with SLAM
   - Particle spread high: Check for map-environment mismatch

### Workflow 5: Complete System Check

Run the full diagnostic dashboard:

```bash
# Terminal 1: Start diagnostics node
python3 system_diagnostics.py

# Terminal 2: Start dashboard
python3 monitoring_dashboard.py

# Terminal 3: Run tests as needed
python3 performance_profiler.py --duration 60
```

## Integration with ROS2

### Using with Launch Files

```python
# In your launch file
from launch_ros.actions import Node

diagnostics = Node(
    package='your_package',
    executable='system_diagnostics.py',
    name='system_diagnostics',
    output='screen'
)
```

### Subscribing to Diagnostics

```python
from diagnostic_msgs.msg import DiagnosticArray

# In your node
self.create_subscription(
    DiagnosticArray,
    '/diagnostics',
    self.diagnostics_callback,
    10
)
```

## Best Practices

1. **Always run diagnostics during development** to catch issues early

2. **Use the monitoring dashboard** during testing to see real-time status

3. **Profile performance regularly** to identify bottlenecks

4. **Check localization quality** before long navigation tasks

5. **Analyze maps** before deploying to new environments

6. **Monitor TF tree** when adding new sensors or frames

7. **Export reports** for documentation and debugging

## Tips

- Run diagnostics in a separate terminal for continuous monitoring
- Use `--export` flags to save reports for later analysis
- Check diagnostics after parameter changes to verify improvements
- Set up automated health checks in production environments
- Use quality thresholds to trigger alerts or failsafes

## Known Issues and Limitations

1. **Rich library**: Dashboard requires `rich` library for terminal UI
2. **Topic types**: Topic checker uses dynamic imports (may fail for custom messages)
3. **Performance overhead**: Running all tools simultaneously may impact performance
4. **Permissions**: Some system metrics may require elevated permissions

## Contributing

When adding new diagnostic tools:
1. Follow the existing naming convention
2. Include `--help` documentation
3. Add usage examples to this README
4. Test with real robot data
5. Handle missing data gracefully

## License

Part of the WayfindR navigation system.
