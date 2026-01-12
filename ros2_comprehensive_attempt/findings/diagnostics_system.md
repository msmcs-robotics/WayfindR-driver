# WayfindR Diagnostics System Documentation

**Date:** 2026-01-11
**Status:** Complete
**Version:** 1.0

## Executive Summary

A comprehensive diagnostic and monitoring system has been implemented for the WayfindR navigation system. This system provides real-time health monitoring, performance profiling, and debugging tools to ensure reliable autonomous navigation.

### Key Features

- **7 Diagnostic Tools**: Complete suite for monitoring all aspects of navigation
- **Real-time Monitoring**: Live dashboard with color-coded status indicators
- **Performance Profiling**: End-to-end latency measurement and resource tracking
- **Quality Analysis**: Map quality and localization assessment tools
- **Automated Health Checks**: Continuous monitoring with alert generation
- **Troubleshooting Workflows**: Step-by-step guides for common issues

## System Architecture

### Components Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    WayfindR Navigation System                │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Sensors    │  │  Navigation  │  │   Control    │      │
│  │  /scan       │→ │    Stack     │→ │  /cmd_vel    │      │
│  │  /odom       │  │   (Nav2)     │  │              │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│         ↓                 ↓                  ↓               │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│              Diagnostics & Monitoring Layer                  │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────────┐  ┌──────────────────┐                │
│  │ System Diagnostics│  │  Monitoring      │                │
│  │   - TF Health    │  │   Dashboard      │                │
│  │   - Topics       │  │  (Real-time UI)  │                │
│  │   - Localization │  │                  │                │
│  │   - Resources    │  │                  │                │
│  └──────────────────┘  └──────────────────┘                │
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ TF Tree      │  │ Topic        │  │ Performance  │      │
│  │ Visualizer   │  │ Checker      │  │ Profiler     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
│  ┌──────────────┐  ┌──────────────┐                        │
│  │ Map Quality  │  │ Localization │                        │
│  │ Analyzer     │  │ Quality      │                        │
│  └──────────────┘  └──────────────┘                        │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

## Tool Details

### 1. System Diagnostics Node (`system_diagnostics.py`)

**Purpose:** Continuous real-time monitoring of all critical system components.

**Monitored Components:**

| Component | Metrics | Update Rate |
|-----------|---------|-------------|
| TF Transforms | Age, staleness, errors | 1 Hz |
| Topics | Frequency, message count | Real-time |
| Localization | Uncertainty, covariance | Real-time |
| System Resources | CPU, memory, network | 1 Hz |

**Health Status Levels:**
- **OK**: Component functioning normally
- **WARN**: Minor issues detected
- **ERROR**: Critical issues detected
- **STALE**: Data too old
- **UNKNOWN**: No data available

**Output:**
- Publishes to `/diagnostics` topic (ROS2 standard)
- Console logging every 5 seconds
- Diagnostic status for each component

**Implementation Details:**
```python
# Critical transforms monitored
('map', 'odom')              # Localization
('odom', 'base_footprint')   # Odometry
('base_footprint', 'base_link')  # Robot base
('base_link', 'laser')       # Sensors

# Topic frequency expectations
'/scan': 10.0 Hz
'/cmd_vel': 20.0 Hz
'/odom': 20.0 Hz
'/amcl_pose': 2.0 Hz
```

**Usage:**
```bash
python3 system_diagnostics.py
```

### 2. Monitoring Dashboard (`monitoring_dashboard.py`)

**Purpose:** Real-time visual monitoring with rich terminal UI.

**Features:**
- Color-coded health indicators (✓ ⚠ ✗)
- Live updating display (2 Hz refresh)
- Alert history tracking
- Robot state visualization
- Obstacle detection display

**Display Sections:**

1. **Header**: System uptime, current time
2. **Diagnostics Table**: All component health status
3. **Robot Status**: Position, velocity, nearest obstacle
4. **Alerts Panel**: Recent warnings and errors

**Requirements:**
```bash
pip3 install rich
```

**Usage:**
```bash
python3 monitoring_dashboard.py
```

**Sample Output:**
```
╭──────────────────────────────────────────────────────────────╮
│ WayfindR Navigation System | Uptime: 00:05:32 | 2026-01-11... │
╰──────────────────────────────────────────────────────────────╯

╭─ System Diagnostics ─────────────────────────────────────────╮
│ Component              Status      Details                    │
│ /scan                  ✓ OK        actual_hz=10.1            │
│ /cmd_vel               ✓ OK        actual_hz=20.0            │
│ map->odom              ✓ OK        age=0.05s                 │
│ Localization (AMCL)    ✓ OK        pos_unc=0.123m            │
│ System Resources       ✓ OK        cpu=45.2%                 │
╰──────────────────────────────────────────────────────────────╯
```

### 3. TF Tree Visualizer (`tf_tree_visualizer.py`)

**Purpose:** Analyze and visualize TF transformation tree structure.

**Capabilities:**
- Hierarchical tree display
- Transform age tracking
- Disconnected frame detection
- Cycle detection (should never happen)
- Health check with exit codes

**Health Checks:**
1. ✓ Critical transforms exist and are fresh
2. ✓ All frames are connected
3. ✓ No cycles in tree
4. ⚠ Detect stale transforms (> 2s old)
5. ✗ Detect missing transforms

**Usage:**
```bash
# Display tree
python3 tf_tree_visualizer.py

# Health check (returns 0 if healthy)
python3 tf_tree_visualizer.py --check
if [ $? -eq 0 ]; then echo "TF tree healthy"; fi

# Export to file
python3 tf_tree_visualizer.py --export tf_tree.txt

# Watch continuously
python3 tf_tree_visualizer.py --watch --interval 2
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
            └── camera_link [OK] 0.01s

======================================================================
TF TREE HEALTH CHECK
======================================================================

Checking critical transforms:
  ✓ map -> odom: OK (0.05s)
  ✓ odom -> base_footprint: OK (0.02s)
  ✓ base_footprint -> base_link: OK (0.01s)
  ✓ base_link -> laser: OK (0.01s)

Checking for disconnected frames:
  ✓ All frames connected

Checking for cycles:
  ✓ No cycles detected

======================================================================
RESULT: TF tree is healthy
======================================================================
```

### 4. Topic Checker (`topic_checker.py`)

**Purpose:** Monitor ROS2 topic frequencies and message content.

**Features:**
- Dynamic topic type detection
- Frequency measurement with statistics
- Message echo functionality
- Multi-topic monitoring
- Comparison with expected rates

**Usage:**
```bash
# Check frequency
python3 topic_checker.py /scan --hz

# Echo messages
python3 topic_checker.py /cmd_vel --echo

# Monitor multiple topics
python3 topic_checker.py /scan /cmd_vel /odom --hz

# Monitor all critical topics
python3 topic_checker.py --all
```

**Output:**
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

### 5. Map Quality Analyzer (`map_quality_analyzer.py`)

**Purpose:** Analyze occupancy grid maps for quality and navigation issues.

**Analysis Types:**

1. **Coverage Analysis**
   - Free space percentage
   - Occupied space percentage
   - Unknown space percentage

2. **Connectivity Analysis**
   - Connected region detection (flood fill algorithm)
   - Disconnected region identification
   - Region size calculation

3. **Navigation Hazards**
   - Narrow passages (configurable threshold)
   - Dead ends detection
   - Bottleneck identification

4. **Visualization**
   - Original map display
   - Connected regions (color-coded)
   - Narrow passage heatmap
   - Dead end locations

**Usage:**
```bash
# Basic analysis
python3 map_quality_analyzer.py --map ../maps/office.yaml

# With visualization
python3 map_quality_analyzer.py --map ../maps/office.yaml --visualize

# Save visualization
python3 map_quality_analyzer.py --map ../maps/office.yaml \
    --visualize --output map_analysis.png
```

**Requirements:**
```bash
pip3 install numpy pillow matplotlib
```

**Sample Analysis:**
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

**Interpretation Guide:**

| Metric | Good | Warning | Critical |
|--------|------|---------|----------|
| Unknown % | < 10% | 10-20% | > 20% |
| Connected Regions | 1 | 2-3 | > 3 |
| Narrow Passages | < 50 | 50-100 | > 100 |
| Free Space % | > 50% | 30-50% | < 30% |

### 6. Localization Quality Checker (`localization_quality.py`)

**Purpose:** Monitor and analyze AMCL localization quality in real-time.

**Metrics Tracked:**

1. **Position Uncertainty** (standard deviation in meters)
   - Excellent: < 0.1m
   - Good: < 0.3m
   - Fair: < 0.5m
   - Poor: < 1.0m
   - Very Poor: > 1.0m

2. **Orientation Uncertainty** (standard deviation in radians)

3. **Particle Cloud Analysis**
   - Particle count
   - Particle spread (spatial distribution)

4. **Convergence Rate**
   - How quickly uncertainty is decreasing

5. **Pose Stability**
   - Variance in recent pose estimates

**Usage:**
```bash
# Quick 5-second snapshot
python3 localization_quality.py

# Monitor for 30 seconds
python3 localization_quality.py --monitor 30

# Export report
python3 localization_quality.py --monitor 20 --export report.txt
```

**Sample Output:**
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

**Troubleshooting Recommendations:**

- **High Uncertainty (> 0.5m)**: Use "2D Pose Estimate" in RViz to re-localize
- **High Particle Spread (> 1.0m)**: Robot may be lost; check for map-environment mismatch
- **Unstable Pose (> 0.1m variance)**: Check sensor data quality or map alignment
- **Low Convergence**: May need to move robot to different location with more features

### 7. Performance Profiler (`performance_profiler.py`)

**Purpose:** Measure end-to-end system performance and identify bottlenecks.

**Performance Metrics:**

1. **Latency Measurement**
   - Sensor data arrival → Control command output
   - Min, mean, max, P50, P95, P99 percentiles
   - Sample count and distribution

2. **Message Rates**
   - Actual vs expected frequencies
   - Topic health indicators

3. **System Resources**
   - CPU usage (mean and max)
   - Memory usage (mean and max)
   - Network bandwidth

4. **ROS Node Information**
   - Active node count
   - Node listing

**Usage:**
```bash
# Profile for 30 seconds
python3 performance_profiler.py

# Profile for 60 seconds
python3 performance_profiler.py --duration 60

# Export report
python3 performance_profiler.py --duration 60 --export profile.txt
```

**Sample Output:**
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
     - /lifecycle_manager
     ... and 10 more

======================================================================
```

**Performance Benchmarks:**

| Metric | Excellent | Good | Acceptable | Poor |
|--------|-----------|------|------------|------|
| Mean Latency | < 50ms | 50-100ms | 100-200ms | > 200ms |
| P95 Latency | < 80ms | 80-150ms | 150-300ms | > 300ms |
| CPU Usage | < 50% | 50-70% | 70-90% | > 90% |
| Memory Usage | < 50% | 50-75% | 75-90% | > 90% |

## Troubleshooting Workflows

### Workflow 1: Robot Not Localizing

**Symptoms:** Robot doesn't know where it is, high uncertainty, particle cloud spread out.

**Diagnostic Steps:**

1. Check if AMCL is publishing:
   ```bash
   python3 topic_checker.py /amcl_pose --hz
   ```

2. Check localization quality:
   ```bash
   python3 localization_quality.py --monitor 10
   ```

3. Verify TF tree:
   ```bash
   python3 tf_tree_visualizer.py --check
   ```

4. Check particle cloud:
   ```bash
   ros2 topic echo /particlecloud --once
   ```

**Resolution Actions:**

- **High uncertainty (> 0.5m)**:
  - Use "2D Pose Estimate" in RViz to set initial pose
  - Drive robot to area with more distinguishable features

- **No AMCL topic**:
  - Check if AMCL node is running: `ros2 node list | grep amcl`
  - Restart localization launch file

- **Missing map→odom transform**:
  - AMCL not publishing transform
  - Check AMCL configuration and map file

- **Particle spread > 1.0m**:
  - Robot may be lost
  - Check map matches environment
  - Re-localize with known pose

### Workflow 2: Robot Not Moving

**Symptoms:** Nav2 appears active but robot doesn't respond to commands.

**Diagnostic Steps:**

1. Check cmd_vel topic:
   ```bash
   python3 topic_checker.py /cmd_vel --echo
   ```

2. Run full diagnostics:
   ```bash
   python3 system_diagnostics.py
   ```

3. Check TF tree health:
   ```bash
   python3 tf_tree_visualizer.py --check
   ```

4. Verify odometry:
   ```bash
   python3 topic_checker.py /odom --echo
   ```

**Resolution Actions:**

- **No cmd_vel messages**:
  - Nav2 controller not running
  - Check for errors: `ros2 topic echo /diagnostics`
  - Verify navigation goal was sent

- **cmd_vel messages but no motion**:
  - Motor controller not receiving commands
  - Check cmd_vel_bridge or motor driver
  - Verify robot is not in safe mode

- **TF errors**:
  - Missing transforms prevent planning
  - Check robot_state_publisher
  - Verify URDF is loaded correctly

- **Odometry not updating**:
  - Sensor issue or driver problem
  - Check physical connections
  - Restart sensor driver

### Workflow 3: Navigation Slow or Jerky

**Symptoms:** Robot moves but motion is erratic, slow, or stuttering.

**Diagnostic Steps:**

1. Profile system performance:
   ```bash
   python3 performance_profiler.py --duration 60
   ```

2. Check all topic rates:
   ```bash
   python3 topic_checker.py --all
   ```

3. Monitor dashboard during navigation:
   ```bash
   python3 monitoring_dashboard.py
   ```

**Resolution Actions:**

- **High latency (> 100ms mean)**:
  - System overloaded
  - Reduce planner frequency
  - Increase controller lookahead
  - Lower sensor resolution

- **Low scan frequency (< 8 Hz)**:
  - Sensor communication issue
  - Check USB bandwidth
  - Verify sensor driver settings

- **High CPU usage (> 80%)**:
  - Reduce costmap update frequency
  - Simplify local costmap
  - Use faster planner algorithm
  - Check for memory leaks

- **Localization degradation**:
  - Check with localization quality checker
  - May need better odometry tuning
  - Increase AMCL particle count

### Workflow 4: Robot Gets Lost

**Symptoms:** Initially localized but loses position over time.

**Diagnostic Steps:**

1. Monitor localization quality:
   ```bash
   python3 localization_quality.py --monitor 60
   ```

2. Analyze map quality:
   ```bash
   python3 map_quality_analyzer.py --map maps/office.yaml --visualize
   ```

3. Check for systematic drift:
   ```bash
   # Monitor pose stability over time
   python3 localization_quality.py --monitor 120 --export drift_report.txt
   ```

**Resolution Actions:**

- **Gradual uncertainty increase**:
  - Odometry drift accumulating
  - Tune AMCL odometry model parameters
  - Check wheel encoders for errors

- **Sudden loss of localization**:
  - Robot in featureless area
  - Add landmarks or features
  - Improve map with more scan data

- **Map mismatch**:
  - Environment changed since mapping
  - Re-create map with current layout
  - Check for moved furniture/obstacles

- **Narrow passages**:
  - Map quality shows many bottlenecks
  - Increase inflation radius carefully
  - Use slower speeds in tight areas

### Workflow 5: Complete System Health Check

**Purpose:** Comprehensive diagnostic before deployment or after issues.

**Procedure:**

```bash
# Terminal 1: Start continuous diagnostics
python3 system_diagnostics.py

# Terminal 2: Start monitoring dashboard
python3 monitoring_dashboard.py

# Terminal 3: Run sequential checks

# 1. TF tree health
python3 tf_tree_visualizer.py --check
# Should exit with code 0

# 2. Topic health
python3 topic_checker.py --all
# All topics should show "OK" status

# 3. Map quality (if using localization)
python3 map_quality_analyzer.py --map maps/office.yaml
# Should report "Map quality is GOOD"

# 4. Localization quality (if robot is localized)
python3 localization_quality.py --monitor 10
# Should be at least "GOOD" quality

# 5. Performance baseline
python3 performance_profiler.py --duration 30
# Mean latency should be < 100ms

# 6. Export results
python3 performance_profiler.py --duration 60 --export health_check.txt
```

**Health Check Criteria:**

✓ **Pass Criteria:**
- All TF transforms present and fresh (< 1s age)
- All critical topics publishing at expected rates
- Localization quality "GOOD" or better (< 0.3m uncertainty)
- Mean latency < 100ms
- CPU usage < 70%
- No ERROR level diagnostics

⚠ **Warning Criteria:**
- Some TF transforms slightly stale (1-2s age)
- Topic rates 50-80% of expected
- Localization quality "FAIR" (0.3-0.5m uncertainty)
- Mean latency 100-150ms
- CPU usage 70-85%
- WARN level diagnostics present

✗ **Fail Criteria:**
- Missing critical TF transforms
- Topic rates < 50% of expected
- Localization quality "POOR" or worse (> 0.5m uncertainty)
- Mean latency > 150ms
- CPU usage > 85%
- ERROR level diagnostics present

## Integration Guide

### Adding Diagnostics to Existing System

1. **Copy diagnostic scripts:**
   ```bash
   cd ros2_comprehensive_attempt/scripts/
   cp -r diagnostics /path/to/your/package/scripts/
   ```

2. **Install dependencies:**
   ```bash
   pip3 install rich psutil numpy pillow matplotlib pyyaml
   ```

3. **Add to launch files:**
   ```python
   # In your main launch file
   from launch_ros.actions import Node

   diagnostics_node = Node(
       package='your_package',
       executable='system_diagnostics.py',
       name='system_diagnostics',
       output='screen'
   )
   ```

4. **Subscribe to diagnostics in your nodes:**
   ```python
   from diagnostic_msgs.msg import DiagnosticArray

   self.create_subscription(
       DiagnosticArray,
       '/diagnostics',
       self.diagnostics_callback,
       10
   )
   ```

### Automated Health Monitoring

Create a monitoring script that runs periodically:

```bash
#!/bin/bash
# health_monitor.sh

LOG_DIR="/var/log/wayfinder"
mkdir -p $LOG_DIR

# Run health checks
python3 tf_tree_visualizer.py --check > $LOG_DIR/tf_health.log 2>&1
TF_STATUS=$?

python3 localization_quality.py --monitor 10 --export $LOG_DIR/loc_quality.txt

python3 performance_profiler.py --duration 30 --export $LOG_DIR/performance.txt

# Check results
if [ $TF_STATUS -eq 0 ]; then
    echo "Health check PASSED" | tee $LOG_DIR/health_status.log
else
    echo "Health check FAILED" | tee $LOG_DIR/health_status.log
    # Send alert (email, slack, etc.)
fi
```

### Production Deployment

**Recommended Setup:**

1. Run system_diagnostics.py as a persistent service
2. Set up log aggregation for diagnostic messages
3. Configure alerts for ERROR level diagnostics
4. Schedule periodic performance profiling
5. Monitor localization quality during navigation
6. Archive diagnostic reports for analysis

**Systemd Service Example:**
```ini
[Unit]
Description=WayfindR Diagnostics
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/home/robot/wayfinder
ExecStart=/usr/bin/python3 scripts/diagnostics/system_diagnostics.py
Restart=always

[Install]
WantedBy=multi-user.target
```

## Best Practices

### During Development

1. **Always run system_diagnostics.py** during testing sessions
2. **Use monitoring_dashboard.py** for visual feedback
3. **Profile performance** after parameter changes
4. **Check map quality** before first deployment
5. **Verify TF tree** when adding sensors

### During Testing

1. **Run complete health check** before each test session
2. **Monitor localization quality** during long runs
3. **Profile performance** under different loads
4. **Export reports** for comparison over time
5. **Document issues** found by diagnostics

### In Production

1. **Continuous diagnostics** node running at all times
2. **Automated health checks** on schedule
3. **Alert system** for critical issues
4. **Log aggregation** for diagnostics data
5. **Regular performance reviews** from exported reports

## Performance Optimization

### Based on Diagnostic Results

**High Latency (> 100ms):**
```yaml
# In nav2_params.yaml
controller_server:
  controller_frequency: 15.0  # Reduce from 20.0

planner_server:
  expected_planner_frequency: 1.0  # Reduce from 2.0

local_costmap:
  update_frequency: 3.0  # Reduce from 5.0
```

**High CPU Usage (> 70%):**
```yaml
# Reduce sensor resolution
rplidar:
  scan_frequency: 8.0  # Reduce from 10.0

# Simplify costmaps
local_costmap:
  width: 3.0  # Reduce from 5.0
  height: 3.0
  resolution: 0.05  # Increase from 0.025
```

**Poor Localization:**
```yaml
amcl:
  max_particles: 1000  # Increase from 500
  min_particles: 200   # Increase from 100
  update_min_d: 0.1    # Decrease for more frequent updates
  update_min_a: 0.2
```

## Known Issues and Limitations

### Current Limitations

1. **Rich library dependency**: Monitoring dashboard requires rich library
2. **Topic type detection**: May fail for custom message types
3. **Performance overhead**: Running all tools simultaneously impacts system
4. **No historical data**: Currently no database for long-term trending
5. **Manual analysis**: No automated issue diagnosis yet

### Future Enhancements

1. **Historical data logging**: Store metrics in time-series database
2. **Automated issue detection**: ML-based anomaly detection
3. **Web-based dashboard**: Remote monitoring via web interface
4. **Alert notifications**: Email/Slack/SMS for critical issues
5. **Predictive diagnostics**: Predict failures before they occur
6. **Custom metric support**: User-defined diagnostic checks

## Conclusion

The WayfindR diagnostics system provides comprehensive monitoring and debugging capabilities for autonomous navigation. By following the troubleshooting workflows and best practices outlined in this document, teams can quickly identify and resolve issues, ensuring reliable navigation performance.

### Key Takeaways

✓ **7 specialized tools** for all aspects of navigation monitoring
✓ **Real-time dashboard** with live status updates
✓ **Performance profiling** with latency percentiles
✓ **Quality analysis** for maps and localization
✓ **Troubleshooting workflows** for common issues
✓ **Production-ready** with automated health checks

### Quick Reference

```bash
# Daily monitoring
python3 monitoring_dashboard.py

# Weekly health check
python3 tf_tree_visualizer.py --check
python3 localization_quality.py --monitor 30
python3 performance_profiler.py --duration 60 --export weekly_profile.txt

# When issues occur
python3 system_diagnostics.py  # See what's wrong
python3 topic_checker.py --all  # Check data flow
python3 localization_quality.py  # Check positioning

# Before deployment
python3 map_quality_analyzer.py --map maps/environment.yaml --visualize
bash health_monitor.sh
```

---

**Documentation Version:** 1.0
**Last Updated:** 2026-01-11
**Maintained By:** WayfindR Development Team
