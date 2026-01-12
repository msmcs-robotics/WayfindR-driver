# WayfindR Diagnostics System - Complete Index

**Version:** 1.0.0
**Date:** 2026-01-11
**Status:** Production Ready

## Quick Access

### Start Here
- **New Users**: [QUICKSTART.md](scripts/diagnostics/QUICKSTART.md)
- **Complete Guide**: [diagnostics_system.md](findings/diagnostics_system.md)
- **Tool Documentation**: [README.md](scripts/diagnostics/README.md)
- **Summary**: [DIAGNOSTICS_TOOLS_SUMMARY.md](findings/DIAGNOSTICS_TOOLS_SUMMARY.md)

### Quick Commands

```bash
# Install dependencies
pip3 install rich psutil numpy pillow matplotlib pyyaml

# Start monitoring
cd scripts/diagnostics/
python3 monitoring_dashboard.py

# Run full diagnostics
./run_full_diagnostics.sh
```

## Complete Tool Reference

### 1. Real-Time Monitoring Tools

| Tool | Purpose | Usage | Output |
|------|---------|-------|--------|
| **system_diagnostics.py** | Continuous health monitoring | `python3 system_diagnostics.py` | Console logs + `/diagnostics` topic |
| **monitoring_dashboard.py** | Visual monitoring dashboard | `python3 monitoring_dashboard.py` | Live terminal UI |

### 2. Diagnostic Tools

| Tool | Purpose | Usage | Output |
|------|---------|-------|--------|
| **tf_tree_visualizer.py** | TF tree analysis | `python3 tf_tree_visualizer.py --check` | Tree structure + health report |
| **topic_checker.py** | Topic frequency monitoring | `python3 topic_checker.py --all` | Frequency statistics |
| **localization_quality.py** | AMCL quality assessment | `python3 localization_quality.py --monitor 30` | Quality report |
| **map_quality_analyzer.py** | Map quality analysis | `python3 map_quality_analyzer.py --map ../maps/office.yaml` | Quality report + visualization |
| **performance_profiler.py** | Performance profiling | `python3 performance_profiler.py --duration 60` | Latency statistics |

### 3. Automation Tools

| Tool | Purpose | Usage | Output |
|------|---------|-------|--------|
| **run_full_diagnostics.sh** | Complete health check | `./run_full_diagnostics.sh [output_dir]` | Comprehensive report |

## Documentation Map

### Getting Started
1. [QUICKSTART.md](scripts/diagnostics/QUICKSTART.md) - 5-minute quick start
2. [README.md](scripts/diagnostics/README.md) - Complete tool documentation
3. [diagnostics_system.md](findings/diagnostics_system.md) - System architecture

### Troubleshooting
- **Robot not localizing**: See [README.md - Workflow 1](scripts/diagnostics/README.md#workflow-1-robot-not-localizing)
- **Robot not moving**: See [README.md - Workflow 2](scripts/diagnostics/README.md#workflow-2-robot-not-moving)
- **Navigation slow**: See [README.md - Workflow 3](scripts/diagnostics/README.md#workflow-3-navigation-slowjerky)
- **Robot gets lost**: See [README.md - Workflow 4](scripts/diagnostics/README.md#workflow-4-robot-gets-lost)
- **Complete health check**: See [README.md - Workflow 5](scripts/diagnostics/README.md#workflow-5-complete-system-health-check)

### Reference
- [DIAGNOSTICS_TOOLS_SUMMARY.md](findings/DIAGNOSTICS_TOOLS_SUMMARY.md) - Complete feature summary
- [diagnostics.launch.py](launch/diagnostics.launch.py) - ROS2 launch file

## Common Use Cases

### Use Case 1: Daily Development

```bash
# Terminal 1: Start diagnostics
python3 system_diagnostics.py

# Terminal 2: Visual monitoring
python3 monitoring_dashboard.py

# Terminal 3: Work as normal
# Dashboard shows live status
```

**Documents**: QUICKSTART.md

### Use Case 2: Troubleshooting Navigation Issues

```bash
# Check TF tree
python3 tf_tree_visualizer.py --check

# Check localization
python3 localization_quality.py --monitor 10

# Profile performance
python3 performance_profiler.py --duration 30

# Check topics
python3 topic_checker.py --all
```

**Documents**: README.md (Troubleshooting Workflows)

### Use Case 3: Pre-Deployment Health Check

```bash
# Run complete diagnostic suite
./run_full_diagnostics.sh ~/diagnostics/$(date +%Y%m%d_%H%M%S)

# Review reports in output directory
# Should show "Overall Result: EXCELLENT" or "GOOD"
```

**Documents**: DIAGNOSTICS_TOOLS_SUMMARY.md

### Use Case 4: Performance Optimization

```bash
# Baseline measurement
python3 performance_profiler.py --duration 60 --export baseline.txt

# Make parameter changes
# ...

# Compare results
python3 performance_profiler.py --duration 60 --export optimized.txt

# Compare baseline.txt vs optimized.txt
```

**Documents**: diagnostics_system.md (Performance Optimization section)

### Use Case 5: Map Quality Assessment

```bash
# Analyze new map
python3 map_quality_analyzer.py --map ../maps/new_environment.yaml --visualize

# Check for issues:
# - Disconnected regions
# - Narrow passages
# - Poor coverage
```

**Documents**: README.md (Map Quality Analyzer section)

## File Locations

### Scripts Directory
```
scripts/diagnostics/
├── system_diagnostics.py          # Real-time monitoring node
├── monitoring_dashboard.py        # Visual dashboard
├── tf_tree_visualizer.py         # TF tree analysis
├── topic_checker.py              # Topic monitoring
├── map_quality_analyzer.py       # Map quality analysis
├── localization_quality.py       # Localization assessment
├── performance_profiler.py       # Performance profiling
├── run_full_diagnostics.sh       # Full diagnostic suite
├── README.md                     # Tool documentation
├── QUICKSTART.md                 # Quick start guide
└── __init__.py                   # Python package
```

### Launch Files
```
launch/
└── diagnostics.launch.py         # ROS2 launch file
```

### Documentation
```
findings/
├── diagnostics_system.md         # Complete system documentation
└── DIAGNOSTICS_TOOLS_SUMMARY.md  # Feature summary
```

### Root
```
ros2_comprehensive_attempt/
└── DIAGNOSTICS_INDEX.md          # This file
```

## Dependencies

### Required (ROS2)
- rclpy
- tf2_ros
- sensor_msgs
- geometry_msgs
- nav_msgs
- nav2_msgs
- diagnostic_msgs

### Optional (Enhanced Features)
- rich (terminal dashboard)
- psutil (system monitoring)
- numpy (statistics)
- pillow (map loading)
- matplotlib (visualization)
- pyyaml (YAML parsing)

### Installation
```bash
# Install all optional dependencies
pip3 install rich psutil numpy pillow matplotlib pyyaml

# Or minimal (no dashboard, no visualization)
pip3 install psutil numpy pyyaml
```

## Integration Examples

### Example 1: Add to Launch File

```python
# In your main launch file
from launch_ros.actions import Node

diagnostics = Node(
    package='your_package',
    executable='system_diagnostics.py',
    name='system_diagnostics',
    output='screen'
)

# Add to launch description
return LaunchDescription([
    # ... other nodes ...
    diagnostics,
])
```

### Example 2: Subscribe to Diagnostics

```python
from diagnostic_msgs.msg import DiagnosticArray

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

    def diagnostics_callback(self, msg):
        for status in msg.status:
            if status.level >= DiagnosticStatus.WARN:
                self.get_logger().warn(f"{status.name}: {status.message}")
```

### Example 3: Automated Health Check in CI/CD

```bash
#!/bin/bash
# ci_health_check.sh

cd scripts/diagnostics/
./run_full_diagnostics.sh /tmp/ci_diagnostics

if [ $? -eq 0 ]; then
    echo "✓ Health check passed"
    exit 0
else
    echo "✗ Health check failed"
    cat /tmp/ci_diagnostics/diagnostics_report.txt
    exit 1
fi
```

## Performance Benchmarks

| Metric | Excellent | Good | Acceptable | Poor |
|--------|-----------|------|------------|------|
| Mean Latency | < 50ms | 50-100ms | 100-200ms | > 200ms |
| P95 Latency | < 80ms | 80-150ms | 150-300ms | > 300ms |
| CPU Usage | < 50% | 50-70% | 70-90% | > 90% |
| Memory Usage | < 50% | 50-75% | 75-90% | > 90% |
| Localization | < 0.1m | < 0.3m | < 0.5m | > 0.5m |
| Topic Rates | > 90% | 80-90% | 50-80% | < 50% |

## Troubleshooting Quick Reference

| Symptom | Command | Expected Result |
|---------|---------|-----------------|
| Robot not moving | `python3 topic_checker.py /cmd_vel --echo` | Messages publishing |
| Robot lost | `python3 localization_quality.py` | < 0.3m uncertainty |
| Slow navigation | `python3 performance_profiler.py --duration 30` | < 100ms latency |
| TF errors | `python3 tf_tree_visualizer.py --check` | "TF tree is healthy" |
| High CPU | `python3 system_diagnostics.py` | < 70% CPU |
| Navigation fails | `./run_full_diagnostics.sh` | All checks pass |

## Support and Resources

### Documentation
- [QUICKSTART.md](scripts/diagnostics/QUICKSTART.md) - 5-minute guide
- [README.md](scripts/diagnostics/README.md) - Complete documentation
- [diagnostics_system.md](findings/diagnostics_system.md) - System architecture
- [DIAGNOSTICS_TOOLS_SUMMARY.md](findings/DIAGNOSTICS_TOOLS_SUMMARY.md) - Feature summary

### Help Commands
```bash
# All tools support --help
python3 system_diagnostics.py --help
python3 monitoring_dashboard.py --help
python3 tf_tree_visualizer.py --help
python3 topic_checker.py --help
python3 map_quality_analyzer.py --help
python3 localization_quality.py --help
python3 performance_profiler.py --help
```

### Common Commands
```bash
# Health check
python3 tf_tree_visualizer.py --check

# Monitor topics
python3 topic_checker.py --all

# Check localization
python3 localization_quality.py --monitor 10

# Profile performance
python3 performance_profiler.py --duration 30

# Full diagnostics
./run_full_diagnostics.sh
```

## Version History

### Version 1.0.0 (2026-01-11)
- Initial release
- 7 diagnostic tools
- Complete documentation
- Troubleshooting workflows
- Automation scripts
- Production ready

## License

Part of the WayfindR navigation system project.

## Next Steps

1. **Install Dependencies**
   ```bash
   pip3 install rich psutil numpy pillow matplotlib pyyaml
   ```

2. **Try Quick Start**
   ```bash
   cd scripts/diagnostics/
   python3 monitoring_dashboard.py
   ```

3. **Read Documentation**
   - Start with [QUICKSTART.md](scripts/diagnostics/QUICKSTART.md)
   - Then [README.md](scripts/diagnostics/README.md)
   - Finally [diagnostics_system.md](findings/diagnostics_system.md)

4. **Run Full Diagnostics**
   ```bash
   ./run_full_diagnostics.sh
   ```

5. **Integrate into Workflow**
   - Add to launch files
   - Set up automated checks
   - Train team on troubleshooting

---

**Status:** Complete and Production Ready
**Total Tools:** 7 diagnostic tools + 1 automation script
**Documentation:** 4 comprehensive guides (46KB)
**Last Updated:** 2026-01-11
