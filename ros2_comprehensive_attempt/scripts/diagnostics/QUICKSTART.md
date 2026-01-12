# WayfindR Diagnostics - Quick Start Guide

Get started with the diagnostics system in 5 minutes.

## Installation

```bash
# Navigate to diagnostics directory
cd ros2_comprehensive_attempt/scripts/diagnostics/

# Install Python dependencies
pip3 install rich psutil numpy pillow matplotlib pyyaml

# Make scripts executable
chmod +x *.py
```

## First Steps

### 1. Check System Health (30 seconds)

```bash
# Check TF tree
python3 tf_tree_visualizer.py --check

# Expected output: "RESULT: TF tree is healthy"
```

### 2. Monitor Topics (Real-time)

```bash
# Check all critical topics
python3 topic_checker.py --all

# Expected: All topics show "OK" status
```

### 3. Start Dashboard (Visual Monitoring)

```bash
# Launch real-time dashboard
python3 monitoring_dashboard.py

# You'll see a live updating display with color-coded status
```

## Common Tasks

### Is My Robot Localized?

```bash
python3 localization_quality.py --monitor 10
# Should show "GOOD" or "EXCELLENT"
```

### Is Navigation Slow?

```bash
python3 performance_profiler.py --duration 30
# Check mean latency (should be < 100ms)
```

### Is My Map Good?

```bash
python3 map_quality_analyzer.py --map ../maps/office.yaml
# Should report "Map quality is GOOD"
```

### What's Wrong with My System?

```bash
# Start full diagnostics
python3 system_diagnostics.py

# In another terminal, watch the dashboard
python3 monitoring_dashboard.py

# Look for red ✗ or yellow ⚠ indicators
```

## Troubleshooting Cheat Sheet

| Problem | Command | What to Look For |
|---------|---------|------------------|
| Robot not moving | `python3 topic_checker.py /cmd_vel --echo` | Messages being published? |
| Robot lost | `python3 localization_quality.py` | Uncertainty > 0.5m |
| Slow navigation | `python3 performance_profiler.py --duration 30` | Latency > 100ms |
| TF errors | `python3 tf_tree_visualizer.py --check` | Missing or stale transforms |
| High CPU | `python3 system_diagnostics.py` | CPU > 80% |

## Quick Workflows

### Before Each Test Session

```bash
python3 tf_tree_visualizer.py --check && \
python3 topic_checker.py --all && \
echo "✓ System ready for testing"
```

### Weekly Health Check

```bash
# Run and save reports
python3 performance_profiler.py --duration 60 --export weekly_profile.txt
python3 localization_quality.py --monitor 30 --export weekly_loc.txt
python3 map_quality_analyzer.py --map ../maps/office.yaml
```

### When Something Goes Wrong

```bash
# Terminal 1: Diagnostics
python3 system_diagnostics.py

# Terminal 2: Dashboard
python3 monitoring_dashboard.py

# Terminal 3: Detailed checks
python3 localization_quality.py --monitor 20
python3 performance_profiler.py --duration 30
```

## Tips

1. **Keep dashboard running** during development for instant feedback
2. **Check TF tree first** when you see errors - many issues stem from TF
3. **Profile regularly** to catch performance degradation early
4. **Export reports** before and after parameter changes to compare
5. **Use --help** on any script for detailed options

## Getting Help

- Full documentation: See `README.md` in this directory
- Detailed guide: See `findings/diagnostics_system.md`
- Issues: Check troubleshooting workflows in documentation

## Next Steps

1. Read the full README.md for detailed usage
2. Review troubleshooting workflows for your specific issues
3. Integrate diagnostics into your launch files
4. Set up automated health checks for production

---

**Need help?** All scripts support `--help` flag for detailed options.
