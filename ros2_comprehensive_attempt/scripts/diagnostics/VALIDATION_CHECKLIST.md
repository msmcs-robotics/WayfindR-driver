# WayfindR Diagnostics - Validation Checklist

Use this checklist to verify the diagnostics system is properly installed and working.

## Installation Validation

### 1. File Verification
```bash
cd scripts/diagnostics/

# Check all files exist
ls -lh *.py *.sh *.md

# Verify executable permissions
ls -l *.py *.sh | grep "^-rwxrwxr-x"
```

**Expected:** All Python scripts and shell scripts should be executable.

### 2. Dependencies Check
```bash
# Test core dependencies (required)
python3 -c "import rclpy; print('✓ rclpy')"
python3 -c "import tf2_ros; print('✓ tf2_ros')"
python3 -c "import yaml; print('✓ pyyaml')"

# Test optional dependencies (for full features)
python3 -c "import rich; print('✓ rich')"
python3 -c "import psutil; print('✓ psutil')"
python3 -c "import numpy; print('✓ numpy')"
python3 -c "import PIL; print('✓ pillow')"
python3 -c "import matplotlib; print('✓ matplotlib')"
```

**Expected:** All core dependencies should print ✓. Optional dependencies enhance features.

**Fix missing dependencies:**
```bash
pip3 install rich psutil numpy pillow matplotlib pyyaml
```

## Functional Validation

### 3. Help Messages
```bash
# All tools should display help without errors
python3 system_diagnostics.py --help
python3 monitoring_dashboard.py --help
python3 tf_tree_visualizer.py --help
python3 topic_checker.py --help
python3 map_quality_analyzer.py --help
python3 localization_quality.py --help
python3 performance_profiler.py --help
```

**Expected:** Each tool displays help message without errors.

### 4. Syntax Validation
```bash
# Check for Python syntax errors
for script in *.py; do
    echo "Checking $script..."
    python3 -m py_compile "$script"
done
```

**Expected:** No syntax errors reported.

### 5. Shell Script Validation
```bash
# Check shell script syntax
bash -n run_full_diagnostics.sh
```

**Expected:** No syntax errors reported.

## Runtime Validation (ROS2 Required)

### 6. System Diagnostics Node (Requires Running ROS2 System)
```bash
# In terminal 1: Start ROS2 system (navigation stack)
# ros2 launch your_package navigation.launch.py

# In terminal 2: Start diagnostics
timeout 10 python3 system_diagnostics.py

# Should see:
# - "System diagnostics node started"
# - Periodic status updates
# - No Python errors
```

**Expected:** Node starts and publishes diagnostics without errors.

### 7. Monitoring Dashboard (Requires Running ROS2 System)
```bash
# Requires 'rich' library
timeout 10 python3 monitoring_dashboard.py

# Should see:
# - Color-coded dashboard display
# - Real-time updates
# - No rendering errors
```

**Expected:** Dashboard displays with live updates (if rich is installed).

### 8. TF Tree Visualizer (Requires Running ROS2 System)
```bash
timeout 10 python3 tf_tree_visualizer.py --check

# Should see:
# - TF TREE HEALTH CHECK
# - List of transforms checked
# - Overall result (PASSED/ISSUES)
```

**Expected:** Displays TF tree structure and health status.

### 9. Topic Checker (Requires Running ROS2 System)
```bash
# Check if topics are being monitored
timeout 15 python3 topic_checker.py /scan --hz &
PID=$!
sleep 10
kill $PID 2>/dev/null

# Should see:
# - "Monitoring topics in hz mode"
# - Topic statistics table
```

**Expected:** Displays topic frequency statistics.

## Standalone Validation (No ROS2 Required)

### 10. Map Quality Analyzer (Standalone)
```bash
# Test with example map (if available)
if [ -f "../../maps/office.yaml" ]; then
    python3 map_quality_analyzer.py --map ../../maps/office.yaml
else
    echo "No map file found - this is OK if you don't have maps yet"
fi

# Should see (if map exists):
# - MAP QUALITY ANALYSIS
# - Coverage statistics
# - Connectivity analysis
# - Overall assessment
```

**Expected:** Analyzes map if available, or reports file not found.

### 11. Full Diagnostics Script (Requires Running ROS2 System)
```bash
# Run full diagnostics suite
./run_full_diagnostics.sh /tmp/diagnostics_test

# Should see:
# - Color-coded progress for each test
# - Summary with passed/failed counts
# - "Overall Result: ..."
```

**Expected:** Runs all checks and generates report in /tmp/diagnostics_test/.

## Documentation Validation

### 12. Documentation Files
```bash
# Verify documentation exists
ls -lh README.md QUICKSTART.md
ls -lh ../../findings/diagnostics_system.md
ls -lh ../../findings/DIAGNOSTICS_TOOLS_SUMMARY.md
ls -lh ../../DIAGNOSTICS_INDEX.md
```

**Expected:** All documentation files exist.

### 13. Documentation Readability
```bash
# Check markdown syntax (if markdown linter available)
# mdl README.md QUICKSTART.md

# Or just view to check formatting
cat QUICKSTART.md | head -20
```

**Expected:** Documentation is properly formatted and readable.

## Integration Validation

### 14. Launch File
```bash
# Verify launch file exists
ls -lh ../../launch/diagnostics.launch.py

# Check syntax
python3 -m py_compile ../../launch/diagnostics.launch.py
```

**Expected:** Launch file exists and has no syntax errors.

### 15. Package Structure
```bash
# Verify __init__.py exists
ls -lh __init__.py

# Check it's valid Python
python3 -c "import sys; sys.path.insert(0, '.'); import diagnostics"
```

**Expected:** Package structure is valid.

## Validation Summary

After running all checks, you should have:

- ✓ All 7 diagnostic tools present and executable
- ✓ All dependencies installed (or optional ones documented)
- ✓ No syntax errors in any script
- ✓ Help messages display correctly
- ✓ Tools run without errors (when ROS2 is available)
- ✓ Documentation complete and readable
- ✓ Launch file valid
- ✓ Package structure correct

## Common Issues and Solutions

### Issue 1: Permission Denied
**Symptom:** `./run_full_diagnostics.sh: Permission denied`

**Solution:**
```bash
chmod +x *.py *.sh
```

### Issue 2: Module Not Found
**Symptom:** `ModuleNotFoundError: No module named 'rich'`

**Solution:**
```bash
pip3 install rich psutil numpy pillow matplotlib pyyaml
```

### Issue 3: ROS2 Not Sourced
**Symptom:** Tools report ROS2 errors or can't find topics

**Solution:**
```bash
source /opt/ros/humble/setup.bash
# Or your specific ROS2 installation
```

### Issue 4: No TF Data
**Symptom:** TF tree visualizer shows no frames

**Solution:**
- Ensure robot or simulation is running
- Check robot_state_publisher is active
- Verify TF is being published: `ros2 topic echo /tf --once`

### Issue 5: Dashboard Not Displaying
**Symptom:** Monitoring dashboard shows plain text instead of colors

**Solution:**
```bash
# Install rich library
pip3 install rich

# Or run without dashboard
python3 system_diagnostics.py  # Works without rich
```

## Quick Validation Command

Run this one-liner to validate basic setup:
```bash
cd scripts/diagnostics/ && \
python3 -c "import rclpy, tf2_ros, yaml, psutil, numpy; print('✓ Core dependencies OK')" && \
python3 -m py_compile *.py && echo '✓ No syntax errors' && \
bash -n run_full_diagnostics.sh && echo '✓ Shell script valid' && \
ls -lh README.md QUICKSTART.md && echo '✓ Documentation present' && \
echo '✓✓✓ VALIDATION PASSED ✓✓✓'
```

## Next Steps After Validation

1. Read QUICKSTART.md
2. Try monitoring dashboard: `python3 monitoring_dashboard.py`
3. Run full diagnostics: `./run_full_diagnostics.sh`
4. Review troubleshooting workflows in README.md
5. Integrate into your launch files

---

**Checklist Version:** 1.0
**Last Updated:** 2026-01-11
