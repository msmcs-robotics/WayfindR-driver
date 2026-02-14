# WayfindR Test Artifacts

This directory contains all test outputs, artifacts, and temporary files generated during development and testing. The contents of this directory are excluded from version control via `.gitignore`.

## Directory Structure

```
tests/
├── tf_frames/          # TF tree visualizations (PDF, GV files)
├── slam_outputs/       # SLAM test results and generated maps
├── rosbags/           # Recorded ROS2 bag files for testing
├── maps/              # Test maps (PGM, YAML)
├── waypoints/         # Test waypoint configurations
├── logs/              # Test execution logs
└── screenshots/       # RViz and GUI screenshots
```

## Purpose

This folder keeps the repository clean by consolidating all test artifacts in one location. Files here are:
- Generated automatically during tests
- Large binary files (rosbags, PDFs, images)
- Temporary outputs that don't need version control
- Session-specific test results

## Usage

### Automatic Organization

Most test scripts will automatically save their outputs to the appropriate subdirectory:

- **TF Frame Diagnostics**: `view_frames` outputs go to `tf_frames/`
- **SLAM Tests**: Map generation outputs go to `slam_outputs/`
- **Recording Sessions**: Rosbag files go to `rosbags/`
- **Map Validation**: Test maps go to `maps/`

### Manual Organization

For ad-hoc tests, manually move artifacts here:

```bash
# Move TF visualizations
mv frames_*.pdf frames_*.gv tests/tf_frames/

# Move SLAM outputs
mv slam_test_* tests/slam_outputs/

# Move test rosbags
mv *.db3 tests/rosbags/
```

### Cleanup

To free disk space, periodically clean old test artifacts:

```bash
# Remove all test outputs (use with caution!)
rm -rf tests/*

# Remove old SLAM tests only
rm -rf tests/slam_outputs/slam_test_*

# Remove old TF frames (keep last 5)
cd tests/tf_frames && ls -t | tail -n +6 | xargs rm -f
```

## Excluded from Git

All contents of this directory are excluded from version control via the root `.gitignore`:

```gitignore
# Test artifacts and outputs
tests/
**/frames_*.pdf
**/frames_*.gv
*.pgm
*.db3
*.mcap
**/test_run/
**/slam_test_*/
```

## Important Notes

1. **Not Backed Up**: Contents are not version controlled, so back up important test results manually
2. **Large Files**: Rosbags can be very large (GB), monitor disk usage
3. **Session Data**: TF frames and logs include timestamps for session tracking
4. **Reproducibility**: Keep configuration files in version control, not the outputs

## Test Session Tracking

Each test session generates timestamped artifacts:

- TF Frames: `frames_YYYY-MM-DD_HH.MM.SS.{pdf,gv}`
- SLAM Tests: `slam_test_YYYYMMDD_HHMMSS/`
- Recordings: `recording_YYYYMMDD_HHMMSS.db3`

This allows tracking test history while keeping the repository clean.

## Related Documentation

- [Testing Checklist](/ros2_comprehensive_attempt/findings/LOCAL_TESTING_CHECKLIST.md)
- [LiDAR Workflow Guide](/findings/lidar-data-workflow.md)
- [Map Tools Guide](/findings/map-editing-guide.md)

---

**Last Updated**: 2026-01-11
**Auto-generated**: Yes (by test scripts)
**Repository**: WayfindR-driver
