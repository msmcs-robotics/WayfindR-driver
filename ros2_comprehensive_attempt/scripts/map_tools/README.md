# ROS2 Map Tools

Practical tools for analyzing and validating ROS2 occupancy grid maps.

## Quick Start

```bash
# Validate a map
./validate_map.py /path/to/map.yaml

# Display map information
./map_info.py /path/to/map.yaml
./map_info.py --detailed /path/to/map.yaml

# Convert/modify map
./map_converter.py input.yaml --output output.yaml --resize 0.1
./map_converter.py input.yaml -o output.yaml --crop 100 100 500 500

# Compare two maps
./compare_maps.py map1.yaml map2.yaml
./compare_maps.py map1.yaml map2.yaml --diff differences.png
```

## Tools

### 1. validate_map.py
Validates map file correctness, checks format, resolution, thresholds, and image properties.

### 2. map_info.py
Displays comprehensive map information including metadata, dimensions, statistics, and coordinate system.

### 3. map_converter.py
Converts and modifies maps: resize, crop, adjust thresholds, rotate, flip, or invert.

### 4. compare_maps.py
Compares two maps for SLAM quality assessment and change detection.

## Dependencies

```bash
pip3 install pillow numpy pyyaml
```

Or in ROS2:
```bash
sudo apt install python3-pil python3-numpy python3-yaml
```

## Documentation

Complete documentation: `/home/devel/Desktop/WayfindR-driver/findings/map-tools-guide.md`

## Examples

```bash
# Validate SLAM output
./validate_map.py ~/maps/slam_map.yaml

# Get detailed statistics
./map_info.py --detailed ~/maps/slam_map.yaml

# Reduce map resolution
./map_converter.py large_map.yaml -o small_map.yaml --resize 0.1

# Compare mapping runs
./compare_maps.py run1.yaml run2.yaml --diff diff.png --overlay overlay.png

# Prepare map for navigation
./map_converter.py raw.yaml -o nav.yaml --crop 50 50 950 950 --thresholds 0.20 0.70
```

## Common Issues

- **File not found**: Check path in YAML 'image' field
- **Inverted colors**: Use `--invert` flag with map_converter.py
- **Wrong resolution**: Use `--resize` flag with map_converter.py
- **Threshold errors**: Use `--thresholds` flag with map_converter.py

## Integration with ROS2

```bash
# Save current map during SLAM
ros2 run nav2_map_server map_saver_cli -f my_map

# Validate the saved map
./validate_map.py my_map.yaml

# Optimize for navigation
./map_converter.py my_map.yaml -o optimized.yaml --resize 0.05

# Load in nav2
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=optimized.yaml
```
