#!/bin/bash
# Example usage of map_tools for ROS2 map editing and waypoint management

# This script demonstrates all the tools in action
# Adjust paths as needed for your environment

MAP_YAML="ros2_cartography_attempt/maps/first_map.yaml"
WAYPOINTS_YAML="ros2_cartography_attempt/maps/first_map_waypoints.yaml"

echo "========================================="
echo "ROS2 Map Tools - Example Usage"
echo "========================================="
echo

# 1. View map information
echo "1. Viewing map information..."
python3 map_viewer.py --map-yaml "$MAP_YAML" --info
echo

# 2. List existing waypoints
echo "2. Listing existing waypoints..."
python3 map_editor.py --map-yaml "$MAP_YAML" --waypoints "$WAYPOINTS_YAML" --list
echo

# 3. Check coordinates at specific location
echo "3. Checking coordinates at pixel (100, 70)..."
python3 map_viewer.py --map-yaml "$MAP_YAML" --coords 100 70
echo

# 4. Add a new waypoint programmatically
echo "4. Adding a new waypoint 'test_point'..."
python3 map_editor.py --map-yaml "$MAP_YAML" \
    --waypoints "$WAYPOINTS_YAML" \
    --add "test_point" 1.0 0.5 45 "Test waypoint added by script" \
    --output /tmp/test_waypoints.yaml
echo

# 5. Export grid points for planning
echo "5. Exporting grid points (0.5m spacing)..."
python3 map_viewer.py --map-yaml "$MAP_YAML" \
    --export-grid 0.5 \
    --output /tmp/grid_points.yaml
echo

echo "========================================="
echo "Examples completed!"
echo "========================================="
echo
echo "Generated files:"
echo "  /tmp/test_waypoints.yaml - Updated waypoints"
echo "  /tmp/grid_points.yaml - Grid of free space points"
echo
echo "Next steps:"
echo "  1. Try interactive annotator:"
echo "     python3 waypoint_annotator.py --map-yaml $MAP_YAML"
echo
echo "  2. Visualize waypoints:"
echo "     python3 map_editor.py --map-yaml $MAP_YAML --waypoints $WAYPOINTS_YAML --visualize"
echo
echo "  3. View map with grid overlay:"
echo "     python3 map_viewer.py --map-yaml $MAP_YAML --grid"
echo
