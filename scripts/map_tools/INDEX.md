# Map Tools - Complete Index

## Quick Links

| Document | Purpose | Location |
|----------|---------|----------|
| **Quick Reference** | Command cheatsheet | `QUICK_REFERENCE.md` |
| **Tool README** | Installation & usage | `README.md` |
| **Complete Guide** | Full documentation | `/findings/map-editing-guide.md` |
| **Research Summary** | Background & testing | `/findings/map-tools-summary.md` |

## Tools

### 1. map_viewer.py
**One-liner:** Inspect maps, view info, export grid points

**When to use:**
- First look at a new map
- Need map dimensions/bounds
- Planning waypoint locations
- Checking coordinates

**Key features:**
- Interactive coordinate display
- Grid overlay
- Map statistics
- Grid export

[Full docs →](#map_viewerpy-reference)

### 2. map_editor.py
**One-liner:** Command-line waypoint management

**When to use:**
- Scripted waypoint addition
- Batch operations
- Editing existing waypoints
- Automation workflows

**Key features:**
- Add/edit/remove waypoints
- Visualization
- Route management
- YAML import/export

[Full docs →](#map_editorpy-reference)

### 3. waypoint_annotator.py
**One-liner:** Interactive GUI for clicking waypoints

**When to use:**
- Manual waypoint placement
- Visual waypoint creation
- Quick annotation sessions
- When precision clicking is needed

**Key features:**
- Click-based interface
- Visual orientation setting
- Real-time preview
- Easy deletion/undo

[Full docs →](#waypoint_annotatorpy-reference)

## Getting Started

### Installation
```bash
# Install dependencies
sudo apt-get install python3-tk python3-pil python3-yaml
pip3 install numpy matplotlib pillow pyyaml

# Make scripts executable
chmod +x scripts/map_tools/*.py
```

### First Use
```bash
cd /home/devel/Desktop/WayfindR-driver

# 1. View your map
python3 scripts/map_tools/map_viewer.py \
    --map-yaml ros2_cartography_attempt/maps/first_map.yaml --info

# 2. Add waypoints interactively
python3 scripts/map_tools/waypoint_annotator.py \
    --map-yaml ros2_cartography_attempt/maps/first_map.yaml

# 3. Visualize result
python3 scripts/map_tools/map_editor.py \
    --map-yaml ros2_cartography_attempt/maps/first_map.yaml \
    --waypoints ros2_cartography_attempt/maps/first_map_waypoints.yaml \
    --visualize
```

## Workflows

### New Map → Waypoints
```bash
# After creating map with SLAM
ros2 run nav2_map_server map_saver_cli -f my_map

# Inspect
python3 scripts/map_tools/map_viewer.py -m my_map.yaml --info --grid

# Add waypoints
python3 scripts/map_tools/waypoint_annotator.py -m my_map.yaml

# Done! waypoints saved to my_map_waypoints.yaml
```

### Batch Waypoint Generation
```bash
# Use existing waypoint_manager for algorithmic waypoints
python3 ros2_cartography_attempt/waypoint_manager.py \
    -m map.yaml --add-center --add-corners -o waypoints.yaml

# Add custom waypoints
python3 scripts/map_tools/map_editor.py -m map.yaml -w waypoints.yaml \
    --add "custom1" 2.0 1.0 90 -o waypoints.yaml
```

### Edit Existing Waypoints
```bash
# Load and modify
python3 scripts/map_tools/map_editor.py -m map.yaml -w waypoints.yaml \
    --edit "goal1" x=2.5 y=1.2 yaw_degrees=95 \
    --remove "old_point" \
    -o waypoints.yaml

# Or use GUI
python3 scripts/map_tools/waypoint_annotator.py -m map.yaml -w waypoints.yaml
```

## File Structure

```
scripts/map_tools/
├── __init__.py              # Package init
├── map_viewer.py            # Map inspection tool
├── map_editor.py            # CLI waypoint manager
├── waypoint_annotator.py    # GUI annotator
├── example_usage.sh         # Example script
├── README.md                # Tool documentation
├── QUICK_REFERENCE.md       # Command cheatsheet
└── INDEX.md                 # This file

findings/
├── map-editing-guide.md     # Complete guide (21K)
└── map-tools-summary.md     # Research summary (18K)
```

## Reference Sections

### map_viewer.py Reference

**Purpose:** Read-only map inspection and analysis

**Usage:**
```bash
python3 map_viewer.py --map-yaml MAP [OPTIONS]
```

**Options:**
- `--info` / `-i`: Print detailed map information
- `--visualize` / `-v`: Show map visualization
- `--grid` / `-g`: Show grid overlay
- `--grid-spacing N`: Grid spacing in pixels (default: 50)
- `--no-ruler`: Hide scale ruler
- `--coords PX PY`: Show coordinates at pixel location
- `--export-grid SPACING`: Export grid points (spacing in meters)
- `--output FILE`: Save visualization or grid to file

**Examples:**
```bash
# Full info display
python3 map_viewer.py -m map.yaml --info

# Interactive viewer with grid
python3 map_viewer.py -m map.yaml --grid --grid-spacing 25

# Export planning grid
python3 map_viewer.py -m map.yaml --export-grid 0.5 -o grid.yaml

# Check specific coordinates
python3 map_viewer.py -m map.yaml --coords 100 70
```

**Output:**
- Map dimensions (pixels and meters)
- Resolution and origin
- World coordinate bounds
- Map center
- Content analysis (free/occupied/unknown)
- Interactive coordinate display on click

### map_editor.py Reference

**Purpose:** Programmatic waypoint management

**Usage:**
```bash
python3 map_editor.py --map-yaml MAP [OPTIONS]
```

**Options:**
- `-w` / `--waypoints FILE`: Load existing waypoints
- `-o` / `--output FILE`: Save waypoints to file
- `--add NAME X Y YAW [DESC]`: Add waypoint
- `--remove NAME`: Remove waypoint
- `--edit NAME [FIELDS]`: Edit waypoint (e.g., x=2.5 yaw_degrees=90)
- `--list` / `-l`: List all waypoints
- `--visualize` / `-v`: Show map with waypoints
- `--viz-output FILE`: Save visualization image

**Examples:**
```bash
# Add waypoint
python3 map_editor.py -m map.yaml \
    --add "kitchen" 2.5 1.0 90 "Kitchen waypoint" \
    -o waypoints.yaml

# Edit waypoint
python3 map_editor.py -m map.yaml -w waypoints.yaml \
    --edit "kitchen" x=2.6 y=1.1 yaw_degrees=85 \
    -o waypoints.yaml

# Remove waypoint
python3 map_editor.py -m map.yaml -w waypoints.yaml \
    --remove "old_point" -o waypoints.yaml

# List all
python3 map_editor.py -m map.yaml -w waypoints.yaml --list

# Visualize
python3 map_editor.py -m map.yaml -w waypoints.yaml --visualize
```

**Visualization:**
- Red circles: Waypoint positions
- Blue arrows: Orientations
- White labels: Names
- Saves to PNG/JPG with `--viz-output`

### waypoint_annotator.py Reference

**Purpose:** Interactive GUI waypoint annotation

**Usage:**
```bash
python3 waypoint_annotator.py --map-yaml MAP [--waypoints FILE]
```

**Options:**
- `-m` / `--map-yaml FILE`: Map YAML file (required)
- `-w` / `--waypoints FILE`: Load existing waypoints

**GUI Workflow:**
1. **Click #1:** Set waypoint position (red dot)
2. **Click #2:** Set orientation direction (blue arrow)
3. **Dialog:** Enter waypoint name
4. **Repeat:** Add more waypoints
5. **Save:** Click "Save Waypoints" button

**GUI Controls:**
- **Save Waypoints:** Export to YAML file
- **Clear All:** Delete all waypoints
- **Undo Last:** Remove most recent waypoint
- **Refresh View:** Redraw the map
- **Waypoint List:** Shows all waypoints
- **Double-click list item:** Delete waypoint

**Examples:**
```bash
# New waypoints
python3 waypoint_annotator.py -m map.yaml

# Edit existing
python3 waypoint_annotator.py -m map.yaml -w waypoints.yaml
```

**Tips:**
- Second click determines robot facing direction
- Arrow points where robot will face
- Waypoints auto-save to `<map>_waypoints.yaml`
- Use grid in map_viewer first to plan positions

## Common Tasks

### Task: "I want to see what's in this map"
```bash
python3 map_viewer.py -m map.yaml --info
```

### Task: "I need to add waypoints by clicking"
```bash
python3 waypoint_annotator.py -m map.yaml
```

### Task: "I need to add waypoints in a script"
```bash
python3 map_editor.py -m map.yaml \
    --add "goal1" 2.0 1.0 0 \
    --add "goal2" 3.0 2.0 90 \
    -o waypoints.yaml
```

### Task: "I want to change a waypoint's position"
```bash
python3 map_editor.py -m map.yaml -w waypoints.yaml \
    --edit "goal1" x=2.1 y=1.1 -o waypoints.yaml
```

### Task: "I need to see waypoints on the map"
```bash
python3 map_editor.py -m map.yaml -w waypoints.yaml --visualize
```

### Task: "I want coordinate conversion"
```bash
# Check specific pixel
python3 map_viewer.py -m map.yaml --coords 100 70

# Or interactive mode
python3 map_viewer.py -m map.yaml
# (click on map to see coordinates)
```

### Task: "I need to plan waypoint locations"
```bash
# View with grid
python3 map_viewer.py -m map.yaml --grid

# Export grid points
python3 map_viewer.py -m map.yaml --export-grid 0.5 -o grid.yaml
```

## Troubleshooting

### "No module named 'tkinter'"
```bash
sudo apt-get install python3-tk
```

### "No module named 'PIL'"
```bash
pip3 install Pillow
```

### "Waypoint out of bounds"
```bash
# Check map bounds first
python3 map_viewer.py -m map.yaml --info
# Ensure: min_x ≤ waypoint.x ≤ max_x
```

### "Wrong orientation"
Remember ROS2 convention:
- 0° = East (→)
- 90° = North (↑)
- 180° = West (←)
- 270° = South (↓)

### "Coordinates don't match"
Verify:
1. Resolution in YAML matches reality
2. Origin is correct
3. Using world (not pixel) coordinates for waypoints

## Additional Resources

### Project Files
- Existing waypoint manager: `ros2_cartography_attempt/waypoint_manager.py`
- Example map: `ros2_cartography_attempt/maps/first_map.yaml`
- Example waypoints: `ros2_cartography_attempt/maps/first_map_waypoints.yaml`

### Documentation
- **Complete Guide:** `/findings/map-editing-guide.md`
  - Map file formats
  - Manual editing
  - RViz visualization
  - Nav2 integration

- **Research Summary:** `/findings/map-tools-summary.md`
  - Background research
  - Testing results
  - Design decisions
  - Future enhancements

### External Links
- [Nav2 Documentation](https://docs.nav2.org)
- [ROS2 Map Server](https://github.com/ros-planning/navigation2/tree/main/nav2_map_server)
- [PGM Format Spec](http://netpbm.sourceforge.net/doc/pgm.html)

## Quick Help

```bash
# Tool help
python3 map_viewer.py --help
python3 map_editor.py --help
python3 waypoint_annotator.py --help

# Run examples
./example_usage.sh

# View documentation
cat QUICK_REFERENCE.md
cat README.md
less /findings/map-editing-guide.md
```

---

**Last Updated:** 2026-01-11
**Version:** 1.0.0
**Author:** Created for WayfindR-driver project
