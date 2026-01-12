# Map Tools - Quick Reference Card

## Tool Selection

| Need | Use | Command |
|------|-----|---------|
| View map info | `map_viewer.py` | `--info` |
| Click coordinates | `map_viewer.py` | (interactive) |
| Plan waypoints | `map_viewer.py` | `--grid --export-grid` |
| Add waypoints (GUI) | `waypoint_annotator.py` | (interactive) |
| Add waypoints (CLI) | `map_editor.py` | `--add NAME X Y YAW` |
| Edit waypoints | `map_editor.py` | `--edit NAME x=X y=Y` |
| Delete waypoints | `map_editor.py` | `--remove NAME` |
| Visualize waypoints | `map_editor.py` | `--visualize` |
| List waypoints | `map_editor.py` | `--list` |

## Common Commands

### View Map
```bash
# Basic info
python3 map_viewer.py -m map.yaml --info

# Interactive with grid
python3 map_viewer.py -m map.yaml --grid

# Check coordinates
python3 map_viewer.py -m map.yaml --coords 100 70
```

### Add Waypoints (Interactive)
```bash
python3 waypoint_annotator.py -m map.yaml
# Click position → Click orientation → Enter name → Repeat → Save
```

### Add Waypoints (Command Line)
```bash
# Add single waypoint
python3 map_editor.py -m map.yaml \
    --add "name" 2.5 1.0 90 -o waypoints.yaml

# Add multiple
python3 map_editor.py -m map.yaml \
    --add "goal1" 2.0 1.0 0 \
    --add "goal2" 3.0 2.0 90 \
    -o waypoints.yaml
```

### Edit Waypoints
```bash
# Change position
python3 map_editor.py -m map.yaml -w waypoints.yaml \
    --edit "goal1" x=2.1 y=1.1 -o waypoints.yaml

# Change orientation
python3 map_editor.py -m map.yaml -w waypoints.yaml \
    --edit "goal1" yaw_degrees=45 -o waypoints.yaml
```

### Visualize
```bash
python3 map_editor.py -m map.yaml -w waypoints.yaml --visualize
```

## File Formats

### Map YAML (Input)
```yaml
image: map.pgm
resolution: 0.05
origin: [-4.88, -4.09, 0]
occupied_thresh: 0.65
free_thresh: 0.25
```

### Waypoint YAML (Output)
```yaml
waypoints:
  - name: goal1
    position: {x: 2.5, y: 1.0, z: 0.0}
    orientation: {x: 0, y: 0, z: 0.707, w: 0.707}
    yaw_degrees: 90.0
    tolerance: {position: 0.3, orientation: 0.2}
```

## Coordinate Systems

```
Pixel:        World:
┌─────→ X     ┌─────→ X (East)
│             │
↓ Y           ↑ Y (North)
```

**Conversion:**
- Pixel → World: Account for origin and resolution
- Y-axis is **inverted** between systems

## Orientation (Yaw)

```
     North (90°)
         ↑
         |
West ←───┼───→ East (0°)
(180°)   |
         ↓
     South (270°)
```

## Typical Workflow

```bash
# 1. Inspect map
python3 map_viewer.py -m map.yaml --info --grid

# 2. Plan waypoints (export grid if needed)
python3 map_viewer.py -m map.yaml --export-grid 0.5

# 3. Add waypoints
python3 waypoint_annotator.py -m map.yaml

# 4. Verify
python3 map_editor.py -m map.yaml -w map_waypoints.yaml --visualize --list

# 5. Use with ROS2
ros2 action send_goal /navigate_to_pose ...
```

## Tips

- **Names:** Use descriptive names (e.g., "kitchen_entrance" not "wp1")
- **Resolution:** 0.05 m/px is standard for indoor
- **Bounds:** Check with `--info` before adding waypoints
- **Backup:** Copy map/waypoint files before editing
- **Test:** Visualize before deploying to robot

## Troubleshooting

| Problem | Solution |
|---------|----------|
| tkinter error | `sudo apt-get install python3-tk` |
| Waypoint out of bounds | Check map bounds with `--info` |
| Wrong orientation | Remember: 0°=East, 90°=North |
| Coordinate mismatch | Verify origin and resolution in YAML |

## Help

```bash
python3 map_viewer.py --help
python3 map_editor.py --help
python3 waypoint_annotator.py --help
```

## Documentation

- Full guide: `/findings/map-editing-guide.md`
- Tool README: `scripts/map_tools/README.md`
- Summary: `/findings/map-tools-summary.md`
