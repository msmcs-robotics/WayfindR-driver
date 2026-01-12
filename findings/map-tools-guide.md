# ROS2 Map Tools Guide

Complete guide to the map analysis and validation tools for ROS2 occupancy grid maps.

## Overview

This toolset provides four Python scripts for working with ROS2 occupancy grid maps (.pgm + .yaml files):

1. **validate_map.py** - Validates map file correctness
2. **map_info.py** - Displays comprehensive map information
3. **map_converter.py** - Converts and modifies maps
4. **compare_maps.py** - Compares two maps

All tools are located in: `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/map_tools/`

## ROS2 Map File Format

### Map Components

A ROS2 occupancy grid map consists of two files:

1. **YAML file** (metadata) - Contains map parameters
2. **PGM file** (image) - Contains the actual occupancy grid data

### YAML File Format

```yaml
image: map_name.pgm           # Image filename (can be absolute or relative)
resolution: 0.05              # Meters per pixel
origin: [-4.88, -4.09, 0]    # [x, y, theta] - position of (0,0) pixel
negate: 0                     # 0 or 1 - whether to invert pixel values
occupied_thresh: 0.65         # Threshold for occupied cells [0-1]
free_thresh: 0.25             # Threshold for free cells [0-1]
mode: trinary                 # trinary, scale, or raw
```

### PGM File Format

- Grayscale image (typically 8-bit)
- Pixel values represent occupancy probability:
  - **0 (black)**: Occupied (100% probability)
  - **255 (white)**: Free (0% probability)
  - **205 (gray)**: Unknown (no information)
  - **Intermediate values**: Partial occupancy probability

### Coordinate System

- **Origin**: Lower-left corner of the map in world coordinates
- **X-axis**: Points right (increases with column number)
- **Y-axis**: Points up (image Y is inverted - row 0 is top)
- **Theta**: Rotation of map frame (radians)

**Important**: Image coordinates are inverted from map coordinates!
- Image (0,0) = upper-left
- Map (0,0) = lower-left (origin parameter)

## Tool 1: validate_map.py

### Purpose
Validates ROS2 map files for correctness, catching common errors before using maps in SLAM or navigation.

### Usage

```bash
# Basic validation
python3 validate_map.py map.yaml

# Validate with full path
python3 validate_map.py /path/to/maps/office_map.yaml

# Validate relative path
python3 validate_map.py ../maps/first_map.yaml
```

### What It Checks

1. **File Existence**
   - YAML file exists
   - PGM file exists and path is correct

2. **YAML Format**
   - Valid YAML syntax
   - All required fields present
   - Correct data types

3. **Resolution**
   - Positive value
   - Reasonable range (warns if too small/large)
   - Calculates real-world dimensions

4. **Origin**
   - Correct format [x, y, theta]
   - All values are numbers

5. **Thresholds**
   - In valid range [0, 1]
   - free_thresh < occupied_thresh
   - Checks for standard values (0.25, 0.65)

6. **Image Properties**
   - Correct format (grayscale or binary)
   - Reasonable dimensions
   - Valid pixel values

7. **Cell Statistics**
   - Counts free/occupied/unknown cells
   - Calculates percentages

### Example Output

```
======================================================================
ROS2 Map Validation Tool
======================================================================

Validating: /path/to/map.yaml

----------------------------------------------------------------------
VALIDATION RESULTS
----------------------------------------------------------------------

INFORMATION (12):
  [INFO]  YAML file loaded successfully
  [INFO]  Image file loaded successfully: first_map.pgm
  [INFO]    Image size: 480 x 640 pixels
  [INFO]    Image mode: L
  [INFO]  Resolution: 0.05 m/pixel (OK)
  [INFO]    Real-world size: 24.00m x 32.00m
  [INFO]  Origin: [-4.88, -4.09, 0] (OK)
  [INFO]    Position: (-4.88m, -4.09m)
  [INFO]    Orientation: 0.00 rad (0.0 deg)
  [INFO]  Thresholds: free=0.25, occupied=0.65 (OK)
  [INFO]    Using standard ROS2 threshold values
  [INFO]  Mode: trinary (OK)

----------------------------------------------------------------------
RESULT: VALIDATION PASSED
Map appears to be correctly formatted.
----------------------------------------------------------------------
```

### When to Use

- **Before using a new map** - Verify it's correctly formatted
- **After creating a map** - Validate SLAM output
- **When debugging** - Check for file/format issues
- **Before navigation** - Ensure map is usable

## Tool 2: map_info.py

### Purpose
Displays comprehensive information about a map including metadata, dimensions, statistics, and coordinate system details.

### Usage

```bash
# Basic info
python3 map_info.py map.yaml

# Detailed info with histogram
python3 map_info.py --detailed map.yaml
python3 map_info.py -d map.yaml
```

### Information Displayed

1. **File Information**
   - File names and paths
   - File sizes

2. **Map Metadata**
   - All YAML parameters
   - Resolution, origin, thresholds, mode, negate

3. **Dimensions**
   - Pixel dimensions
   - Real-world size (meters and feet)
   - Total area
   - Aspect ratio

4. **Coordinate System**
   - Origin position and orientation
   - Corner coordinates
   - Map center
   - Coordinate frame details

5. **Cell Statistics**
   - Count of free/occupied/unknown cells
   - Percentages
   - Physical areas
   - Pixel value statistics (min, max, mean, median, std dev)

6. **Detailed Statistics** (with -d flag)
   - Pixel value percentiles
   - Map complexity metrics
   - Edge density

7. **Histogram** (with -d flag)
   - Distribution of pixel values
   - Visual bar chart

### Example Output

```
======================================================================
ROS2 Map Information
======================================================================

FILE INFORMATION
----------------------------------------------------------------------
  YAML file: first_map.yaml
  Directory: /path/to/maps
  Image file: first_map.pgm
  YAML size: 156 bytes
  Image size: 307,200 bytes (300.0 KB)

MAP METADATA
----------------------------------------------------------------------
  Resolution: 0.05 m/pixel
  Origin: [-4.88, -4.09, 0]
  Mode: trinary
  Negate: 0
  Occupied threshold: 0.65
  Free threshold: 0.25

DIMENSIONS
----------------------------------------------------------------------
  Image dimensions: 480 x 640 pixels
  Total pixels: 307,200
  Real-world width: 24.00 m (78.74 ft)
  Real-world height: 32.00 m (104.99 ft)
  Real-world area: 768.00 m² (8267.66 ft²)
  Aspect ratio: 1.33:1

COORDINATE SYSTEM
----------------------------------------------------------------------
  Origin (lower-left): (-4.880, -4.090) m
  Origin orientation: 0.000 rad (0.0°)
  Lower-left corner: (-4.880, -4.090) m
  Lower-right corner: (19.120, -4.090) m
  Upper-left corner: (-4.880, 27.910) m
  Upper-right corner: (19.120, 27.910) m
  Center: (7.120, 11.910) m

CELL STATISTICS
----------------------------------------------------------------------
  Total cells: 307,200
  Free cells: 245,760 (80.00%)
  Occupied cells: 15,360 (5.00%)
  Unknown cells: 46,080 (15.00%)

  Physical areas (resolution = 0.05 m/pixel):
    Free area: 614.40 m² (6614.13 ft²)
    Occupied area: 38.40 m² (413.38 ft²)
    Unknown area: 115.20 m² (1240.15 ft²)

  Pixel value statistics:
    Min: 0
    Max: 255
    Mean: 217.50
    Median: 255.00
    Std dev: 45.32
    Unique values: 3

======================================================================
```

### When to Use

- **Understanding a map** - Get complete picture of map properties
- **Planning navigation** - Know map dimensions and coverage
- **Quality assessment** - Check cell distribution and statistics
- **Documentation** - Generate map specifications
- **Debugging** - Understand coordinate system

## Tool 3: map_converter.py

### Purpose
Modify and convert ROS2 maps: resize, crop, adjust parameters, rotate, flip, or invert.

### Usage

```bash
# Change resolution (resizes map)
python3 map_converter.py input.yaml --output output.yaml --resize 0.1

# Crop to specific pixel coordinates
python3 map_converter.py input.yaml -o output.yaml --crop 100 100 500 500

# Adjust occupancy thresholds
python3 map_converter.py input.yaml -o output.yaml --thresholds 0.2 0.7

# Rotate map
python3 map_converter.py input.yaml -o output.yaml --rotate 90

# Flip map
python3 map_converter.py input.yaml -o output.yaml --flip horizontal

# Invert colors
python3 map_converter.py input.yaml -o output.yaml --invert

# Multiple operations (applied in order)
python3 map_converter.py input.yaml -o output.yaml --resize 0.05 --crop 0 0 1000 1000 --rotate 90
```

### Operations

#### 1. Resize (Change Resolution)

Changes map resolution by resizing the image.

```bash
--resize 0.1  # New resolution in m/pixel
```

- Larger resolution = smaller file, less detail
- Smaller resolution = larger file, more detail
- Uses NEAREST interpolation to preserve discrete values
- Automatically updates resolution in YAML

**Use case**: Reduce map file size or increase detail level

#### 2. Crop

Crops map to specified pixel coordinates.

```bash
--crop X1 Y1 X2 Y2
```

- Coordinates are in pixels (0,0 = upper-left)
- X1, Y1 = top-left corner of crop region
- X2, Y2 = bottom-right corner of crop region
- Automatically updates origin in YAML

**Use case**: Extract region of interest, reduce map size

#### 3. Adjust Thresholds

Changes occupancy thresholds without modifying image.

```bash
--thresholds FREE_THRESH OCCUPIED_THRESH
```

- Both values must be in [0, 1]
- FREE_THRESH must be < OCCUPIED_THRESH
- Standard values: 0.25 and 0.65

**Use case**: Fine-tune what counts as occupied/free

#### 4. Rotate

Rotates map by 90, 180, or 270 degrees.

```bash
--rotate 90   # or 180, or 270
```

- Updates orientation in YAML
- Note: Origin position may need manual adjustment

**Use case**: Correct map orientation

#### 5. Flip

Flips map horizontally or vertically.

```bash
--flip horizontal
--flip vertical
```

- Note: Origin may need manual adjustment

**Use case**: Mirror map

#### 6. Invert

Inverts pixel colors (black ↔ white).

```bash
--invert
```

- Toggles negate flag in YAML
- Swaps meaning of black and white pixels

**Use case**: Correct inverted map data

### When to Use

- **Reduce file size** - Resize to larger resolution
- **Increase detail** - Resize to smaller resolution
- **Extract region** - Crop to area of interest
- **Fix orientation** - Rotate or flip map
- **Adjust sensitivity** - Change thresholds
- **Fix colors** - Invert if map is backwards

## Tool 4: compare_maps.py

### Purpose
Compare two maps to assess SLAM quality, detect changes, or analyze differences.

### Usage

```bash
# Basic comparison (statistics only)
python3 compare_maps.py map1.yaml map2.yaml

# Generate difference visualization
python3 compare_maps.py map1.yaml map2.yaml --diff difference.png

# Generate overlay visualization
python3 compare_maps.py map1.yaml map2.yaml --overlay overlay.png

# Generate side-by-side comparison
python3 compare_maps.py map1.yaml map2.yaml --side-by-side comparison.png

# Generate all visualizations
python3 compare_maps.py map1.yaml map2.yaml --diff diff.png --overlay over.png --side-by-side sbs.png
```

### Comparison Metrics

1. **Metadata Comparison**
   - Resolution match
   - Dimension match
   - Origin match
   - Threshold comparison

2. **Pixel Statistics**
   - Identical pixels (%)
   - Changed pixels (%)
   - Mean absolute difference
   - RMS difference

3. **Change Analysis**
   - Small changes (1-50 pixel value difference)
   - Medium changes (51-150)
   - Large changes (151-255)

4. **Occupancy Changes**
   - Free → Occupied cells
   - Occupied → Free cells
   - Unknown cell changes

5. **Similarity Metrics**
   - Overall similarity percentage
   - Assessment (identical, very similar, moderately similar, significantly different)

### Visualizations

#### 1. Difference Map (--diff)

Color-coded visualization of differences:

- **Gray**: No change
- **Red**: Became occupied (was free, now occupied)
- **Green**: Became free (was occupied, now free)
- **Yellow**: Other changes

**Use case**: Identify where maps differ, spot mapping errors

#### 2. Overlay (--overlay)

Overlays both maps in different colors:

- **Red**: Only in map 1
- **Green**: Only in map 2
- **Yellow**: In both maps (overlap)
- **Black**: Free space in both

**Use case**: Visual comparison, alignment check

#### 3. Side-by-Side (--side-by-side)

Places both maps next to each other with labels.

**Use case**: Quick visual comparison, presentations

### Example Output

```
======================================================================
METADATA COMPARISON
======================================================================

Resolution:
  Map 1: 0.05
  Map 2: 0.05
  Status: Match

Dimensions:
  Map 1: 480 x 640 pixels
  Map 2: 480 x 640 pixels
  Status: Match

Origin:
  Map 1: [-4.88, -4.09, 0]
  Map 2: [-4.85, -4.10, 0]
  WARNING: Origins differ!

======================================================================
PIXEL COMPARISON
======================================================================

Pixel Statistics:
  Total pixels: 307,200
  Identical pixels: 295,680 (96.25%)
  Changed pixels: 11,520 (3.75%)

Difference Statistics:
  Mean absolute difference: 4.32
  Max absolute difference: 255
  RMS difference: 12.45

Change Magnitude:
  Small (1-50): 8,640 (2.81%)
  Medium (51-150): 1,920 (0.62%)
  Large (151-255): 960 (0.31%)

Occupancy Changes:
  Free -> Occupied: 480 (0.16%)
  Occupied -> Free: 320 (0.10%)
  Unknown in Map 1: 46,080 (15.00%)
  Unknown in Map 2: 42,240 (13.75%)

Similarity Metrics:
  Overall similarity: 98.31%
  Assessment: Maps are nearly identical
```

### When to Use

- **SLAM quality assessment** - Compare successive mapping runs
- **Change detection** - Detect environmental changes
- **Algorithm comparison** - Compare different SLAM algorithms
- **Validation** - Verify map against ground truth
- **Progress tracking** - Monitor mapping progress

## Common Use Cases

### 1. Validating a New SLAM Map

```bash
# First validate the map
python3 validate_map.py slam_output.yaml

# Get detailed information
python3 map_info.py --detailed slam_output.yaml

# If resolution too high, reduce it
python3 map_converter.py slam_output.yaml -o optimized.yaml --resize 0.1
```

### 2. Preparing Map for Navigation

```bash
# Validate map
python3 validate_map.py raw_map.yaml

# Crop to navigation area
python3 map_converter.py raw_map.yaml -o nav_map.yaml --crop 50 50 950 950

# Adjust thresholds for navigation
python3 map_converter.py nav_map.yaml -o final_map.yaml --thresholds 0.20 0.70

# Verify final map
python3 validate_map.py final_map.yaml
```

### 3. Comparing SLAM Runs

```bash
# Compare two mapping runs
python3 compare_maps.py run1.yaml run2.yaml --diff differences.png

# Generate all visualizations
python3 compare_maps.py run1.yaml run2.yaml \
  --diff diff.png \
  --overlay overlay.png \
  --side-by-side comparison.png
```

### 4. Troubleshooting Map Issues

```bash
# Check what's wrong
python3 validate_map.py problem_map.yaml

# Get detailed statistics
python3 map_info.py -d problem_map.yaml

# If colors inverted
python3 map_converter.py problem_map.yaml -o fixed_map.yaml --invert

# Validate fix
python3 validate_map.py fixed_map.yaml
```

## Common Map Issues and Fixes

### Issue 1: Map File Not Found

**Error**: `Image file does not exist`

**Fix**: Check that the 'image' field in YAML points to the correct PGM file. Use absolute path or ensure relative path is correct.

```bash
# Check current paths
python3 validate_map.py map.yaml

# Edit YAML to fix path
nano map.yaml
```

### Issue 2: Inverted Colors

**Symptom**: Black and white reversed (free space is black)

**Fix**: Use map converter to invert

```bash
python3 map_converter.py map.yaml -o fixed.yaml --invert
```

### Issue 3: Wrong Resolution

**Symptom**: Map too large/small for navigation

**Fix**: Resize map

```bash
# Increase resolution (make smaller file)
python3 map_converter.py map.yaml -o smaller.yaml --resize 0.1

# Decrease resolution (more detail)
python3 map_converter.py map.yaml -o detailed.yaml --resize 0.025
```

### Issue 4: Threshold Errors

**Error**: `free_thresh must be < occupied_thresh`

**Fix**: Adjust thresholds

```bash
python3 map_converter.py map.yaml -o fixed.yaml --thresholds 0.25 0.65
```

### Issue 5: Map Too Large

**Symptom**: High memory usage, slow processing

**Fix**: Crop or reduce resolution

```bash
# Option 1: Crop to region of interest
python3 map_converter.py large.yaml -o cropped.yaml --crop 0 0 1000 1000

# Option 2: Reduce resolution
python3 map_converter.py large.yaml -o smaller.yaml --resize 0.1
```

## Map Quality Metrics

### Good Map Characteristics

- **Resolution**: 0.05 - 0.1 m/pixel (balance of detail and size)
- **Free space**: 60-85% (enough navigable area)
- **Unknown**: < 20% (well-explored)
- **Occupied**: 5-20% (realistic obstacles)
- **File size**: < 10 MB (manageable)

### Red Flags

- Unknown cells > 30% (poor exploration)
- Free space < 50% (too cluttered)
- Very high resolution (< 0.01 m/pixel) without reason
- Very low resolution (> 0.5 m/pixel)
- Extreme aspect ratio (> 10:1)

## Dependencies

All tools require:

```bash
pip3 install pillow numpy pyyaml
```

Or in ROS2 environment:
```bash
sudo apt install python3-pil python3-numpy python3-yaml
```

## Script Locations

```
ros2_comprehensive_attempt/
└── scripts/
    └── map_tools/
        ├── validate_map.py      # Validation tool
        ├── map_info.py          # Information display
        ├── map_converter.py     # Conversion/modification
        └── compare_maps.py      # Map comparison
```

## Quick Reference

```bash
# Validate
python3 validate_map.py map.yaml

# Info
python3 map_info.py map.yaml
python3 map_info.py -d map.yaml  # detailed

# Convert
python3 map_converter.py input.yaml -o output.yaml [operations]
  --resize RESOLUTION
  --crop X1 Y1 X2 Y2
  --thresholds FREE OCC
  --rotate {90,180,270}
  --flip {horizontal,vertical}
  --invert

# Compare
python3 compare_maps.py map1.yaml map2.yaml [visualizations]
  --diff FILE
  --overlay FILE
  --side-by-side FILE
```

## Best Practices

1. **Always validate** maps before using them
2. **Back up originals** before converting
3. **Use descriptive names** for output files
4. **Document changes** when modifying maps
5. **Compare maps** to assess SLAM quality
6. **Check file sizes** after operations
7. **Verify coordinate systems** after cropping/rotating

## Integration with ROS2 Workflow

### During Mapping (SLAM)

```bash
# While SLAM is running, periodically save and check map
ros2 run nav2_map_server map_saver_cli -f current_map

# Validate the saved map
cd ~/maps
python3 /path/to/validate_map.py current_map.yaml

# Check progress
python3 /path/to/map_info.py current_map.yaml
```

### After Mapping

```bash
# Final validation
python3 validate_map.py final_map.yaml

# Optimize for navigation
python3 map_converter.py final_map.yaml -o nav_map.yaml --resize 0.05

# Compare with previous version
python3 compare_maps.py old_map.yaml final_map.yaml --diff changes.png
```

### Before Navigation

```bash
# Validate map one more time
python3 validate_map.py nav_map.yaml

# Check statistics
python3 map_info.py nav_map.yaml

# Start navigation with validated map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=nav_map.yaml
```

## Troubleshooting

### Tool won't run

```bash
# Make executable
chmod +x validate_map.py map_info.py map_converter.py compare_maps.py

# Run with python3
python3 validate_map.py map.yaml
```

### Import errors

```bash
# Install dependencies
pip3 install pillow numpy pyyaml

# Or in ROS2
sudo apt install python3-pil python3-numpy python3-yaml
```

### Permission denied

```bash
# Check file permissions
ls -l map.yaml map.pgm

# Fix if needed
chmod 644 map.yaml map.pgm
```

## Additional Resources

- ROS2 Map Server: http://wiki.ros.org/map_server
- PGM Format: https://en.wikipedia.org/wiki/Netpbm#PGM_example
- Occupancy Grids: http://wiki.ros.org/costmap_2d/hydro/inflation

---

**Created**: January 2026
**For**: WayfindR ROS2 Navigation Project
**Location**: `/home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/scripts/map_tools/`
