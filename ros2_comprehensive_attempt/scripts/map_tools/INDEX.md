# Map Tools Index

## Overview
Comprehensive suite of ROS2 occupancy grid map analysis and validation tools.

## Directory Structure
```
scripts/map_tools/
├── validate_map.py          # Map validation tool
├── map_info.py              # Map information display
├── map_converter.py         # Map format conversion
├── compare_maps.py          # Map comparison tool
├── demo_tools.sh            # Interactive demo
├── test_tools.sh            # Automated test suite
├── install_dependencies.sh  # Dependency installer
├── README.md                # Quick reference
└── INDEX.md                 # This file
```

## Quick Reference

| Tool | Purpose | Example |
|------|---------|---------|
| validate_map.py | Validate map correctness | `./validate_map.py map.yaml` |
| map_info.py | Display map information | `./map_info.py --detailed map.yaml` |
| map_converter.py | Convert/modify maps | `./map_converter.py in.yaml -o out.yaml --resize 0.1` |
| compare_maps.py | Compare two maps | `./compare_maps.py map1.yaml map2.yaml --diff diff.png` |

## Documentation
- **Complete Guide**: `/home/devel/Desktop/WayfindR-driver/findings/map-tools-guide.md`
- **Summary**: `/home/devel/Desktop/WayfindR-driver/findings/map-validation-tools-summary.md`

## Testing
Run automated tests: `./test_tools.sh`
Run interactive demo: `./demo_tools.sh`

## Installation
```bash
./install_dependencies.sh
```

## All Tools Pass Tests
✅ 20/20 tests passing
