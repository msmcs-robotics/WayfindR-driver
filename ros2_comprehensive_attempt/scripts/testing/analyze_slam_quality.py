#!/usr/bin/env python3
"""
SLAM Map Quality Analyzer

Analyzes the quality of SLAM-generated maps by examining:
- Map coverage (known vs unknown space)
- Obstacle density
- Map resolution
- Potential issues

Usage:
    python3 analyze_slam_quality.py <map_file.yaml>

Or as a ROS2 node (subscribes to /map topic):
    python3 analyze_slam_quality.py --live

Author: WayfindR Development Team
Date: 2026-01-11
"""

import sys
import yaml
import numpy as np
from pathlib import Path
import argparse


def load_map_from_file(yaml_file):
    """Load map from PGM file using map YAML"""
    with open(yaml_file, 'r') as f:
        map_info = yaml.safe_load(f)

    # Get PGM filename
    pgm_file = Path(yaml_file).parent / map_info['image']

    # Load PGM file
    with open(pgm_file, 'rb') as f:
        # Read PGM header
        magic = f.readline().strip()
        if magic not in [b'P5', b'P2']:
            raise ValueError(f"Not a valid PGM file: {magic}")

        # Skip comments
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()

        # Read dimensions
        width, height = map(int, line.split())

        # Read max value
        max_val = int(f.readline())

        # Read image data
        if magic == b'P5':
            # Binary
            data = np.frombuffer(f.read(), dtype=np.uint8)
        else:
            # ASCII
            data = np.array(f.read().split(), dtype=np.uint8)

        # Reshape
        map_data = data.reshape((height, width))

    return map_data, map_info


def analyze_map(map_data, map_info):
    """Analyze map quality and return metrics"""

    # Convert to occupancy values
    # PGM: 0-254 scale, where 0=black (occupied), 254=white (free), 205=unknown
    free_thresh = map_info.get('free_thresh', 0.196)
    occupied_thresh = map_info.get('occupied_thresh', 0.65)

    # Convert thresholds to PGM values
    free_val = int((1.0 - free_thresh) * 254)
    occupied_val = int((1.0 - occupied_thresh) * 254)

    total_cells = map_data.size

    # Count cell types
    free_cells = np.sum(map_data >= free_val)
    occupied_cells = np.sum(map_data <= occupied_val)
    unknown_cells = np.sum((map_data > occupied_val) & (map_data < free_val))

    # Percentages
    free_pct = (free_cells / total_cells) * 100
    occupied_pct = (occupied_cells / total_cells) * 100
    unknown_pct = (unknown_cells / total_cells) * 100

    # Additional metrics
    resolution = map_info['resolution']
    width_m = map_data.shape[1] * resolution
    height_m = map_data.shape[0] * resolution
    area_m2 = width_m * height_m
    explored_area_m2 = (free_cells + occupied_cells) * (resolution ** 2)

    metrics = {
        'total_cells': total_cells,
        'free_cells': free_cells,
        'occupied_cells': occupied_cells,
        'unknown_cells': unknown_cells,
        'free_percentage': free_pct,
        'occupied_percentage': occupied_pct,
        'unknown_percentage': unknown_pct,
        'resolution': resolution,
        'width_cells': map_data.shape[1],
        'height_cells': map_data.shape[0],
        'width_meters': width_m,
        'height_meters': height_m,
        'total_area_m2': area_m2,
        'explored_area_m2': explored_area_m2,
        'exploration_percentage': (explored_area_m2 / area_m2) * 100,
    }

    return metrics


def assess_quality(metrics):
    """Assess map quality and return score/issues"""
    issues = []
    score = 100.0

    # Check unknown percentage
    if metrics['unknown_percentage'] > 50:
        issues.append("Very high unknown space (>50%) - poor coverage")
        score -= 30
    elif metrics['unknown_percentage'] > 30:
        issues.append("High unknown space (>30%) - limited coverage")
        score -= 15
    elif metrics['unknown_percentage'] < 10:
        issues.append("Excellent coverage (<10% unknown)")

    # Check occupied percentage
    if metrics['occupied_percentage'] < 1:
        issues.append("Very low obstacle density (<1%) - may be empty space or SLAM failure")
        score -= 10
    elif metrics['occupied_percentage'] > 40:
        issues.append("Very high obstacle density (>40%) - unusual, check data")
        score -= 10
    elif 5 < metrics['occupied_percentage'] < 20:
        issues.append("Normal obstacle density (5-20%)")

    # Check resolution
    if metrics['resolution'] > 0.1:
        issues.append(f"Low resolution ({metrics['resolution']}m) - consider finer resolution")
        score -= 5
    elif metrics['resolution'] < 0.02:
        issues.append(f"Very high resolution ({metrics['resolution']}m) - may be overkill")

    # Check map size
    if metrics['total_area_m2'] < 10:
        issues.append("Small map area (<10 m²) - very limited environment")
        score -= 5
    elif metrics['total_area_m2'] > 1000:
        issues.append("Large map area (>1000 m²) - good coverage")

    score = max(0, min(100, score))

    return score, issues


def print_report(metrics, score, issues):
    """Print formatted analysis report"""
    print("\n" + "=" * 70)
    print(" SLAM MAP QUALITY ANALYSIS REPORT")
    print("=" * 70)

    print("\n--- Map Dimensions ---")
    print(f"  Size (cells):     {metrics['width_cells']} x {metrics['height_cells']}")
    print(f"  Size (meters):    {metrics['width_meters']:.2f} x {metrics['height_meters']:.2f}")
    print(f"  Resolution:       {metrics['resolution']} m/cell")
    print(f"  Total area:       {metrics['total_area_m2']:.2f} m²")

    print("\n--- Map Coverage ---")
    print(f"  Total cells:      {metrics['total_cells']:,}")
    print(f"  Free cells:       {metrics['free_cells']:,} ({metrics['free_percentage']:.1f}%)")
    print(f"  Occupied cells:   {metrics['occupied_cells']:,} ({metrics['occupied_percentage']:.1f}%)")
    print(f"  Unknown cells:    {metrics['unknown_cells']:,} ({metrics['unknown_percentage']:.1f}%)")
    print(f"  Explored area:    {metrics['explored_area_m2']:.2f} m² ({metrics['exploration_percentage']:.1f}%)")

    print("\n--- Quality Assessment ---")
    print(f"  Overall Score:    {score:.1f}/100")

    if score >= 80:
        rating = "Excellent"
        symbol = "✓✓✓"
    elif score >= 60:
        rating = "Good"
        symbol = "✓✓"
    elif score >= 40:
        rating = "Fair"
        symbol = "✓"
    else:
        rating = "Poor"
        symbol = "✗"

    print(f"  Quality Rating:   {rating} {symbol}")

    if issues:
        print("\n--- Issues and Observations ---")
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")

    print("\n" + "=" * 70)


def save_report(metrics, score, issues, output_file):
    """Save report to YAML file"""
    report = {
        'analysis_timestamp': str(Path().resolve()),
        'metrics': metrics,
        'quality_score': float(score),
        'issues': issues,
    }

    with open(output_file, 'w') as f:
        yaml.dump(report, f, default_flow_style=False)

    print(f"\nReport saved to: {output_file}")


def analyze_live_map():
    """Subscribe to /map topic and analyze in real-time"""
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import OccupancyGrid
    except ImportError:
        print("Error: rclpy not available. Install ROS2 Python packages.")
        sys.exit(1)

    class MapAnalyzer(Node):
        def __init__(self):
            super().__init__('map_analyzer')
            self.subscription = self.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                10
            )
            self.map_received = False

        def map_callback(self, msg):
            print("\nReceived map, analyzing...")

            # Convert OccupancyGrid to numpy array
            width = msg.info.width
            height = msg.info.height
            map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Convert from occupancy grid [-1, 0-100] to PGM-like [0-254]
            # -1 = unknown (205), 0 = free (254), 100 = occupied (0)
            pgm_data = np.zeros_like(map_data, dtype=np.uint8)
            pgm_data[map_data == -1] = 205  # Unknown
            pgm_data[map_data >= 0] = 254 - (map_data[map_data >= 0] * 254 // 100)

            map_info = {
                'resolution': msg.info.resolution,
                'free_thresh': 0.196,
                'occupied_thresh': 0.65,
            }

            # Analyze
            metrics = analyze_map(pgm_data, map_info)
            score, issues = assess_quality(metrics)
            print_report(metrics, score, issues)

            self.map_received = True

    rclpy.init()
    node = MapAnalyzer()

    print("Waiting for /map topic...")
    print("(Make sure SLAM or map_server is publishing)")

    import time
    start_time = time.time()
    timeout = 30.0

    while not node.map_received and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=1.0)

    if not node.map_received:
        print("\nTimeout: No map received after 30 seconds")
        print("Check that /map topic is being published:")
        print("  ros2 topic list | grep /map")

    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze SLAM map quality',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Analyze saved map
  python3 analyze_slam_quality.py my_map.yaml

  # Analyze live map from /map topic
  python3 analyze_slam_quality.py --live

  # Analyze and save report
  python3 analyze_slam_quality.py my_map.yaml --output report.yaml
        """
    )

    parser.add_argument('map_file', nargs='?', help='Map YAML file to analyze')
    parser.add_argument('--live', action='store_true', help='Subscribe to /map topic')
    parser.add_argument('--output', '-o', help='Save report to file')

    args = parser.parse_args()

    if args.live:
        analyze_live_map()
    elif args.map_file:
        # Analyze file
        if not Path(args.map_file).exists():
            print(f"Error: Map file not found: {args.map_file}")
            sys.exit(1)

        print(f"Analyzing map: {args.map_file}")
        map_data, map_info = load_map_from_file(args.map_file)
        metrics = analyze_map(map_data, map_info)
        score, issues = assess_quality(metrics)
        print_report(metrics, score, issues)

        if args.output:
            save_report(metrics, score, issues, args.output)

    else:
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()
