#!/usr/bin/env python3
"""
LiDAR Scan Quality Analyzer for ROS2 Bag Files

Analyzes recorded LiDAR scan data from rosbag2 files to assess quality
for SLAM applications. Provides detailed statistics and quality warnings.

Usage:
    python3 check_lidar_quality.py <bag_path> [options]

Options:
    --topic TOPIC        Topic name to analyze (default: /scan)
    --max-samples N      Maximum number of scans to analyze (default: all)
    --output FILE        Save detailed report to file
    --plot              Generate quality plots (requires matplotlib)
    --verbose           Show detailed output

Examples:
    python3 check_lidar_quality.py my_session
    python3 check_lidar_quality.py my_session --max-samples 100
    python3 check_lidar_quality.py my_session --output report.txt
    python3 check_lidar_quality.py my_session --plot
"""

import argparse
import sys
import numpy as np
from pathlib import Path
from collections import defaultdict
import time

# ROS2 imports
try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
except ImportError as e:
    print(f"ERROR: Failed to import ROS2 modules: {e}")
    print("Make sure ROS2 is installed and sourced:")
    print("  source /opt/ros/humble/setup.bash")
    sys.exit(1)


class Colors:
    """ANSI color codes for terminal output"""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    BOLD = '\033[1m'
    NC = '\033[0m'  # No Color


class LiDARQualityAnalyzer:
    """Analyzes LiDAR scan quality from ROS2 bag files"""

    def __init__(self, bag_path, topic='/scan', max_samples=None, verbose=False):
        self.bag_path = Path(bag_path)
        self.topic = topic
        self.max_samples = max_samples
        self.verbose = verbose

        # Statistics storage
        self.scans_analyzed = 0
        self.scan_times = []
        self.valid_percentages = []
        self.point_counts = []
        self.range_data = []
        self.intensity_data = []

        # Range categories
        self.zero_ranges = []
        self.inf_ranges = []
        self.nan_ranges = []
        self.min_ranges = []
        self.max_ranges = []

        # Quality metrics
        self.scan_rate_issues = 0
        self.low_validity_scans = 0
        self.low_point_scans = 0

    def analyze_bag(self):
        """Main analysis function"""
        print(f"{Colors.BLUE}========================================{Colors.NC}")
        print(f"{Colors.BLUE}  LiDAR Quality Analyzer{Colors.NC}")
        print(f"{Colors.BLUE}========================================{Colors.NC}")
        print()

        if not self.bag_path.exists():
            print(f"{Colors.RED}ERROR: Bag not found: {self.bag_path}{Colors.NC}")
            return False

        print(f"Bag: {Colors.CYAN}{self.bag_path}{Colors.NC}")
        print(f"Topic: {Colors.CYAN}{self.topic}{Colors.NC}")
        print()

        # Open bag
        try:
            storage_options = rosbag2_py.StorageOptions(
                uri=str(self.bag_path),
                storage_id='sqlite3'
            )
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)

            # Get bag metadata
            metadata = reader.get_metadata()
            print(f"{Colors.BLUE}Bag Information:{Colors.NC}")
            print(f"  Duration: {metadata.duration.nanoseconds / 1e9:.1f} seconds")
            print(f"  Messages: {metadata.message_count}")

            # Find scan topic
            topic_found = False
            for topic_metadata in metadata.topics_with_message_count:
                if topic_metadata.topic_metadata.name == self.topic:
                    topic_found = True
                    scan_count = topic_metadata.message_count
                    print(f"  Scan messages: {scan_count}")
                    break

            if not topic_found:
                print(f"{Colors.RED}ERROR: Topic {self.topic} not found in bag{Colors.NC}")
                print(f"{Colors.YELLOW}Available topics:{Colors.NC}")
                for topic_metadata in metadata.topics_with_message_count:
                    print(f"  - {topic_metadata.topic_metadata.name}")
                return False

            print()
            print(f"{Colors.YELLOW}Analyzing scans...{Colors.NC}")

            # Process messages
            start_time = time.time()
            last_scan_time = None

            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()

                if topic == self.topic:
                    # Deserialize message
                    msg_type = get_message('sensor_msgs/msg/LaserScan')
                    scan_msg = deserialize_message(data, msg_type)

                    # Analyze scan
                    self._analyze_scan(scan_msg, timestamp)

                    # Track scan timing
                    if last_scan_time is not None:
                        scan_interval = (timestamp - last_scan_time) / 1e9
                        self.scan_times.append(scan_interval)
                    last_scan_time = timestamp

                    # Progress indicator
                    if self.verbose and self.scans_analyzed % 10 == 0:
                        print(f"  Processed {self.scans_analyzed} scans...", end='\r')

                    # Check max samples
                    if self.max_samples and self.scans_analyzed >= self.max_samples:
                        break

            elapsed_time = time.time() - start_time

            print(f"{Colors.GREEN}Analyzed {self.scans_analyzed} scans in {elapsed_time:.2f} seconds{Colors.NC}")
            print()

            # Generate report
            self._generate_report()

            return True

        except Exception as e:
            print(f"{Colors.RED}ERROR: Failed to analyze bag: {e}{Colors.NC}")
            import traceback
            traceback.print_exc()
            return False

    def _analyze_scan(self, scan_msg, timestamp):
        """Analyze a single LaserScan message"""
        self.scans_analyzed += 1

        # Convert ranges to numpy array
        ranges = np.array(scan_msg.ranges)
        intensities = np.array(scan_msg.intensities) if scan_msg.intensities else None

        # Analyze range validity
        valid_mask = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        valid_mask &= np.isfinite(ranges)  # Exclude inf and nan

        valid_count = np.sum(valid_mask)
        total_points = len(ranges)
        valid_percentage = (valid_count / total_points * 100) if total_points > 0 else 0

        # Store statistics
        self.valid_percentages.append(valid_percentage)
        self.point_counts.append(total_points)

        if valid_count > 0:
            valid_ranges = ranges[valid_mask]
            self.range_data.extend(valid_ranges.tolist())

            if intensities is not None and len(intensities) > 0:
                valid_intensities = intensities[valid_mask]
                self.intensity_data.extend(valid_intensities.tolist())

        # Count invalid types
        zero_count = np.sum(ranges == 0)
        inf_count = np.sum(np.isinf(ranges))
        nan_count = np.sum(np.isnan(ranges))
        min_count = np.sum((ranges > 0) & (ranges < scan_msg.range_min))
        max_count = np.sum(ranges > scan_msg.range_max)

        self.zero_ranges.append(zero_count)
        self.inf_ranges.append(inf_count)
        self.nan_ranges.append(nan_count)
        self.min_ranges.append(min_count)
        self.max_ranges.append(max_count)

        # Quality checks
        if valid_percentage < 80:
            self.low_validity_scans += 1

        if total_points < 300:
            self.low_point_scans += 1

    def _generate_report(self):
        """Generate quality report"""
        print(f"{Colors.BOLD}========================================{Colors.NC}")
        print(f"{Colors.BOLD}  Quality Analysis Report{Colors.NC}")
        print(f"{Colors.BOLD}========================================{Colors.NC}")
        print()

        if self.scans_analyzed == 0:
            print(f"{Colors.RED}No scans analyzed!{Colors.NC}")
            return

        # Convert to numpy arrays for statistics
        valid_pct = np.array(self.valid_percentages)
        point_counts = np.array(self.point_counts)
        scan_times = np.array(self.scan_times) if self.scan_times else np.array([])

        # Basic statistics
        print(f"{Colors.BLUE}Scan Statistics:{Colors.NC}")
        print(f"  Total scans analyzed: {self.scans_analyzed}")
        print(f"  Average points per scan: {point_counts.mean():.0f}")
        print(f"  Points per scan range: {point_counts.min():.0f} - {point_counts.max():.0f}")
        print()

        # Scan rate analysis
        if len(scan_times) > 0:
            scan_rate = 1.0 / scan_times.mean() if scan_times.mean() > 0 else 0
            scan_rate_std = np.std(1.0 / scan_times) if len(scan_times) > 1 else 0

            print(f"{Colors.BLUE}Scan Rate:{Colors.NC}")
            print(f"  Average: {scan_rate:.2f} Hz")
            print(f"  Std deviation: {scan_rate_std:.2f} Hz")
            print(f"  Expected: ~10 Hz for RP LIDAR C1M1")

            # Quality check
            if scan_rate < 9.5 or scan_rate > 10.5:
                print(f"  {Colors.YELLOW}WARNING: Scan rate outside expected range{Colors.NC}")
                self.scan_rate_issues += 1
            else:
                print(f"  {Colors.GREEN}OK{Colors.NC}")
            print()

        # Range validity
        print(f"{Colors.BLUE}Range Validity:{Colors.NC}")
        print(f"  Average valid ranges: {valid_pct.mean():.1f}%")
        print(f"  Min valid ranges: {valid_pct.min():.1f}%")
        print(f"  Max valid ranges: {valid_pct.max():.1f}%")

        if valid_pct.mean() >= 90:
            print(f"  {Colors.GREEN}EXCELLENT - High data quality{Colors.NC}")
        elif valid_pct.mean() >= 80:
            print(f"  {Colors.YELLOW}GOOD - Acceptable quality{Colors.NC}")
        else:
            print(f"  {Colors.RED}POOR - Low data quality{Colors.NC}")
        print()

        # Invalid range breakdown
        print(f"{Colors.BLUE}Invalid Range Breakdown (average per scan):{Colors.NC}")
        print(f"  Zero ranges: {np.mean(self.zero_ranges):.1f}")
        print(f"  Inf ranges: {np.mean(self.inf_ranges):.1f}")
        print(f"  NaN ranges: {np.mean(self.nan_ranges):.1f}")
        print(f"  Below min range: {np.mean(self.min_ranges):.1f}")
        print(f"  Above max range: {np.mean(self.max_ranges):.1f}")
        print()

        # Range distribution
        if self.range_data:
            range_array = np.array(self.range_data)
            print(f"{Colors.BLUE}Range Distribution:{Colors.NC}")
            print(f"  Mean range: {range_array.mean():.2f} m")
            print(f"  Median range: {np.median(range_array):.2f} m")
            print(f"  Std deviation: {range_array.std():.2f} m")
            print(f"  Min range: {range_array.min():.2f} m")
            print(f"  Max range: {range_array.max():.2f} m")

            # Range histogram
            print()
            print(f"  Range histogram:")
            bins = [0, 1, 2, 4, 6, 8, 10, 12]
            hist, _ = np.histogram(range_array, bins=bins)
            for i in range(len(bins) - 1):
                pct = hist[i] / len(range_array) * 100
                bar = '#' * int(pct / 2)
                print(f"    {bins[i]:2.0f}-{bins[i+1]:2.0f}m: {bar:30s} {pct:5.1f}%")
            print()

        # Intensity data
        if self.intensity_data:
            intensity_array = np.array(self.intensity_data)
            print(f"{Colors.BLUE}Intensity Statistics:{Colors.NC}")
            print(f"  Mean intensity: {intensity_array.mean():.1f}")
            print(f"  Std deviation: {intensity_array.std():.1f}")
            print(f"  Range: {intensity_array.min():.1f} - {intensity_array.max():.1f}")
            print()

        # Quality warnings
        print(f"{Colors.BOLD}Quality Assessment:{Colors.NC}")
        warnings = []
        recommendations = []

        if self.scan_rate_issues > 0:
            warnings.append("Inconsistent scan rate detected")
            recommendations.append("Check LiDAR connection and CPU usage")

        if self.low_validity_scans > self.scans_analyzed * 0.1:
            warnings.append(f"{self.low_validity_scans} scans with <80% valid ranges")
            recommendations.append("Check for obstacles too close/far, or reflective surfaces")

        if self.low_point_scans > self.scans_analyzed * 0.1:
            warnings.append(f"{self.low_point_scans} scans with <300 points")
            recommendations.append("Check LiDAR mode (use DenseBoost) and mounting")

        if np.mean(self.inf_ranges) > point_counts.mean() * 0.4:
            warnings.append("High number of max-range readings (>40%)")
            recommendations.append("May indicate open spaces (normal) or sensor issue")

        if warnings:
            print(f"{Colors.YELLOW}Warnings:{Colors.NC}")
            for i, warning in enumerate(warnings, 1):
                print(f"  {i}. {warning}")
            print()

            print(f"{Colors.CYAN}Recommendations:{Colors.NC}")
            for i, rec in enumerate(recommendations, 1):
                print(f"  {i}. {rec}")
            print()

        # Overall quality score
        score = 100
        if valid_pct.mean() < 90:
            score -= 20
        if valid_pct.mean() < 80:
            score -= 20
        if self.scan_rate_issues > 0:
            score -= 15
        if self.low_point_scans > self.scans_analyzed * 0.1:
            score -= 15
        if self.low_validity_scans > self.scans_analyzed * 0.1:
            score -= 15

        score = max(0, score)

        print(f"{Colors.BOLD}Overall Quality Score: ", end='')
        if score >= 85:
            print(f"{Colors.GREEN}{score}/100 - EXCELLENT{Colors.NC}")
        elif score >= 70:
            print(f"{Colors.YELLOW}{score}/100 - GOOD{Colors.NC}")
        elif score >= 50:
            print(f"{Colors.YELLOW}{score}/100 - ACCEPTABLE{Colors.NC}")
        else:
            print(f"{Colors.RED}{score}/100 - POOR{Colors.NC}")

        print()

        # Usage recommendations
        print(f"{Colors.BLUE}SLAM Usage Recommendations:{Colors.NC}")
        if score >= 85:
            print("  - Excellent quality for SLAM")
            print("  - Suitable for high-accuracy mapping")
            print("  - Good for algorithm parameter tuning")
        elif score >= 70:
            print("  - Good quality for SLAM")
            print("  - Suitable for general mapping tasks")
            print("  - May need minor parameter adjustments")
        elif score >= 50:
            print("  - Acceptable for basic SLAM")
            print("  - May have some drift or artifacts")
            print("  - Consider improving data collection")
        else:
            print("  - Poor quality for reliable SLAM")
            print("  - Recommend re-recording with improvements")
            print("  - Address warnings above before mapping")

        print()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze LiDAR scan quality from ROS2 bag files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument('bag_path', help='Path to ROS2 bag file/directory')
    parser.add_argument('--topic', default='/scan', help='Topic to analyze (default: /scan)')
    parser.add_argument('--max-samples', type=int, help='Maximum scans to analyze (default: all)')
    parser.add_argument('--output', help='Save report to file')
    parser.add_argument('--plot', action='store_true', help='Generate quality plots (requires matplotlib)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')

    args = parser.parse_args()

    # Initialize ROS2 (required for message deserialization)
    rclpy.init()

    try:
        # Create analyzer
        analyzer = LiDARQualityAnalyzer(
            bag_path=args.bag_path,
            topic=args.topic,
            max_samples=args.max_samples,
            verbose=args.verbose
        )

        # Run analysis
        success = analyzer.analyze_bag()

        if not success:
            sys.exit(1)

        # TODO: Add plotting if requested and matplotlib available
        if args.plot:
            print(f"{Colors.YELLOW}Note: Plotting not yet implemented{Colors.NC}")

        # TODO: Add file output if requested
        if args.output:
            print(f"{Colors.YELLOW}Note: File output not yet implemented{Colors.NC}")

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
