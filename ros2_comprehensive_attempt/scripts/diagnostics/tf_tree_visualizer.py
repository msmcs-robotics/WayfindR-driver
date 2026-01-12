#!/usr/bin/env python3
"""
TF Tree Visualizer

Visualizes the TF tree structure and checks for issues:
- Missing transforms
- Stale transforms
- Transform cycles
- Disconnected frames

Usage:
    python3 tf_tree_visualizer.py
    python3 tf_tree_visualizer.py --export tree.txt
    python3 tf_tree_visualizer.py --check
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import TransformException

import time
import argparse
from collections import defaultdict, deque
from typing import Dict, List, Set, Tuple, Optional


class TFTreeVisualizer(Node):
    """Visualize and analyze TF tree."""

    def __init__(self):
        super().__init__('tf_tree_visualizer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait for TF data
        self.get_logger().info("Collecting TF data...")
        time.sleep(2.0)

    def get_all_frames(self) -> Set[str]:
        """Get all known frames."""
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames = set()

        for line in frames_yaml.split('\n'):
            if line.startswith('  ') and ':' in line:
                frame = line.strip().split(':')[0].strip('"').strip("'")
                if frame:
                    frames.add(frame)

        return frames

    def build_tree(self) -> Dict[str, List[str]]:
        """Build parent->children tree structure."""
        frames = self.get_all_frames()
        tree = defaultdict(list)
        frame_parents = {}

        for frame in frames:
            # Try to find parent
            try:
                # Get transform to find parent
                frames_yaml = self.tf_buffer.all_frames_as_yaml()
                for line in frames_yaml.split('\n'):
                    if f"'{frame}':" in line or f'"{frame}":' in line:
                        # Next line contains parent info
                        idx = frames_yaml.split('\n').index(line)
                        lines = frames_yaml.split('\n')
                        if idx + 1 < len(lines):
                            parent_line = lines[idx + 1]
                            if 'parent' in parent_line:
                                parent = parent_line.split("'")[1]
                                tree[parent].append(frame)
                                frame_parents[frame] = parent
                                break

            except Exception as e:
                continue

        return tree, frame_parents

    def get_transform_info(self, parent: str, child: str) -> Tuple[str, float, str]:
        """Get transform information."""
        try:
            trans = self.tf_buffer.lookup_transform(
                parent,
                child,
                Time(),
                timeout=Duration(seconds=0.5)
            )

            current_time = time.time()
            stamp_sec = trans.header.stamp.sec + trans.header.stamp.nanosec / 1e9
            age = current_time - stamp_sec

            if age > 5.0:
                status = "STALE"
            elif age > 2.0:
                status = "OLD"
            else:
                status = "OK"

            return status, age, ""

        except TransformException as ex:
            return "ERROR", 0.0, str(ex)

    def print_tree(self, root: str = None, export_file: str = None):
        """Print TF tree in a hierarchical format."""
        tree, frame_parents = self.build_tree()

        # Find root if not specified
        if root is None:
            # Find frames with no parents
            all_frames = self.get_all_frames()
            children = set()
            for kids in tree.values():
                children.update(kids)

            roots = all_frames - children
            if 'map' in roots:
                root = 'map'
            elif roots:
                root = list(roots)[0]
            else:
                root = list(all_frames)[0] if all_frames else None

        if not root:
            self.get_logger().error("No TF frames found")
            return

        output_lines = []
        output_lines.append("=" * 70)
        output_lines.append("TF TREE STRUCTURE")
        output_lines.append("=" * 70)
        output_lines.append("")

        def print_subtree(frame: str, prefix: str = "", is_last: bool = True):
            """Recursively print tree."""
            # Get transform info
            parent = frame_parents.get(frame)
            if parent:
                status, age, error = self.get_transform_info(parent, frame)
                status_str = f"[{status}]"
                if status == "OK":
                    status_str += f" {age:.2f}s"
                elif error:
                    status_str += f" {error[:30]}"
            else:
                status_str = "[ROOT]"

            # Print current frame
            connector = "└── " if is_last else "├── "
            line = f"{prefix}{connector}{frame} {status_str}"
            output_lines.append(line)

            # Print children
            children = tree.get(frame, [])
            for i, child in enumerate(sorted(children)):
                is_last_child = (i == len(children) - 1)
                extension = "    " if is_last else "│   "
                print_subtree(child, prefix + extension, is_last_child)

        print_subtree(root)

        # Print or export
        output = "\n".join(output_lines)
        if export_file:
            with open(export_file, 'w') as f:
                f.write(output)
            self.get_logger().info(f"TF tree exported to {export_file}")
        else:
            print(output)

    def check_health(self) -> bool:
        """Check TF tree health and report issues."""
        print("\n" + "=" * 70)
        print("TF TREE HEALTH CHECK")
        print("=" * 70 + "\n")

        issues_found = False
        tree, frame_parents = self.build_tree()

        # Check for critical transforms
        critical_transforms = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'laser'),
        ]

        print("Checking critical transforms:")
        for parent, child in critical_transforms:
            status, age, error = self.get_transform_info(parent, child)

            if status == "OK":
                print(f"  ✓ {parent} -> {child}: OK ({age:.2f}s)")
            elif status in ["OLD", "STALE"]:
                print(f"  ⚠ {parent} -> {child}: {status} ({age:.2f}s)")
                issues_found = True
            else:
                print(f"  ✗ {parent} -> {child}: ERROR - {error}")
                issues_found = True

        # Check for disconnected frames
        print("\nChecking for disconnected frames:")
        all_frames = self.get_all_frames()
        connected_frames = set(frame_parents.keys())
        connected_frames.update(tree.keys())

        disconnected = all_frames - connected_frames
        if disconnected:
            print(f"  ⚠ Found {len(disconnected)} disconnected frames:")
            for frame in sorted(disconnected):
                print(f"    - {frame}")
            issues_found = True
        else:
            print("  ✓ All frames connected")

        # Check for cycles (should not happen in TF)
        print("\nChecking for cycles:")
        visited = set()
        rec_stack = set()

        def has_cycle(frame):
            visited.add(frame)
            rec_stack.add(frame)

            for child in tree.get(frame, []):
                if child not in visited:
                    if has_cycle(child):
                        return True
                elif child in rec_stack:
                    return True

            rec_stack.remove(frame)
            return False

        # Find roots
        roots = all_frames - set(frame_parents.keys())
        cycle_found = False
        for root in roots:
            if has_cycle(root):
                cycle_found = True
                break

        if cycle_found:
            print("  ✗ Cycle detected in TF tree!")
            issues_found = True
        else:
            print("  ✓ No cycles detected")

        # Summary
        print("\n" + "=" * 70)
        if issues_found:
            print("RESULT: Issues found in TF tree")
        else:
            print("RESULT: TF tree is healthy")
        print("=" * 70)

        return not issues_found

    def watch(self, interval: float = 2.0):
        """Continuously watch TF tree."""
        print("Watching TF tree (Ctrl+C to stop)...")
        try:
            while True:
                self.print_tree()
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nStopped watching")


def main():
    parser = argparse.ArgumentParser(description='TF Tree Visualizer and Health Checker')
    parser.add_argument('--export', '-e', type=str,
                       help='Export tree to file')
    parser.add_argument('--check', '-c', action='store_true',
                       help='Perform health check')
    parser.add_argument('--watch', '-w', action='store_true',
                       help='Continuously watch TF tree')
    parser.add_argument('--root', '-r', type=str,
                       help='Root frame (default: map)')
    parser.add_argument('--interval', '-i', type=float, default=2.0,
                       help='Watch interval in seconds (default: 2.0)')

    args = parser.parse_args()

    rclpy.init()
    node = TFTreeVisualizer()

    try:
        if args.check:
            healthy = node.check_health()
            exit(0 if healthy else 1)
        elif args.watch:
            node.watch(args.interval)
        else:
            node.print_tree(root=args.root, export_file=args.export)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
