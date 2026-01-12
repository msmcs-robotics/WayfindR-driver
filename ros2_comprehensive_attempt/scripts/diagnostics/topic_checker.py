#!/usr/bin/env python3
"""
Topic Echo/Hz Checker

Monitors ROS2 topics for frequency, latency, and message content.

Usage:
    # Check topic frequency
    python3 topic_checker.py /scan --hz

    # Echo topic messages
    python3 topic_checker.py /cmd_vel --echo

    # Check multiple topics
    python3 topic_checker.py /scan /cmd_vel /odom --hz

    # Monitor all critical topics
    python3 topic_checker.py --all
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
import argparse
from collections import deque, defaultdict
from typing import Dict, List
import importlib


class TopicChecker(Node):
    """Monitor ROS2 topics."""

    def __init__(self, topics: List[str], mode: str = 'hz'):
        super().__init__('topic_checker')

        self.mode = mode
        self.topic_stats = {}

        # Initialize stats for each topic
        for topic in topics:
            self.topic_stats[topic] = {
                'count': 0,
                'timestamps': deque(maxlen=100),
                'last_msg': None,
                'subscription': None
            }

        # Subscribe to topics
        for topic in topics:
            self._subscribe_to_topic(topic)

        self.start_time = time.time()
        self.get_logger().info(f"Monitoring {len(topics)} topics in {mode} mode")

    def _subscribe_to_topic(self, topic: str):
        """Subscribe to a topic dynamically."""
        try:
            # Get topic type
            topic_list = self.get_topic_names_and_types()
            topic_type = None

            for name, types in topic_list:
                if name == topic:
                    topic_type = types[0]
                    break

            if not topic_type:
                self.get_logger().error(f"Topic not found: {topic}")
                return

            # Import message type
            parts = topic_type.split('/')
            module_name = f"{parts[0]}.{parts[1]}"
            class_name = parts[2]

            try:
                module = importlib.import_module(module_name)
                msg_class = getattr(module, class_name)
            except (ImportError, AttributeError) as e:
                self.get_logger().error(f"Cannot import {topic_type}: {e}")
                return

            # Create subscription
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            def callback(msg):
                self._topic_callback(topic, msg)

            sub = self.create_subscription(msg_class, topic, callback, qos)
            self.topic_stats[topic]['subscription'] = sub

            self.get_logger().info(f"Subscribed to {topic} ({topic_type})")

        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic}: {e}")

    def _topic_callback(self, topic: str, msg):
        """Handle topic messages."""
        current_time = time.time()

        stats = self.topic_stats[topic]
        stats['count'] += 1
        stats['timestamps'].append(current_time)
        stats['last_msg'] = msg

        # Echo mode: print message
        if self.mode == 'echo':
            self.get_logger().info(f"\n[{topic}] {msg}")

    def calculate_hz(self, topic: str) -> float:
        """Calculate topic frequency."""
        stats = self.topic_stats[topic]
        timestamps = stats['timestamps']

        if len(timestamps) < 2:
            return 0.0

        time_span = timestamps[-1] - timestamps[0]
        if time_span > 0:
            return (len(timestamps) - 1) / time_span
        return 0.0

    def print_stats(self):
        """Print topic statistics."""
        print("\n" + "=" * 80)
        print(f"TOPIC STATISTICS (uptime: {time.time() - self.start_time:.1f}s)")
        print("=" * 80)
        print(f"{'Topic':<30} {'Count':<10} {'Hz':<10} {'Status'}")
        print("-" * 80)

        for topic, stats in self.topic_stats.items():
            count = stats['count']
            hz = self.calculate_hz(topic)

            # Determine status
            if count == 0:
                status = "NO DATA"
            elif hz < 1.0:
                status = "SLOW"
            else:
                status = "OK"

            print(f"{topic:<30} {count:<10} {hz:<10.2f} {status}")

        print("=" * 80)

    def monitor_loop(self):
        """Continuous monitoring loop."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=1.0)

                if self.mode == 'hz':
                    self.print_stats()

        except KeyboardInterrupt:
            print("\nStopped monitoring")


def main():
    parser = argparse.ArgumentParser(description='ROS2 Topic Checker')
    parser.add_argument('topics', nargs='*', help='Topics to monitor')
    parser.add_argument('--hz', action='store_true',
                       help='Display frequency statistics')
    parser.add_argument('--echo', action='store_true',
                       help='Echo topic messages')
    parser.add_argument('--all', action='store_true',
                       help='Monitor all critical topics')

    args = parser.parse_args()

    # Determine mode
    if args.echo:
        mode = 'echo'
    else:
        mode = 'hz'

    # Determine topics
    if args.all:
        topics = ['/scan', '/cmd_vel', '/odom', '/amcl_pose', '/map', '/tf', '/tf_static']
    elif args.topics:
        topics = args.topics
    else:
        parser.print_help()
        return

    rclpy.init()

    try:
        node = TopicChecker(topics, mode)
        node.monitor_loop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
