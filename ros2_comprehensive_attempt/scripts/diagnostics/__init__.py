"""
WayfindR Diagnostics and Monitoring Tools

A comprehensive suite of diagnostic and monitoring tools for the WayfindR
autonomous navigation system.

Tools:
------
1. system_diagnostics.py - Real-time system health monitoring
2. monitoring_dashboard.py - Terminal-based visual dashboard
3. tf_tree_visualizer.py - TF transform tree analysis
4. topic_checker.py - ROS2 topic frequency monitoring
5. map_quality_analyzer.py - Occupancy grid map quality analysis
6. localization_quality.py - AMCL localization quality assessment
7. performance_profiler.py - End-to-end performance profiling

Quick Start:
-----------
See QUICKSTART.md for installation and basic usage.

Documentation:
-------------
- README.md - Comprehensive tool documentation
- findings/diagnostics_system.md - Detailed system guide

Version: 1.0
Date: 2026-01-11
"""

__version__ = "1.0.0"
__author__ = "WayfindR Development Team"

__all__ = [
    'system_diagnostics',
    'monitoring_dashboard',
    'tf_tree_visualizer',
    'topic_checker',
    'map_quality_analyzer',
    'localization_quality',
    'performance_profiler',
]
