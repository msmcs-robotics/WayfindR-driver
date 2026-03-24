# AMBOT - Autonomous Mobile Bot

## Overview

AMBOT is an autonomous conversational robot platform built on three independent components:

- **Bootylicious**: LLM inference and RAG knowledge retrieval running on NVIDIA Jetson Orin Nano
- **Locomotion**: Motor control for differential drive robots using L298N, TB6612FNG, or DRV8833 drivers
- **Pathfinder**: LiDAR-based wandering and obstacle avoidance using LD19 LiDAR sensor

## Hardware

### Jetson Orin Nano Developer Kit
- CPU: 6x Cortex-A78AE (aarch64)
- RAM: 7.4 GiB unified memory (shared GPU/CPU)
- GPU: Orin (nvgpu) with CUDA 12.6
- Storage: 113 GB eMMC
- OS: Ubuntu 22.04.5 LTS (JetPack R36.4.4)
- Purpose: Runs LLM inference, RAG system, and future visual person detection

### Raspberry Pi 3B
- CPU: Cortex-A53 (armhf)
- RAM: 906 MB
- OS: Debian 13
- Purpose: Runs motor control, LiDAR navigation, and sensor fusion
- Connected sensors: LD19 LiDAR, EMEET SmartCam S600 USB camera, MPU6050 IMU

## Wandering Behavior

The robot uses a NaturalWanderBehavior algorithm for exploring spaces without SLAM:

1. Bins ~467 raw LiDAR scan points into 36 angular buckets (10 degrees each)
2. Picks top 10 clearance peaks, each at least 30 degrees apart
3. Cycles through targets sequentially (longest, then 2nd, then 3rd...) to avoid ping-ponging
4. Safety zones (STOP/SLOW/WARN) override all navigation decisions
5. Dynamic obstacle detection monitors for sudden changes (people walking by)

This creates natural-looking exploration without needing odometry, SLAM, or waypoints.

## Wandering Demos

| Demo | File | Components | Description |
|------|------|------------|-------------|
| Demo 1 | wandering_demo_1.py | LiDAR + Motors + IMU | Basic obstacle avoidance wandering |
| Demo 2 | wandering_demo_2.py | Camera + LiDAR + Motors | Wandering with face tracking interrupts |
| Demo 3 | wandering_demo_3.py | All + LLM/RAG | Full robot with conversation (future) |

## Communication

The Jetson and Raspberry Pi will communicate over the local network. The protocol (REST API or MQTT) is still being decided. Current development focuses on getting each component working independently.
