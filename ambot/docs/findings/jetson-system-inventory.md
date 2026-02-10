# Jetson Orin Nano Developer Kit - System Inventory

> Date: 2026-02-10 (Session 9)
> Status: Complete
> Keywords: jetson orin nano, system inventory, hardware, JetPack 6

## Summary

First boot and inventory of the Jetson Orin Nano Developer Kit. Fresh JetPack R36.4.4 installation with Docker, NVIDIA drivers, and CUDA runtime pre-installed.

---

## Hardware

| Property | Value |
|----------|-------|
| Model | NVIDIA Jetson Orin Nano Developer Kit |
| CPU | 6x ARM Cortex-A78AE (aarch64) |
| RAM | 7.4 GiB unified (shared CPU+GPU) |
| Swap | 3.7 GiB |
| Disk | 113G eMMC (/dev/mmcblk0p1), 86G available |
| GPU | Orin (nvgpu) — integrated, shares RAM |
| Hostname | georgejetson-desktop |

## Software

| Component | Version | Notes |
|-----------|---------|-------|
| OS | Ubuntu 22.04.5 LTS (Jammy Jellyfish) | |
| Kernel | 5.15.148-tegra | NVIDIA L4T kernel |
| JetPack | R36 rev 4.4 (L4T 36.4.4) | Flashed June 2025 |
| NVIDIA Driver | 540.4.0 | |
| CUDA Runtime | 12.6 (via nvidia-smi) | nvcc NOT installed (no dev headers) |
| Docker | 28.2.2 | Pre-installed, service active |
| Docker Compose | v2.36.2 | Plugin installed |
| NVIDIA Container Toolkit | 1.16.2-1 | Installed |
| Python | 3.10.12 | System Python |
| pip | NOT installed | Needs `apt install python3-pip` |
| Ollama | NOT installed | Need to install |

## Network Interfaces

| Interface | MAC Address | Status |
|-----------|-------------|--------|
| wlP1p1s0 (WiFi) | f8:3d:c6:57:05:77 | UP |
| enP8p1s0 (Ethernet) | 3c:6d:66:b4:7f:ed | DOWN (no cable) |
| docker0 | 32:6a:a9:49:79:2b | DOWN (no containers) |
| l4tbr0 | 3e:c7:e7:79:f3:87 | DOWN |
| usb0 | 56:cb:d5:6b:f4:3d | DOWN |
| usb1 | 56:cb:d5:6b:f4:3f | DOWN |
| can0 | — | DOWN (CAN bus) |

## NVIDIA Packages Installed

Key packages (all L4T 36.4.4):
- `nvidia-l4t-core`, `nvidia-l4t-cuda`, `nvidia-l4t-cuda-utils`
- `nvidia-l4t-firmware`, `nvidia-l4t-kernel`
- `nvidia-l4t-camera`, `nvidia-l4t-gstreamer`
- `nvidia-l4t-jetson-multimedia-api`
- `nvidia-l4t-dla-compiler` (Deep Learning Accelerator)
- `nvidia-container-toolkit` 1.16.2
- `cuda-nvtx-12-6`, `cuda-nsight-compute-12-6`

## User Setup

- User: `georgejetson`
- Groups: adm, cdrom, sudo, audio, dip, video, plugdev, render, i2c, lpadmin, gdm, sambashare, weston-launch, gpio
- **NOT in docker group** (needs `usermod -aG docker georgejetson`)
- Passwordless sudo: configured via `/etc/sudoers.d/georgejetson`

## Key Observations

1. **Docker pre-installed and working** — major time saver vs installing from scratch
2. **NVIDIA Container Toolkit already present** — GPU passthrough to Docker should work
3. **No nvcc** — CUDA development headers not installed, but runtime CUDA 12.6 is available via drivers. Can install `cuda-toolkit-12-6` if needed for building from source
4. **No pip** — simple fix with `apt install python3-pip`
5. **86GB free disk** — plenty of space for Docker images and LLM models
6. **5.5GB available RAM** — enough for tinyllama (~637MB) + RAG stack (~1.5GB) with headroom
7. **JetPack R36.4.4** — this is JetPack 6.x series, recent and well-supported

## What Needs Installing

| Component | Command | Purpose |
|-----------|---------|---------|
| pip | `sudo apt install python3-pip` | Python package management |
| docker group | `sudo usermod -aG docker georgejetson` | Run Docker without sudo |
| Ollama | `curl -fsSL https://ollama.com/install.sh \| sh` | LLM inference server |
| tinyllama model | `ollama pull tinyllama` | First ~1B param test model |

## References

- [Jetson LLM deployment research](jetson-llm-deployment-research.md)
- [Connections info](../../connections.md)
