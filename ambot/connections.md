Raspberry Pi 3B (Primary RPi)

user - pi
password - erau
known MAC address WLAN0 - b8:27:eb:5f:11:79
known IP - 10.33.224.1

rsync the ambot folder into the remote user's home directory on the raspberry pi so it would look like ~/ambot/

Hardware Status (as of Session 22):
- Camera: EMEET SmartCam S600, /dev/video0, 640x480, working
- LiDAR: LD19, USB, 230400 baud, ~467 pts/scan, working
- Motors: L298N — RIGHT working, LEFT not spinning (wiring/power issue)
- IMU: MPU6050 driver ready, needs hardware wiring + test


Raspberry Pi 3B (Second RPi - Adeept HAT, ABANDONED)

user - erauieee (also ieeeerau)
known IP - 10.33.214.246
Status: Motor testing paused, Adeept Robot HAT abandoned in favor of L298N+Jetson approach


Jetson Orin Nano Developer Kit

user - georgejetson
password - Ambot
known MAC address WLAN (wlP1p1s0) - f8:3d:c6:57:05:77
known MAC address ETH (enP8p1s0) - 3c:6d:66:b4:7f:ed
known IP - 10.33.155.83
hostname - georgejetson-desktop

SSH from WSL:
  ssh jetson                (uses ~/.ssh/id_git, configured in ~/.ssh/config)
  ssh -i ~/.ssh/id_git georgejetson@10.33.155.83   (manual equivalent)

rsync the bootylicious folder into ~/bootylicious/ on the Jetson

System Info (as of 2026-02-10):
- OS: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- Kernel: 5.15.148-tegra aarch64
- JetPack: R36.4.4
- Model: NVIDIA Jetson Orin Nano Developer Kit
- CPU: 6x Cortex-A78AE
- RAM: 7.4 GiB total
- Swap: 3.7 GiB
- Disk: 113G total, ~86G available (21% used)
- GPU: Orin (nvgpu), Compute 8.7 (Ampere), CUDA 12.6, Driver 540.4.0
- Docker: 28.2.2, Compose v2.36.2, NVIDIA Container Toolkit 1.16.2

Sensors on Jetson (as of Session 22):
- Camera: EMEET SmartCam S600, /dev/video0, 640x480, working
- LiDAR: LD19, /dev/ttyUSB0, 230400 baud, ~350 pts/scan, working
- Motors: L298N, GPIO pins 7/13/29/31/32/33 (BOARD numbering), confirmed driving, awaiting batteries

Services on Jetson:
- Ollama: localhost:11434
  - Primary model: llama3.2:3b
  - Also available: tinyllama, gemma2:2b, phi3:mini, smollm2:1.7b
  - GPU-accelerated (CUDA0)
  - Override: OLLAMA_HOST=0.0.0.0, OLLAMA_MAX_LOADED_MODELS=1, OLLAMA_FLASH_ATTENTION=1, OLLAMA_KV_CACHE_TYPE=q8_0
- RAG API: localhost:8000
  - Docker stack: PostgreSQL (pgvector:pg16) + Redis (7-alpine) + FastAPI
  - Phase 1-3 complete (junk filtering, text normalization, retry, dual keyword, adaptive weight, DB indexes)
  - Ollama URL from Docker: http://172.18.0.1:11434 (bridge gateway, NOT host.docker.internal)
- Web dashboard: localhost:5000
  - Access from WSL via SSH tunnel (see below)
- MCP ability server: mcp_ability/ (not yet deployed as service)

SSH Port Forwarding:
  ssh -f -N -L 5123:localhost:5000 jetson    (web dashboard -> localhost:5123 on dev machine)
