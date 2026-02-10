Raspberry Pi 3B

user - pi
password - erau
known MAC address WLAN0 - b8:27:eb:5f:11:79
known IP - 10.33.224.1

rsync the ambot folder into the remote user's home directory on the raspberry pi so it would look like ~/ambot/


Jetson Orin Nano Developer Kit

user - georgejetson
password - Ambot
known MAC address WLAN (wlP1p1s0) - f8:3d:c6:57:05:77
known MAC address ETH (enP8p1s0) - 3c:6d:66:b4:7f:ed
known IP - 10.33.255.82
hostname - georgejetson-desktop

System Info (as of 2026-02-10):
- OS: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- Kernel: 5.15.148-tegra aarch64
- Model: NVIDIA Jetson Orin Nano Developer Kit
- CPU: 6x Cortex-A78AE
- RAM: 7.4 GiB total
- Swap: 3.7 GiB
- Disk: 113G total, ~86G available (21% used)
- Docker: pre-installed (docker0 interface visible)

rsync the bootylicious folder into ~/bootylicious/ on the Jetson

SSH from WSL:
  ssh -i ~/.ssh/id_git georgejetson@10.33.255.82
