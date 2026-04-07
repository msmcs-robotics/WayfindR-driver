# Jetson Orin Nano Developer Kit: Flash Recovery Guide

**Date**: 2026-04-07
**Device**: Jetson Orin Nano Developer Kit (p3768-0000 carrier + p3767-0005 module, 8GB Orin Nano Super)
**Previous config**: JetPack R36.4.4, Ubuntu 22.04.5, CUDA 12.6, cuDNN 9.3.0, TensorRT 10.3.0

---

## Which JetPack Version to Use

**JetPack 6.x (R36.x) is correct for this hardware.** Do NOT use JetPack 5.x (R35.x) -- those are not cross-compatible.

| JetPack   | L4T     | Status                    | Notes                          |
|-----------|---------|---------------------------|--------------------------------|
| 6.2.2     | 36.5    | Latest (Feb 2026)         | Security/bug fixes, CUDA fix   |
| 6.2.1     | 36.4.4  | Previous production        | **This is what we were running** |
| 6.2       | 36.4.3  | Older production           |                                |
| 7.0       | --      | Announced                  | Newer series, verify compat    |

**Recommendation**: Flash **JetPack 6.2.1** to restore the identical environment we had, OR upgrade to **JetPack 6.2.2** for latest fixes. Both use Ubuntu 22.04, CUDA 12.6, cuDNN 9.3.0, TensorRT 10.3.0.

### Component Versions (JetPack 6.2.1)

- Jetson Linux: 36.4.4
- Linux Kernel: 5.15
- Ubuntu: 22.04
- CUDA: 12.6.10
- TensorRT: 10.3.0
- cuDNN: 9.3.0
- VPI: 3.2

---

## Two Flashing Methods

### Method A: SD Card Image (Simpler, no host Ubuntu PC required)

Best if you just want to boot from microSD card.

### Method B: SDK Manager (Recommended for NVMe SSD)

Best if you want to flash directly to NVMe SSD, or want full control. Requires an x86 Ubuntu 20.04/22.04 host PC.

---

## Method A: SD Card Image Flashing

### Download Links

**JetPack 6.2.1 SD Card Image** (matches our previous L4T 36.4.4):
```
https://developer.nvidia.com/downloads/embedded/L4T/r36_Release_v4.4/jp62-r1-orin-nano-sd-card-image.zip
```

**JetPack 5.1.3 SD Card Image** (needed ONLY if firmware is old/pre-36.0):
- Download from: https://developer.nvidia.com/embedded/jetpack-sdk-513
- File: `JP513-orin-nano-sd-card-image_b29.zip`

**Balena Etcher** (for writing images):
```
https://etcher.balena.io/
```

### Step-by-Step: SD Card Method

#### Pre-check: Is firmware already updated?

Since we were already running JetPack R36.4.4, our QSPI firmware should already be at version 36.x. This means we can **skip the firmware update steps** and go straight to flashing JetPack 6.2.1.

If for some reason the QSPI got corrupted, you would need the full firmware update path (see "Firmware Update Path" section below).

#### Steps (assuming firmware is already 36.x)

1. **Download** the JetPack 6.2.1 SD card image (URL above)
2. **Flash the SD card** using Balena Etcher:
   - Insert microSD card (64GB minimum) into your PC
   - Open Balena Etcher
   - Select the downloaded `.zip` file (no need to unzip)
   - Select the SD card drive
   - Click "Flash!" (~15 minutes via USB3)
3. **Insert the SD card** into the Jetson (slot on underside of module)
4. **Connect** display (DisplayPort or DP-to-HDMI), keyboard, mouse
5. **Apply 19V DC power** -- the kit boots automatically
6. **Complete OEM setup** (EULA, language, timezone, user account, network)
7. **Verify firmware update** after first boot:
   ```bash
   sudo systemctl status nv-l4t-bootloader-config
   ```
8. **Reboot** to apply any firmware update:
   ```bash
   sudo reboot
   ```
9. **Set power mode** to MAXN SUPER:
   - Click NVIDIA icon in top-right taskbar
   - Power Mode -> MAXN SUPER

#### Linux Command-Line Flash (alternative to Etcher)

```bash
# Identify the SD card device
dmesg | tail | awk '$3 == "sd" {print}'

# Flash (replace sdX with your device)
unzip -p jp62-r1-orin-nano-sd-card-image.zip | sudo dd of=/dev/sdX bs=1M status=progress

# Eject
sudo eject /dev/sdX
```

---

## Method B: SDK Manager Flashing (NVMe / Full Control)

### Prerequisites

- x86 host PC running Ubuntu 20.04 or 22.04
- NVIDIA Developer account (free)
- USB Type-C to Type-A cable
- 19V DC power supply for Jetson

### Download SDK Manager

```
https://developer.nvidia.com/nvidia-sdk-manager
```

Install the .deb package on your Ubuntu host:
```bash
sudo dpkg -i sdkmanager_*_amd64.deb
sudo apt-get install -f
```

### Step-by-Step: SDK Manager Method

#### 1. Enter Force Recovery Mode

**Button Header J14** is a 12-pin header located on the carrier board, near the edge, under/beside the Jetson module.

**J14 Pin Reference:**
| Pins   | Function           |
|--------|--------------------|
| 3, 4   | UART (TXD/RXD)    |
| 7, 8   | Reset              |
| 9, 10  | Force Recovery (FC_REC + GND) |

**Recovery Mode Procedure:**

1. **Power off** the Jetson completely (unplug DC power)
2. **Place a jumper wire** (female-to-female) between **Pin 9 (FC_REC)** and **Pin 10 (GND)** on the J14 button header
3. **Connect USB-C cable** from the Jetson's USB-C port to your host PC (Type-A end)
4. **Insert DC power** -- the Jetson powers on in Force Recovery Mode
5. **Remove the jumper** from pins 9 and 10 (after power is applied)
6. **Verify** the host PC detects the device:
   ```bash
   lsusb | grep -i nvidia
   ```
   You should see something like `NVIDIA Corp. APX` or similar.

#### 2. Launch SDK Manager

```bash
sdkmanager
```

1. Log in with NVIDIA Developer credentials
2. SDK Manager should **auto-detect** the Jetson in recovery mode
3. If prompted, select: **Jetson Orin Nano [8GB Developer Kit version]**
4. Select JetPack version: **6.2.1** (or 6.2.2 if available)

#### 3. Configure Flash Target

- **Storage**: Select **NVMe** if you have an NVMe SSD installed (recommended), or **microSD** for SD card
- **Components**: Choose full installation or custom (custom is faster if you install extras later via apt)

#### 4. Flash

- Click through to begin flashing
- The process takes 15-30 minutes depending on connection speed
- SDK Manager will flash firmware (QSPI), bootloader, kernel, and rootfs

#### 5. Post-Flash Setup

- After flashing completes, the Jetson reboots into OEM setup
- Complete the setup wizard (user account, timezone, etc.)
- Set power mode to MAXN SUPER

---

## Firmware Update Path (Only if QSPI firmware is old)

**This section is needed ONLY if the board has never run JetPack 6.x before, or if QSPI got corrupted back to factory state.**

Since our board was previously running JetPack R36.4.4, the QSPI firmware should already be up to date. If using SDK Manager, it handles firmware automatically. If using SD card method and firmware is old:

### Full Firmware Update Sequence (SD Card Path)

1. **Check firmware version** -- connect serial console or monitor, press Esc during boot, check UEFI screen (third line from top)

2. If firmware version < 36.0, you need the two-stage update:

   **Stage 1: Flash JetPack 5.1.3 SD card**
   - Write JP513 image to SD card
   - Boot, complete OEM setup
   - Verify: `sudo systemctl status nv-l4t-bootloader-config`
   - Reboot -- firmware updates to 5.0 (35.5.0)

   **Stage 2: Install QSPI updater**
   - Verify: `sudo nvbootctrl dump-slots-info` (should show 35.5.0)
   - Install: `sudo apt-get install nvidia-l4t-jetson-orin-nano-qspi-updater`
   - Reboot -- QSPI content updates
   - Device may hang after update; power cycle

3. **Now flash JetPack 6.2.1 SD card** (the normal procedure above)

4. After first boot with JP 6.2.1, firmware updates to 36.4.4 automatically on reboot.

---

## Storage: SD Card vs NVMe

The Jetson Orin Nano Developer Kit supports both:

- **microSD card**: Slot on underside of module. Simpler setup. Slower I/O.
- **NVMe M.2 SSD**: Slot on underside of carrier board. Much faster. Recommended for production use.

**SD Card method** can only flash to SD card.
**SDK Manager method** can flash to either SD card or NVMe SSD.

For NVMe flashing, NVIDIA uses "initrd flash" -- the official method for Orin Nano/NX series with NVMe as the boot device.

---

## Recovery Mode Quick Reference

```
POWER OFF
  |
  v
Jumper Pin 9 (FC_REC) <---> Pin 10 (GND) on J14 header
  |
  v
Connect USB-C to host PC
  |
  v
Insert DC power (board enters recovery mode)
  |
  v
Remove jumper
  |
  v
Verify: lsusb | grep -i nvidia  (should show APX device)
  |
  v
Flash via SDK Manager or command-line flash.sh
```

---

## Command-Line Flash (Advanced, no SDK Manager)

If you have the L4T BSP downloaded and extracted:

```bash
# On host Ubuntu PC, after entering recovery mode:
cd Linux_for_Tegra/
sudo ./flash.sh jetson-orin-nano-devkit internal
# For NVMe:
sudo ./flash.sh jetson-orin-nano-devkit nvme0n1p1
```

This requires downloading the L4T Driver Package (BSP) and Root Filesystem separately from:
```
https://developer.nvidia.com/embedded/downloads
```

---

## Post-Flash Restoration Checklist (AMBOT-specific)

After reflashing, restore our development environment:

1. Set hostname, create user account (georgejetson / Ambot)
2. Configure WiFi (interface: wlP1p1s0, not wlan0)
3. Set up SSH key auth (`~/.ssh/id_git`)
4. Set NOPASSWD sudo for georgejetson
5. Set power mode to MAXN SUPER
6. Run `install-jetson.sh` from our repo (installs Docker, Ollama, CUDA PATH, etc.)
7. Restore Docker containers (PostgreSQL/pgvector, Redis, FastAPI RAG stack)
8. Fix browser: install `firefox-esr` (snap browsers broken on Tegra kernel)
9. Set Ollama overrides (OLLAMA_HOST, MAX_LOADED_MODELS, FLASH_ATTENTION, KV_CACHE_TYPE)
10. Verify: `http://10.33.155.83:8000/api/health`

---

## Key URLs Summary

| Resource | URL |
|----------|-----|
| JetPack 6.2.1 SD Image | https://developer.nvidia.com/downloads/embedded/L4T/r36_Release_v4.4/jp62-r1-orin-nano-sd-card-image.zip |
| JetPack SDK page (latest) | https://developer.nvidia.com/embedded/jetpack |
| JetPack 6.2.1 page | https://developer.nvidia.com/embedded/jetpack-sdk-621 |
| JetPack 6.2 page | https://developer.nvidia.com/embedded/jetpack-sdk-62 |
| SDK Manager download | https://developer.nvidia.com/nvidia-sdk-manager |
| Jetson Download Center | https://developer.nvidia.com/embedded/downloads |
| Getting Started Guide | https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit |
| Initial Setup (Jetson AI Lab) | https://www.jetson-ai-lab.com/tutorials/initial-setup-jetson-orin-nano/ |
| Balena Etcher | https://etcher.balena.io/ |
| Carrier Board User Guide | https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html |

---

## Sources

- [Jetson Orin Nano Getting Started Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [JetPack 6.2.1 SDK Page](https://developer.nvidia.com/embedded/jetpack-sdk-621)
- [JetPack 6.2 SDK Page](https://developer.nvidia.com/embedded/jetpack-sdk-62)
- [Jetson AI Lab Initial Setup Guide](https://www.jetson-ai-lab.com/tutorials/initial-setup-jetson-orin-nano/)
- [Jetson Orin Nano User Guide - How-to](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html)
- [Jetson Orin Nano User Guide - Software Setup](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/software_setup.html)
- [NVIDIA Forum: Recovery Mode](https://forums.developer.nvidia.com/t/does-the-jetson-orin-nano-have-a-recovery-mode/313786)
- [Cytron: Upgrade to Super via SDK Manager](https://www.cytron.io/tutorial/upgrade-jetson-orin-nano-to-super-using-nvidia-sdk-manager)
- [JetsonHacks: JetPack 6.2.2](https://jetsonhacks.com/2026/02/06/jetpack-6-2-2-for-jetson-orin/)
- [NVIDIA Flashing Support Docs](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/FlashingSupport.html)
