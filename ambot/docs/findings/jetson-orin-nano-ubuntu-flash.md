# Jetson Orin Nano - Ubuntu 22.04 Flash Guide (Windows)

> Date: 2026-01-29
> Status: Complete
> Keywords: jetson orin nano, ubuntu 22.04, jetpack 6, sd card flash, windows, balenaEtcher

## Summary

Step-by-step guide for flashing Ubuntu 22.04 (via JetPack 6.x) to a Jetson Orin Nano Developer Kit SD card from a Windows machine.

---

## Prerequisites

### Hardware Required
- Jetson Orin Nano Developer Kit
- MicroSD card (minimum 32GB, recommended 64GB+ UHS-1 or faster)
- MicroSD card reader for your Windows PC
- Power supply (USB-C, 5V/3A minimum or official NVIDIA power adapter)
- Display (HDMI), keyboard, mouse for initial setup

### Software Required (Windows)
- **balenaEtcher** (recommended) or **Rufus** for flashing
- Web browser for downloading the image

---

## Step 1: Download the JetPack SD Card Image

NVIDIA's JetPack 6.x is based on **Ubuntu 22.04 LTS** and includes:
- CUDA, cuDNN, TensorRT
- Jetson Linux (L4T)
- Development tools and samples

### Download Options

**Option A: Direct Download (Recommended)**
1. Go to: https://developer.nvidia.com/embedded/jetpack
2. Look for **"JetPack 6.x SD Card Image"** for Jetson Orin Nano Developer Kit
3. You may need a free NVIDIA Developer account to download
4. The file will be named something like: `jetson-orin-nano-devkit-sd-card-image.zip`
5. File size is approximately **15-18GB**

**Option B: SDK Manager (Linux only)**
- Not available for Windows - skip this option

### Important Notes
- Ensure you download the image specifically for **Jetson Orin Nano Developer Kit**
- The Orin Nano and Orin NX have different images - don't mix them up
- JetPack 6.0+ = Ubuntu 22.04
- JetPack 5.x = Ubuntu 20.04 (older, don't use unless required)

---

## Step 2: Download and Install balenaEtcher

balenaEtcher is a free, cross-platform tool for flashing SD cards.

1. Go to: https://etcher.balena.io/
2. Download the Windows installer
3. Install and launch balenaEtcher

**Alternative: Rufus**
- Download from: https://rufus.ie/
- Works similarly but has more options (can be confusing)
- balenaEtcher is simpler for this task

---

## Step 3: Prepare the SD Card

1. **Insert the microSD card** into your card reader
2. **Backup any existing data** - the card will be completely wiped
3. Open **File Explorer** and note the drive letter (e.g., `D:`, `E:`)

### SD Card Recommendations
| Size | Use Case |
|------|----------|
| 32GB | Minimum, will be tight for development |
| 64GB | Recommended for general use |
| 128GB+ | Recommended if storing LLM models, datasets |

**Speed class matters**: Use UHS-I (U1/U3) or faster for reasonable performance.

---

## Step 4: Flash the Image

### Using balenaEtcher

1. **Launch balenaEtcher**

2. **Select Image**
   - Click "Flash from file"
   - Navigate to your downloaded `.zip` file (no need to extract!)
   - balenaEtcher can flash directly from `.zip`

3. **Select Target**
   - Click "Select target"
   - Choose your microSD card (verify by size to avoid selecting wrong drive!)
   - **WARNING**: Triple-check you selected the SD card, not your system drive

4. **Flash**
   - Click "Flash!"
   - Enter admin credentials if prompted
   - Wait for the process to complete (15-30 minutes depending on SD card speed)
   - balenaEtcher will verify the flash automatically

5. **Completion**
   - You'll see "Flash Complete!" when done
   - Windows may pop up "You need to format this disk" - **click Cancel**
   - The SD card now contains a Linux filesystem Windows can't read

6. **Eject the SD card** safely via Windows taskbar

---

## Step 5: First Boot Setup

### Physical Setup
1. **Insert the flashed SD card** into the Jetson Orin Nano (slot on carrier board)
2. **Connect peripherals:**
   - HDMI display
   - USB keyboard and mouse
   - Ethernet cable (recommended for updates) OR prepare WiFi credentials
3. **Connect power** (USB-C or barrel jack depending on your kit version)

### Initial Configuration (OEM Setup)
The Jetson will boot into Ubuntu's first-time setup wizard:

1. **Accept NVIDIA EULA and License Agreement**

2. **Select Language and Keyboard Layout**
   - English (US) recommended for development

3. **Select Time Zone**
   - Choose your location

4. **Create User Account**
   ```
   Suggested for Ambot project:
   Username: ambot
   Computer name: ambot-jetson
   Password: [choose a strong password]
   ```

5. **Configure APP Partition Size**
   - Use the maximum available (default)

6. **Select Nvpmodel Mode**
   - 15W mode (MAXN) for maximum performance
   - 7W mode for lower power consumption
   - Can be changed later via `nvpmodel` command

7. **Wait for Setup to Complete**
   - System will configure and reboot
   - This may take 5-10 minutes

---

## Step 6: Post-Installation Verification

Once logged in, verify the system:

### Check Ubuntu Version
```bash
lsb_release -a
# Should show: Ubuntu 22.04.x LTS
```

### Check L4T (Linux for Tegra) Version
```bash
cat /etc/nv_tegra_release
# Shows the JetPack/L4T version
```

### Check CUDA Installation
```bash
nvcc --version
# Shows CUDA version (should be 12.x for JetPack 6)
```

### Check Available Memory
```bash
free -h
# Should show approximately 8GB total
```

### Check GPU
```bash
sudo tegrastats
# Shows real-time GPU/CPU usage, temperature
# Press Ctrl+C to exit
```

---

## Step 7: Essential First Steps

### Update the System
```bash
sudo apt update && sudo apt upgrade -y
```

### Enable SSH (for remote access)
```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
```

### Get IP Address (for SSH access)
```bash
ip addr show
# Look for inet address on eth0 (wired) or wlan0 (WiFi)
```

### Install Basic Development Tools
```bash
sudo apt install -y git curl wget htop nano
```

---

## Troubleshooting

### Jetson Won't Boot
- Ensure SD card is fully inserted
- Try re-flashing the SD card
- Use a different SD card (some cards are incompatible)
- Check power supply provides adequate power

### No Display Output
- Try a different HDMI cable
- Try a different display
- Some displays don't work before OS loads - wait a few minutes

### "Disk needs to be formatted" on Windows After Flash
- This is normal - Linux partitions aren't readable by Windows
- Click Cancel/Ignore

### Flash Verification Failed
- Try a different SD card (card may be corrupted or too slow)
- Re-download the image (download may have been corrupted)
- Try Rufus instead of balenaEtcher

### Slow Performance
- Check you're in 15W mode: `sudo nvpmodel -q`
- Switch to 15W mode: `sudo nvpmodel -m 0`
- Ensure adequate cooling (fan should spin)

---

## Next Steps for Ambot Project

After successful setup:

1. **Configure static IP or note the DHCP IP** for SSH access
2. **Set up SSH keys** for passwordless login from development machine
3. **Install Docker** for the RAG stack
4. **Clone the Ambot project** repository
5. **Proceed with nanoLLM installation** per the roadmap

See: [jetson_nano/README.md](../../jetson_nano/README.md) for project-specific configuration.

---

## Related

**Project Documents:**
- [initial-system-research.md](./initial-system-research.md) - Resource planning research
- [../roadmap.md](../roadmap.md) - Project milestones

**External Resources:**
- [NVIDIA JetPack Downloads](https://developer.nvidia.com/embedded/jetpack)
- [Jetson Orin Nano Developer Kit User Guide](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/index.html)
- [balenaEtcher](https://etcher.balena.io/)
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)

---

## Notes

- JetPack 6.x is recommended for Ubuntu 22.04 compatibility
- The 8GB unified RAM is shared between CPU and GPU - monitor closely
- NVMe SSD can be used instead of SD card for better performance (requires additional hardware setup)
- First boot takes longer than subsequent boots due to initial configuration

---

*Document created: 2026-01-29*
