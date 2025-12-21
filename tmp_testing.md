The Fully Automatic Flow

YOU DO:                           THE SCRIPT DOES:
────────                          ─────────────────
1. Insert SD card          ──→    Auto-detects device
                                  
2. Run: sudo ./bake_auto.sh ──→   - Finds Ubuntu image
                                  - Finds your SSH key
                                  - Flashes image (dd)
                                  - Mounts partitions
                                  - Writes cloud-init config
                                  - Copies ROS2 install scripts
                                  - Safely ejects
                                  
3. Insert SD into Pi       ──→    
4. Power on               ──→    Pi boots, cloud-init runs
5. Wait 25 min            ──→    ROS2 installs automatically
6. SSH in                 ──→    ssh ubuntu@hostname.local
Usage Examples

# FULLY AUTOMATIC - prompts for anything missing
sudo ./bake_auto.sh

# SPECIFY EVERYTHING
sudo ./bake_auto.sh \
    --image ~/Downloads/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
    --device /dev/sdb \
    --hostname robot-01

# WITH WIFI
sudo ./bake_auto.sh \
    --hostname robot-01 \
    --wifi-ssid "MyNetwork" \
    --wifi-pass "MyPassword"
What You Need
Ubuntu Image - Download once:

wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz
SSH Key - Script auto-detects ~/.ssh/id_rsa.pub or generates one
SD Card - 32GB+ recommended