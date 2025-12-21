# Research Findings: Raspberry Pi Automation

**Date:** 2025-12-20
**Purpose:** Document learnings from old_bakery analysis and cloud-init research

---

## Key Discovery: Cloud-Init is the Answer

Ubuntu 22.04 for Raspberry Pi comes with **cloud-init pre-installed**. This eliminates the need for complex custom first-boot systems.

### What is Cloud-Init?

Cloud-init is a standard for cloud instance initialization that:
- Runs on first boot only
- Configures users, SSH keys, hostname
- Can install packages
- Can run custom scripts
- Uses simple YAML configuration

### Where are the Files?

On a freshly flashed Ubuntu Pi SD card:

```
system-boot partition (FAT32, ~256MB):
├── user-data          # Cloud-init user configuration
├── network-config     # Network settings
├── config.txt         # Raspberry Pi hardware config
└── cmdline.txt        # Kernel command line

writable partition (ext4, rest of card):
└── (root filesystem)
```

---

## Old Bakery Analysis

### What Was Tried Before

1. **PyQt5 GUI Application** (`bakery_old.py`)
   - Complex, hard to maintain
   - Too many features in one app
   - Required X11/display

2. **Custom First-Boot System** (`firstboot.sh`)
   - State file tracking
   - Script ordering via `runlist.txt`
   - Systemd service
   - **This approach was solid** - we kept it

3. **Pre-baked Images**
   - **ABANDONED** - too fragile across OS versions
   - Can't maintain images for every Ubuntu update

4. **Fleet Management Vision**
   - SSH-based deployment
   - Mini-Ansible approach
   - Never implemented

### What Worked

- Systemd for first-boot (instead of cron)
- State-based script tracking
- Script ordering by filename
- Hostname timestamping for uniqueness

### What Didn't Work

- Pre-baking OS images
- Complex GUI applications
- Cloud-init on Raspberry Pi OS (different from Ubuntu)

---

## Cloud-Init Best Practices

### Enabling SSH

Two methods:

1. **Cloud-init user-data** (recommended):
```yaml
#cloud-config
ssh_pwauth: true  # or false for key-only
users:
  - name: ubuntu
    ssh_authorized_keys:
      - ssh-rsa AAAA...
```

2. **Empty SSH file** (legacy, for Pi OS):
```bash
touch /boot/ssh
```

For Ubuntu 22.04, use method 1.

### Running Scripts on First Boot

**Method 1: runcmd** (runs once during cloud-init):
```yaml
runcmd:
  - apt-get update
  - apt-get install -y nginx
```

**Method 2: write_files + systemd** (runs after cloud-init):
```yaml
write_files:
  - path: /etc/systemd/system/myservice.service
    content: |
      [Unit]
      After=cloud-final.service
      [Service]
      ExecStart=/opt/scripts/setup.sh
      [Install]
      WantedBy=multi-user.target
runcmd:
  - systemctl enable myservice
```

We use **Method 2** because ROS2 installation takes 20+ minutes and we want proper logging.

### Package Installation

**Preferred** - use packages directive:
```yaml
package_update: true
package_upgrade: true
packages:
  - git
  - python3-pip
```

**Alternative** - use runcmd:
```yaml
runcmd:
  - apt-get update
  - apt-get install -y package-name
```

---

## Critical Learnings

### 1. Cloud-Init Only Runs Once

After first boot, cloud-init marks itself complete. To re-run:
```bash
sudo cloud-init clean --logs --reboot
```

### 2. Partition Names Vary

- SD card via USB adapter: `/dev/sdb1`, `/dev/sdb2`
- Built-in SD reader: `/dev/mmcblk0p1`, `/dev/mmcblk0p2`
- NVMe: `/dev/nvme0n1p1`, `/dev/nvme0n1p2`

Script must handle all cases.

### 3. Network Must Be Ready

If scripts need to download packages, they must wait for network:
```ini
[Unit]
After=network-online.target
Wants=network-online.target
```

### 4. APT Lock Handling

Multiple apt processes can conflict. Always wait:
```bash
while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    sleep 5
done
```

### 5. Don't Use /tmp in Cloud-Init

Race condition with `systemd-tmpfiles-clean`. Use `/run/` instead.

---

## Two-Stage Installation Approach

### Stage 1: Cloud-Init (2-3 minutes)

Runs immediately on first boot:
- Create user with SSH key
- Set hostname
- Install basic packages (avahi, curl, git)
- Enable first-boot service
- Reboot

### Stage 2: First-Boot Service (20-30 minutes)

Runs after cloud-init completes:
- System update and upgrade
- ROS2 installation
- SLAM/Nav2 packages
- LiDAR drivers
- Cleanup

### Why Two Stages?

1. **Quick SSH access**: User can SSH in within 3 minutes
2. **Logging**: Long installations are logged properly
3. **Recovery**: If something fails, user can SSH in and fix
4. **Monitoring**: Can watch progress via `tail -f`

---

## Ubuntu vs Raspberry Pi OS

| Feature | Ubuntu 22.04 | Raspberry Pi OS |
|---------|--------------|-----------------|
| Cloud-init | Pre-installed | Needs setup |
| ROS2 Humble | Native support | Requires compilation |
| 64-bit | Default | Optional |
| First-boot config | cloud-init | Custom scripts |
| SSH enable | user-data | Empty ssh file |

**Recommendation**: Use Ubuntu 22.04 for ROS2 robotics.

---

## Security Notes

1. **Passwordless sudo**: Required for robotics (serial ports, hardware access)
2. **SSH keys**: Preferred over passwords
3. **No WiFi password in git**: Use --wifi-pass at runtime only
4. **UFW**: Not enabled by default, can add in extras

---

## Future Improvements

1. **Fleet Management**: SSH-based deployment to multiple Pis
2. **Web UI**: Simple interface for non-technical users
3. **Validation**: Check waypoints are in free space
4. **OTA Updates**: Push script updates to running Pis

---

## References Used

- [Jim Angel - Automate Ubuntu 22.04 on Pi](https://www.jimangel.io/posts/autoinstall-ubuntu-22-on-raspberry-pi-4/)
- [Cloud-init Documentation](https://cloudinit.readthedocs.io/)
- [Ubuntu Raspberry Pi Tutorial](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi)
- [Richard Bronosky's Pi Cloud-Init Gist](https://gist.github.com/RichardBronosky/fa7d4db13bab3fbb8d9e0fff7ea88aa2)
- [Raspberry Pi Cloud-Init Announcement](https://www.raspberrypi.com/news/cloud-init-on-raspberry-pi-os/)

---

**Summary**: The new_bakery approach uses Ubuntu's built-in cloud-init for initial configuration and a custom systemd service for the lengthy ROS2 installation. This is simpler, more reliable, and easier to maintain than the old PyQt5-based approach.
