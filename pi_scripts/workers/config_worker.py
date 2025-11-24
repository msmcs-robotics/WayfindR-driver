"""
Worker thread for post-flash configuration operations
"""

import os
import json
import subprocess
import time
from datetime import datetime
from typing import Dict, Optional, Tuple
from pathlib import Path

from PyQt5.QtCore import QObject, pyqtSignal

from utils.disk_utils import DiskUtils


class ConfigWorker(QObject):
    """Worker thread for configuration operations"""
    status = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, device_path: str, config: Dict):
        super().__init__()
        self.device_path = device_path
        self.config = config
    
    def run(self):
        """Execute configuration"""
        boot_mount = None
        root_mount = None
        
        try:
            self.status.emit("Starting configuration...")
            
            # IMPROVED: More aggressive partition detection with longer wait
            self.status.emit("Detecting partitions (please be patient)...")
            boot_part, root_part = DiskUtils.detect_partitions(self.device_path, max_attempts=25)
            
            if not boot_part or not root_part:
                # Try one more time after unmounting everything
                self.status.emit("First detection failed, unmounting and retrying...")
                DiskUtils.unmount_all_partitions(self.device_path)
                time.sleep(5)
                
                # Force another partition table reload
                subprocess.run(
                    ['sudo', 'blockdev', '--rereadpt', self.device_path],
                    stderr=subprocess.DEVNULL,
                    timeout=10
                )
                subprocess.run(
                    ['sudo', 'partprobe', self.device_path],
                    stderr=subprocess.DEVNULL,
                    timeout=10
                )
                subprocess.run(['sudo', 'udevadm', 'settle'], timeout=10, stderr=subprocess.DEVNULL)
                time.sleep(5)
                
                boot_part, root_part = DiskUtils.detect_partitions(self.device_path, max_attempts=15)
            
            if not boot_part or not root_part:
                self.finished.emit(False, self._get_partition_error_message())
                return
            
            self.status.emit(f"Found boot partition: {boot_part}")
            self.status.emit(f"Found root partition: {root_part}")
            
            # Verify partitions actually exist as device files
            if not os.path.exists(boot_part) or not os.path.exists(root_part):
                self.status.emit(f"ERROR: Partition device files don't exist!")
                self.status.emit(f"Boot exists: {os.path.exists(boot_part)}")
                self.status.emit(f"Root exists: {os.path.exists(root_part)}")
                self.finished.emit(False, "Partition devices not found in filesystem")
                return
            
            # Wait a bit more for filesystem to settle
            time.sleep(3)
            
            # Mount partitions
            self.status.emit("Mounting partitions...")
            boot_mount, root_mount = self._mount_partitions(boot_part, root_part)
            
            # Apply configurations
            self._apply_boot_config(boot_mount)
            self._apply_root_config(root_mount)
            
            # Configure user accounts
            if self.config.get('username'):
                self._configure_user(root_mount)
            
            # Copy custom scripts
            if self.config.get('custom_scripts'):
                self._copy_scripts(root_mount)
                self._setup_firstboot_service(root_mount)
            
            self.status.emit("Configuration completed successfully")
            self.finished.emit(True, "Configuration applied successfully")
            
        except Exception as e:
            import traceback
            self.status.emit(f"Configuration error: {str(e)}")
            self.status.emit(f"Traceback: {traceback.format_exc()}")
            self.finished.emit(False, f"Configuration failed: {str(e)}")
            
        finally:
            # Always cleanup
            self._cleanup_mounts(boot_mount, root_mount)
    
    def _mount_partitions(self, boot_part: str, root_part: str) -> Tuple[str, str]:
        """Mount boot and root partitions with better error handling"""
        boot_mount = f"/tmp/rpi-boot-{os.getpid()}"
        root_mount = f"/tmp/rpi-root-{os.getpid()}"
        
        os.makedirs(boot_mount, exist_ok=True)
        os.makedirs(root_mount, exist_ok=True)
        
        try:
            # Ensure partitions are unmounted first
            subprocess.run(['sudo', 'umount', boot_part], stderr=subprocess.DEVNULL)
            subprocess.run(['sudo', 'umount', root_part], stderr=subprocess.DEVNULL)
            time.sleep(1)
            
            # Mount boot partition (FAT32)
            self.status.emit(f"Mounting boot partition from {boot_part}...")
            result = subprocess.run(
                ['sudo', 'mount', '-t', 'vfat', boot_part, boot_mount],
                capture_output=True,
                text=True,
                timeout=30
            )
            if result.returncode != 0:
                raise Exception(f"Boot mount failed: {result.stderr}")
            
            self.status.emit(f"✓ Mounted boot: {boot_mount}")
            
            # Verify boot partition has expected files
            boot_files = os.listdir(boot_mount)
            self.status.emit(f"Boot partition contains {len(boot_files)} files/directories")
            
            # Mount root partition (ext4)
            self.status.emit(f"Mounting root partition from {root_part}...")
            result = subprocess.run(
                ['sudo', 'mount', '-t', 'ext4', root_part, root_mount],
                capture_output=True,
                text=True,
                timeout=30
            )
            if result.returncode != 0:
                raise Exception(f"Root mount failed: {result.stderr}")
            
            self.status.emit(f"✓ Mounted root: {root_mount}")
            
            # Verify root partition structure
            if os.path.exists(os.path.join(root_mount, 'etc')):
                self.status.emit("✓ Root filesystem structure verified")
            else:
                raise Exception("Root filesystem doesn't have /etc directory")
            
        except Exception as e:
            # Cleanup on failure
            subprocess.run(['sudo', 'umount', boot_mount], stderr=subprocess.DEVNULL)
            subprocess.run(['sudo', 'umount', root_mount], stderr=subprocess.DEVNULL)
            raise Exception(f"Failed to mount partitions: {e}")
        
        return boot_mount, root_mount
    
    def _apply_boot_config(self, boot_mount: str):
        """Apply configurations to boot partition"""
        self.status.emit("Configuring boot partition...")
        
        # Enable SSH
        if self.config.get('ssh', False):
            ssh_file = os.path.join(boot_mount, 'ssh')
            subprocess.run(['sudo', 'touch', ssh_file], check=True)
            self.status.emit("✓ SSH enabled")
        
        # Configure WiFi
        if self.config.get('wifi_ssid'):
            self._create_wpa_supplicant(boot_mount)
            self.status.emit("✓ WiFi configured")
        
        # Configure boot config.txt for services
        self._configure_boot_config(boot_mount)
    
    def _create_wpa_supplicant(self, boot_mount: str):
        """Create wpa_supplicant.conf for WiFi"""
        wpa_conf = f"""country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={{
    ssid="{self.config['wifi_ssid']}"
    psk="{self.config.get('wifi_password', '')}"
}}
"""
        wpa_file = os.path.join(boot_mount, 'wpa_supplicant.conf')
        
        with open('/tmp/wpa_supplicant.conf', 'w') as f:
            f.write(wpa_conf)
        
        subprocess.run(['sudo', 'cp', '/tmp/wpa_supplicant.conf', wpa_file], check=True)
        os.remove('/tmp/wpa_supplicant.conf')
    
    def _configure_boot_config(self, boot_mount: str):
        """Configure boot/config.txt for hardware interfaces"""
        config_file = os.path.join(boot_mount, 'config.txt')
        
        # Read existing config if it exists
        try:
            result = subprocess.run(
                ['sudo', 'cat', config_file],
                capture_output=True,
                text=True,
                timeout=10
            )
            config_content = result.stdout
        except:
            config_content = ""
        
        # Add configurations if needed
        additions = []
        
        if self.config.get('i2c'):
            if 'dtparam=i2c_arm=on' not in config_content:
                additions.append('dtparam=i2c_arm=on')
            self.status.emit("✓ I2C enabled")
        
        if self.config.get('spi'):
            if 'dtparam=spi=on' not in config_content:
                additions.append('dtparam=spi=on')
            self.status.emit("✓ SPI enabled")
        
        if additions:
            config_content += '\n\n# Added by RPi Pre-Baker\n'
            config_content += '\n'.join(additions) + '\n'
            
            with open('/tmp/config.txt', 'w') as f:
                f.write(config_content)
            
            subprocess.run(['sudo', 'cp', '/tmp/config.txt', config_file], check=True)
            os.remove('/tmp/config.txt')
    
    def _apply_root_config(self, root_mount: str):
        """Apply configurations to root partition"""
        self.status.emit("Configuring root partition...")
        
        # Set hostname
        hostname = self.config.get('hostname', 'raspberrypi')
        timestamp = datetime.now().strftime("%Y%m%d-%H%M")
        full_hostname = f"{hostname}-{timestamp}"
        
        hostname_file = os.path.join(root_mount, 'etc', 'hostname')
        
        with open('/tmp/hostname', 'w') as f:
            f.write(full_hostname + '\n')
        
        subprocess.run(['sudo', 'cp', '/tmp/hostname', hostname_file], check=True)
        os.remove('/tmp/hostname')
        
        self.status.emit(f"✓ Hostname set to: {full_hostname}")
        
        # Update /etc/hosts
        self._update_hosts_file(root_mount, full_hostname)
    
    def _update_hosts_file(self, root_mount: str, hostname: str):
        """Update /etc/hosts with new hostname"""
        hosts_file = os.path.join(root_mount, 'etc', 'hosts')
        
        try:
            result = subprocess.run(
                ['sudo', 'cat', hosts_file],
                capture_output=True,
                text=True,
                timeout=10
            )
            hosts_content = result.stdout
            
            # Replace raspberrypi with new hostname
            hosts_content = hosts_content.replace('raspberrypi', hostname)
            
            with open('/tmp/hosts', 'w') as f:
                f.write(hosts_content)
            
            subprocess.run(['sudo', 'cp', '/tmp/hosts', hosts_file], check=True)
            os.remove('/tmp/hosts')
            
        except Exception as e:
            self.status.emit(f"Warning: Could not update hosts file: {e}")
    
    def _configure_user(self, root_mount: str):
        """Configure user account with full options"""
        username = self.config.get('username', 'pi')
        password = self.config.get('password', 'raspberry')
        nopasswd_sudo = self.config.get('nopasswd_sudo', True)
        create_dirs = self.config.get('create_dirs', True)
        
        self.status.emit(f"Configuring user: {username}")
        
        # Create user configuration script
        user_script = f"""#!/bin/bash
set -e

USERNAME="{username}"
PASSWORD="{password}"
NOPASSWD_SUDO={"true" if nopasswd_sudo else "false"}
CREATE_DIRS={"true" if create_dirs else "false"}

echo "=== User Configuration Script ==="
echo "Configuring user: $USERNAME"

# Check if user exists, if not create it
if ! id "$USERNAME" &>/dev/null; then
    echo "Creating user: $USERNAME"
    useradd -m -G sudo,adm,dialout,cdrom,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi -s /bin/bash "$USERNAME"
else
    echo "User $USERNAME already exists"
    # Ensure user is in required groups
    usermod -a -G sudo,adm,dialout,cdrom,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi "$USERNAME" || true
fi

# Set password
echo "Setting password for $USERNAME"
echo "$USERNAME:$PASSWORD" | chpasswd

# Configure passwordless sudo if requested
if [ "$NOPASSWD_SUDO" = "true" ]; then
    echo "Configuring passwordless sudo for $USERNAME"
    SUDOERS_FILE="/etc/sudoers.d/010_$USERNAME-nopasswd"
    cat > "$SUDOERS_FILE" << 'SUDOEOF'
# Allow {username} to run sudo without password
{username} ALL=(ALL) NOPASSWD:ALL
SUDOEOF
    chmod 0440 "$SUDOERS_FILE"
    chown root:root "$SUDOERS_FILE"
    echo "✓ Passwordless sudo configured"
fi

# Create working directories if requested
if [ "$CREATE_DIRS" = "true" ]; then
    echo "Creating working directories for $USERNAME"
    HOME_DIR="/home/$USERNAME"
    
    # Create directory structure
    mkdir -p "$HOME_DIR/workspace"
    mkdir -p "$HOME_DIR/projects"
    mkdir -p "$HOME_DIR/downloads"
    mkdir -p "$HOME_DIR/documents"
    mkdir -p "$HOME_DIR/scripts"
    mkdir -p "$HOME_DIR/bin"
    
    # Set ownership and permissions
    chown -R "$USERNAME:$USERNAME" "$HOME_DIR"
    chmod 755 "$HOME_DIR"/*
    
    echo "✓ Working directories created"
fi

echo "=== User configuration completed ==="
exit 0
"""
        
        # Write script
        script_path = os.path.join(root_mount, 'tmp', 'configure_user.sh')
        os.makedirs(os.path.join(root_mount, 'tmp'), exist_ok=True)
        
        with open('/tmp/configure_user.sh', 'w') as f:
            f.write(user_script)
        
        subprocess.run(['sudo', 'cp', '/tmp/configure_user.sh', script_path], check=True)
        subprocess.run(['sudo', 'chmod', '+x', script_path], check=True)
        
        # Execute in chroot
        try:
            result = subprocess.run(
                ['sudo', 'chroot', root_mount, '/bin/bash', '/tmp/configure_user.sh'],
                check=True,
                capture_output=True,
                text=True,
                timeout=60
            )
            self.status.emit("✓ User configuration completed")
            
        except subprocess.CalledProcessError as e:
            self.status.emit(f"Warning during user configuration: {e.stderr}")
        
        # Cleanup
        os.remove('/tmp/configure_user.sh')
        subprocess.run(['sudo', 'rm', '-f', script_path], stderr=subprocess.DEVNULL)
    
    def _copy_scripts(self, root_mount: str):
        """Copy custom scripts to /opt/bakery"""
        if not self.config.get('custom_scripts'):
            return
        
        self.status.emit("Copying custom scripts...")
        
        bakery_dir = os.path.join(root_mount, 'opt', 'bakery')
        custom_dir = os.path.join(bakery_dir, 'custom')
        
        subprocess.run(['sudo', 'mkdir', '-p', custom_dir], check=True)
        
        # Copy config
        config_file = os.path.join(bakery_dir, 'bakery_config.json')
        with open('/tmp/bakery_config.json', 'w') as f:
            json.dump(self.config, f, indent=2)
        
        subprocess.run(['sudo', 'cp', '/tmp/bakery_config.json', config_file], check=True)
        os.remove('/tmp/bakery_config.json')
        
        # Copy scripts
        for script_path in self.config['custom_scripts']:
            if os.path.exists(script_path):
                script_name = os.path.basename(script_path)
                dest = os.path.join(custom_dir, script_name)
                subprocess.run(['sudo', 'cp', script_path, dest], check=True)
                subprocess.run(['sudo', 'chmod', '+x', dest], check=True)
                self.status.emit(f"  Copied: {script_name}")
        
        # Create runlist
        runlist_file = os.path.join(bakery_dir, 'runlist.txt')
        with open('/tmp/runlist.txt', 'w') as f:
            for script_path in self.config['custom_scripts']:
                f.write(os.path.basename(script_path) + '\n')
        
        subprocess.run(['sudo', 'cp', '/tmp/runlist.txt', runlist_file], check=True)
        os.remove('/tmp/runlist.txt')
        
        self.status.emit("✓ Scripts copied")
    
    def _setup_firstboot_service(self, root_mount: str):
        """Setup systemd service for first boot script execution"""
        if not self.config.get('custom_scripts'):
            return
        
        self.status.emit("Setting up first boot service...")
        
        # Service file
        service_content = """[Unit]
Description=Raspberry Pi Pre-Baker First Boot Setup
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/opt/bakery/firstboot.sh
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
"""
        
        # First boot script
        script_content = """#!/bin/bash
set -e

LOG_FILE=/opt/bakery/firstboot.log

echo "========================================" | tee -a $LOG_FILE
echo "RPi Pre-Baker First Boot Setup" | tee -a $LOG_FILE
echo "Started at: $(date)" | tee -a $LOG_FILE
echo "========================================" | tee -a $LOG_FILE

if [ -f /opt/bakery/runlist.txt ]; then
    while IFS= read -r script; do
        if [ -f "/opt/bakery/custom/$script" ]; then
            echo "" | tee -a $LOG_FILE
            echo ">>> Running: $script" | tee -a $LOG_FILE
            echo "---" | tee -a $LOG_FILE
            
            if bash "/opt/bakery/custom/$script" >> $LOG_FILE 2>&1; then
                echo "✓ $script completed successfully" | tee -a $LOG_FILE
            else
                echo "✗ $script failed with exit code $?" | tee -a $LOG_FILE
            fi
        else
            echo "✗ Script not found: $script" | tee -a $LOG_FILE
        fi
    done < /opt/bakery/runlist.txt
else
    echo "No runlist found" | tee -a $LOG_FILE
fi

echo "" | tee -a $LOG_FILE
echo "========================================" | tee -a $LOG_FILE
echo "First Boot Setup completed at: $(date)" | tee -a $LOG_FILE
echo "========================================" | tee -a $LOG_FILE

# Disable this service after first run
systemctl disable firstboot.service

exit 0
"""
        
        # Write service file
        service_file = os.path.join(root_mount, 'etc', 'systemd', 'system', 'firstboot.service')
        with open('/tmp/firstboot.service', 'w') as f:
            f.write(service_content)
        subprocess.run(['sudo', 'cp', '/tmp/firstboot.service', service_file], check=True)
        os.remove('/tmp/firstboot.service')
        
        # Write firstboot script
        script_file = os.path.join(root_mount, 'opt', 'bakery', 'firstboot.sh')
        with open('/tmp/firstboot.sh', 'w') as f:
            f.write(script_content)
        subprocess.run(['sudo', 'cp', '/tmp/firstboot.sh', script_file], check=True)
        subprocess.run(['sudo', 'chmod', '+x', script_file], check=True)
        os.remove('/tmp/firstboot.sh')
        
        # Enable service
        symlink_dir = os.path.join(root_mount, 'etc', 'systemd', 'system', 'multi-user.target.wants')
        symlink = os.path.join(symlink_dir, 'firstboot.service')
        subprocess.run(['sudo', 'mkdir', '-p', symlink_dir], check=True)
        subprocess.run(
            ['sudo', 'ln', '-sf', '/etc/systemd/system/firstboot.service', symlink],
            check=True
        )
        
        self.status.emit("✓ First boot service configured")
    
    def _cleanup_mounts(self, boot_mount: Optional[str], root_mount: Optional[str]):
        """Unmount and cleanup mount points"""
        self.status.emit("Unmounting partitions and syncing...")
        
        if root_mount:
            subprocess.run(['sudo', 'umount', root_mount], stderr=subprocess.DEVNULL)
            try:
                os.rmdir(root_mount)
            except:
                pass
        
        if boot_mount:
            subprocess.run(['sudo', 'umount', boot_mount], stderr=subprocess.DEVNULL)
            try:
                os.rmdir(boot_mount)
            except:
                pass
        
        # Final sync
        subprocess.run(['sync'], check=True, timeout=30)
        time.sleep(2)
        
        # Unmount device completely
        DiskUtils.unmount_all_partitions(self.device_path)
        
        self.status.emit("✓ Device safely unmounted - ready to remove")
    
    def _get_partition_error_message(self) -> str:
        """Get detailed error message for partition detection failure"""
        return """Could not detect boot and root partitions.

Possible causes:
1. The flash operation may have failed
2. The image file may be corrupted
3. The device was disconnected during flash
4. The image uses a non-standard partition layout
5. The kernel hasn't recognized the new partitions yet

Troubleshooting:
• Check the flash log above for errors
• Try flashing again with a different image
• Verify the image file is a valid Raspberry Pi image
• Ensure the device stays connected during the entire process
• Try manually running: sudo partprobe {device}
• Try: sudo blockdev --rereadpt {device}
• Try: sudo udevadm settle

The device has been left in its current state.""".format(device=self.device_path)