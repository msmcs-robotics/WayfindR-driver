#!/usr/bin/env python3
"""
Enhanced Raspberry Pi Script Sideloader
Automatically detects, mounts, and manages SD cards
"""

import sys
import os
import json
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QTextEdit, QGroupBox, 
    QFileDialog, QMessageBox, QLineEdit, QFormLayout, QComboBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont


class DiskManager(QThread):
    """Thread for disk operations to avoid UI freezing"""
    disk_detected = pyqtSignal(list)
    operation_status = pyqtSignal(str)
    mount_complete = pyqtSignal(str, str)  # device, mount_point
    
    def __init__(self):
        super().__init__()
        self.mount_points = {}
    
    def detect_disks(self):
        """Detect available SD cards and USB disks"""
        try:
            # Use lsblk to get disk information
            result = subprocess.run([
                'lsblk', '-J', '-o', 'NAME,SIZE,TYPE,MOUNTPOINT,LABEL,MODEL'
            ], capture_output=True, text=True, check=True)
            
            disks_info = json.loads(result.stdout)
            removable_disks = []
            
            for device in disks_info['blockdevices']:
                # Look for removable media (SD cards, USB drives)
                if device['type'] == 'disk' and (
                    'mmcblk' in device['name'] or  # SD cards
                    'sd' in device['name'] or      # USB drives
                    any(part.get('mountpoint') for part in device.get('children', []))
                ):
                    disk_info = {
                        'device': f"/dev/{device['name']}",
                        'name': device['name'],
                        'size': device['size'],
                        'model': device.get('model', 'Unknown'),
                        'partitions': []
                    }
                    
                    # Get partition info
                    for part in device.get('children', []):
                        part_info = {
                            'device': f"/dev/{part['name']}",
                            'mountpoint': part.get('mountpoint'),
                            'label': part.get('label', ''),
                            'size': part['size']
                        }
                        disk_info['partitions'].append(part_info)
                    
                    removable_disks.append(disk_info)
            
            self.disk_detected.emit(removable_disks)
            
        except subprocess.CalledProcessError as e:
            self.operation_status.emit(f"Error detecting disks: {e}")
        except Exception as e:
            self.operation_status.emit(f"Unexpected error: {e}")
    
    def mount_disk(self, device_path, mount_point=None):
        """Mount a disk partition"""
        try:
            if not mount_point:
                mount_point = tempfile.mkdtemp(prefix='pi_sideloader_')
            
            self.operation_status.emit(f"Mounting {device_path} to {mount_point}...")
            
            result = subprocess.run([
                'sudo', 'mount', device_path, mount_point
            ], capture_output=True, text=True, check=True)
            
            self.mount_points[device_path] = mount_point
            self.mount_complete.emit(device_path, mount_point)
            self.operation_status.emit(f"✓ Successfully mounted {device_path}")
            
            return mount_point
            
        except subprocess.CalledProcessError as e:
            self.operation_status.emit(f"❌ Failed to mount {device_path}: {e}")
            return None
    
    def unmount_disk(self, device_path):
        """Unmount a disk partition"""
        try:
            if device_path in self.mount_points:
                mount_point = self.mount_points[device_path]
                
                self.operation_status.emit(f"Unmounting {device_path}...")
                
                # Sync and unmount
                subprocess.run(['sync'], check=True)
                result = subprocess.run([
                    'sudo', 'umount', mount_point
                ], capture_output=True, text=True, check=True)
                
                # Clean up temporary directory
                if 'pi_sideloader_' in mount_point:
                    os.rmdir(mount_point)
                
                del self.mount_points[device_path]
                self.operation_status.emit(f"✓ Successfully unmounted {device_path}")
                
        except subprocess.CalledProcessError as e:
            self.operation_status.emit(f"❌ Failed to unmount {device_path}: {e}")
        except Exception as e:
            self.operation_status.emit(f"Error during unmount: {e}")
    
    def run(self):
        """Main thread loop"""
        self.detect_disks()


class AutoPiSideloader(QMainWindow):
    """Enhanced sideloader with automatic disk detection"""
    
    def __init__(self):
        super().__init__()
        self.scripts = []
        self.root_path = None
        self.boot_path = None
        self.current_device = None
        self.mount_point = None
        
        # Disk management
        self.disk_manager = DiskManager()
        self.disk_manager.disk_detected.connect(self.on_disks_detected)
        self.disk_manager.operation_status.connect(self.on_disk_status)
        self.disk_manager.mount_complete.connect(self.on_mount_complete)
        
        self.init_ui()
        self.start_disk_detection()
    
    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle("Auto Pi Script Sideloader")
        self.setGeometry(100, 100, 900, 800)
        
        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)
        
        # Header
        header = QLabel("🥧 Automatic Raspberry Pi Script Sideloader")
        header.setFont(QFont("Arial", 16, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        layout.addWidget(header)
        
        # Instructions
        info = QLabel(
            "1. Insert SD card into your computer\n"
            "2. Select detected SD card below\n"
            "3. Add your scripts in execution order\n"
            "4. Click 'Copy Scripts' - Done!\n\n"
            "✅ Automatic mounting/unmounting\n"
            "✅ No manual browsing required\n"
            "✅ Safe ejection after copy\n"
            "⚠️  Scripts run as ROOT on first boot\n"
            "⚠️  Scripts must get username from config"
        )
        info.setStyleSheet("background-color: #e3f2fd; padding: 10px; border-radius: 5px;")
        layout.addWidget(info)
        
        # Disk Detection Section
        disk_group = QGroupBox("SD Card Detection")
        disk_layout = QVBoxLayout()
        
        # Refresh button
        refresh_row = QHBoxLayout()
        refresh_btn = QPushButton("🔄 Refresh Disks")
        refresh_btn.clicked.connect(self.start_disk_detection)
        refresh_row.addWidget(refresh_btn)
        refresh_row.addStretch()
        disk_layout.addLayout(refresh_row)
        
        # Disk selection
        disk_select_row = QHBoxLayout()
        disk_select_row.addWidget(QLabel("Select SD Card:"))
        
        self.disk_combo = QComboBox()
        self.disk_combo.currentTextChanged.connect(self.on_disk_selected)
        disk_select_row.addWidget(self.disk_combo, 1)
        
        self.mount_btn = QPushButton("🔗 Mount")
        self.mount_btn.clicked.connect(self.mount_selected_disk)
        self.mount_btn.setEnabled(False)
        disk_select_row.addWidget(self.mount_btn)
        
        disk_layout.addLayout(disk_select_row)
        
        # Disk info
        self.disk_info = QLabel("No disk selected")
        self.disk_info.setStyleSheet("font-family: monospace; padding: 5px; background-color: #f5f5f5;")
        disk_layout.addWidget(self.disk_info)
        
        disk_group.setLayout(disk_layout)
        layout.addWidget(disk_group)
        
        # Config section
        config_group = QGroupBox("Configuration")
        config_layout = QFormLayout()
        
        self.username_input = QLineEdit("pi")
        config_layout.addRow("Username:", self.username_input)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Scripts section
        script_group = QGroupBox("Scripts (execution order)")
        script_layout = QVBoxLayout()
        
        self.script_list = QListWidget()
        script_layout.addWidget(self.script_list)
        
        btn_row = QHBoxLayout()
        
        add_btn = QPushButton("➕ Add Scripts")
        add_btn.clicked.connect(self.add_scripts)
        btn_row.addWidget(add_btn)
        
        remove_btn = QPushButton("➖ Remove")
        remove_btn.clicked.connect(self.remove_script)
        btn_row.addWidget(remove_btn)
        
        up_btn = QPushButton("⬆️ Move Up")
        up_btn.clicked.connect(self.move_up)
        btn_row.addWidget(up_btn)
        
        down_btn = QPushButton("⬇️ Move Down")
        down_btn.clicked.connect(self.move_down)
        btn_row.addWidget(down_btn)
        
        clear_btn = QPushButton("🗑️ Clear All")
        clear_btn.clicked.connect(self.clear_scripts)
        btn_row.addWidget(clear_btn)
        
        script_layout.addLayout(btn_row)
        script_group.setLayout(script_layout)
        layout.addWidget(script_group)
        
        # Status
        status_group = QGroupBox("Operations Log")
        status_layout = QVBoxLayout()
        
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(200)
        self.status_log.setStyleSheet("font-family: monospace; font-size: 11px;")
        status_layout.addWidget(self.status_log)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Action buttons
        action_layout = QHBoxLayout()
        
        self.unmount_btn = QPushButton("⏏️ Unmount SD Card")
        self.unmount_btn.clicked.connect(self.unmount_current_disk)
        self.unmount_btn.setEnabled(False)
        action_layout.addWidget(self.unmount_btn)
        
        self.copy_btn = QPushButton("🚀 Copy Scripts to SD Card")
        self.copy_btn.setEnabled(False)
        self.copy_btn.clicked.connect(self.copy_scripts)
        self.copy_btn.setStyleSheet("""
            QPushButton {
                padding: 15px;
                font-size: 14px;
                font-weight: bold;
                background-color: #4CAF50;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        action_layout.addWidget(self.copy_btn)
        
        layout.addLayout(action_layout)
    
    def start_disk_detection(self):
        """Start disk detection in background thread"""
        self.log("Scanning for SD cards and USB drives...")
        self.disk_manager.detect_disks()
    
    def on_disks_detected(self, disks):
        """Handle detected disks"""
        self.disk_combo.clear()
        self.available_disks = disks
        
        if not disks:
            self.disk_combo.addItem("No SD cards or USB drives detected")
            self.mount_btn.setEnabled(False)
            self.log("❌ No removable disks found")
            return
        
        for disk in disks:
            display_text = f"{disk['device']} - {disk['size']} - {disk['model']}"
            self.disk_combo.addItem(display_text, disk)
        
        self.log(f"✓ Found {len(disks)} removable disk(s)")
        self.mount_btn.setEnabled(True)
    
    def on_disk_selected(self, text):
        """Handle disk selection"""
        if not text or "No SD cards" in text:
            return
        
        disk_index = self.disk_combo.currentIndex()
        if disk_index >= 0:
            disk = self.disk_combo.itemData(disk_index)
            if disk:
                info_text = f"Device: {disk['device']}\n"
                info_text += f"Size: {disk['size']}\n"
                info_text += f"Model: {disk['model']}\n"
                info_text += f"Partitions: {len(disk['partitions'])}"
                
                for part in disk['partitions']:
                    mount_status = "✓ Mounted" if part['mountpoint'] else "⏏️ Unmounted"
                    info_text += f"\n  - {part['device']} ({part['size']}) {mount_status}"
                
                self.disk_info.setText(info_text)
                self.current_device = disk
    
    def mount_selected_disk(self):
        """Mount the selected disk"""
        if not self.current_device:
            return
        
        # Look for root and boot partitions
        root_partition = None
        boot_partition = None
        
        for part in self.current_device['partitions']:
            if part['mountpoint']:
                # Already mounted
                self.log(f"✓ Using already mounted: {part['device']} at {part['mountpoint']}")
                if 'boot' in part['mountpoint'].lower() or part['label'].lower() == 'boot':
                    self.boot_path = part['mountpoint']
                else:
                    self.root_path = part['mountpoint']
                continue
            
            # Identify partitions
            if part['label'].lower() == 'boot' or 'fat' in part['device']:
                boot_partition = part['device']
            else:
                root_partition = part['device']
        
        # Mount root partition if not already mounted
        if not self.root_path and root_partition:
            self.disk_manager.mount_disk(root_partition)
        
        # Mount boot partition if not already mounted
        if not self.boot_path and boot_partition:
            self.disk_manager.mount_disk(boot_partition)
    
    def on_mount_complete(self, device_path, mount_point):
        """Handle successful mount"""
        # Determine if this is root or boot partition
        if 'boot' in device_path.lower() or self.is_boot_partition(mount_point):
            self.boot_path = mount_point
            self.log(f"✓ Boot partition mounted: {mount_point}")
        else:
            self.root_path = mount_point
            self.log(f"✓ Root partition mounted: {mount_point}")
            self.update_copy_button()
        
        self.mount_point = mount_point
        self.unmount_btn.setEnabled(True)
    
    def is_boot_partition(self, mount_point):
        """Check if mount point contains boot files"""
        try:
            files = os.listdir(mount_point)
            return 'config.txt' in files or 'cmdline.txt' in files
        except:
            return False
    
    def unmount_current_disk(self):
        """Unmount current disk"""
        if self.current_device:
            for part in self.current_device['partitions']:
                if part['device'] in self.disk_manager.mount_points:
                    self.disk_manager.unmount_disk(part['device'])
            
            self.root_path = None
            self.boot_path = None
            self.mount_point = None
            self.unmount_btn.setEnabled(False)
            self.copy_btn.setEnabled(False)
            self.log("✓ Disk unmounted and ready for safe removal")
    
    def on_disk_status(self, message):
        """Handle disk operation status messages"""
        self.log(message)
    
    def add_scripts(self):
        """Add scripts"""
        files, _ = QFileDialog.getOpenFileNames(
            self, "Select Scripts", str(Path.home()),
            "Scripts (*.sh *.bash *.py);;All Files (*)"
        )
        
        for file in files:
            if file not in self.scripts:
                self.scripts.append(file)
                name = os.path.basename(file)
                self.script_list.addItem(f"#{len(self.scripts)} {name}")
                self.log(f"Added: {name}")
        
        self.update_copy_button()
    
    def remove_script(self):
        """Remove script"""
        row = self.script_list.currentRow()
        if row >= 0:
            removed = self.scripts.pop(row)
            self.script_list.takeItem(row)
            self.update_script_numbers()
            self.log(f"Removed: {os.path.basename(removed)}")
            self.update_copy_button()
    
    def move_up(self):
        """Move script up"""
        row = self.script_list.currentRow()
        if row > 0:
            self.scripts[row], self.scripts[row-1] = self.scripts[row-1], self.scripts[row]
            item = self.script_list.takeItem(row)
            self.script_list.insertItem(row-1, item)
            self.script_list.setCurrentRow(row-1)
            self.update_script_numbers()
    
    def move_down(self):
        """Move script down"""
        row = self.script_list.currentRow()
        if 0 <= row < len(self.scripts)-1:
            self.scripts[row], self.scripts[row+1] = self.scripts[row+1], self.scripts[row]
            item = self.script_list.takeItem(row)
            self.script_list.insertItem(row+1, item)
            self.script_list.setCurrentRow(row+1)
            self.update_script_numbers()
    
    def clear_scripts(self):
        """Clear all scripts"""
        self.scripts.clear()
        self.script_list.clear()
        self.log("Cleared all scripts")
        self.update_copy_button()
    
    def update_script_numbers(self):
        """Update script numbering"""
        for i in range(self.script_list.count()):
            item = self.script_list.item(i)
            text = item.text()
            if text.startswith("#"):
                text = " ".join(text.split()[1:])
            item.setText(f"#{i+1} {text}")
    
    def update_copy_button(self):
        """Update copy button state"""
        self.copy_btn.setEnabled(bool(self.root_path and self.scripts))
    
    def copy_scripts(self):
        """Copy scripts to SD card"""
        try:
            username = self.username_input.text().strip()
            
            # Confirm
            reply = QMessageBox.question(
                self, "Confirm Copy",
                f"Copy {len(self.scripts)} scripts to:\n{self.root_path}\n\n"
                f"Username: {username}\n\n"
                "This will create /opt/bakery/ and setup first-boot service.\n"
                "SD card will be automatically unmounted after copy.\n"
                "Continue?",
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                return
            
            self.status_log.clear()
            self.log("=" * 50)
            self.log("Starting automatic script copy...")
            self.log("=" * 50)
            
            # Create directories
            bakery_dir = Path(self.root_path) / "opt" / "bakery"
            custom_dir = bakery_dir / "custom"
            
            self.log(f"\nCreating directories in: {bakery_dir}")
            bakery_dir.mkdir(parents=True, exist_ok=True)
            custom_dir.mkdir(exist_ok=True)
            self.log("✓ Directories created")
            
            # Create configuration
            config = {
                'username': username,
                'scripts_count': len(self.scripts),
                'scripts': [os.path.basename(s) for s in self.scripts]
            }
            
            config_file = bakery_dir / "baker-config.json"
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)
            self.log(f"✓ Configuration saved: {config_file.name}")
            
            # Copy user scripts
            self.log("\nCopying user scripts:")
            for i, script_path in enumerate(self.scripts, 1):
                script_name = os.path.basename(script_path)
                dest = custom_dir / script_name
                shutil.copy2(script_path, dest)
                dest.chmod(0o755)
                self.log(f"  {i}. {script_name} ✓")
            
            # Copy firstboot.sh (you'll need to provide this)
            firstboot_content = """#!/bin/bash
# First boot script - runs user scripts in order

set -e
LOG_FILE="/opt/bakery/firstboot.log"
STATE_FILE="/opt/bakery/firstboot.state"
CONFIG_FILE="/opt/bakery/baker-config.json"
RUNLIST_FILE="/opt/bakery/runlist.txt"

log() {
    echo "[$(date)] $1" >> "$LOG_FILE"
}

# Load configuration
if [ -f "$CONFIG_FILE" ]; then
    USERNAME=$(jq -r '.username' "$CONFIG_FILE")
else
    USERNAME="pi"
fi

log "Starting firstboot setup for user: $USERNAME"

# Check state to see which scripts need to run
if [ ! -f "$STATE_FILE" ]; then
    touch "$STATE_FILE"
fi

# Run scripts in order
if [ -f "$RUNLIST_FILE" ]; then
    while IFS= read -r script; do
        script_path="/opt/bakery/custom/$script"
        
        if [ -f "$script_path" ] && ! grep -q "$script" "$STATE_FILE"; then
            log "Running: $script"
            if bash "$script_path" >> "$LOG_FILE" 2>&1; then
                echo "$script" >> "$STATE_FILE"
                log "✓ $script completed successfully"
            else
                log "❌ $script failed with exit code $?"
                exit 1
            fi
        fi
    done < "$RUNLIST_FILE"
fi

log "Firstboot setup completed successfully"

# Disable service after successful completion
systemctl disable firstboot.service
log "Firstboot service disabled"

exit 0
"""
            
            firstboot_dest = bakery_dir / "firstboot.sh"
            with open(firstboot_dest, 'w') as f:
                f.write(firstboot_content)
            firstboot_dest.chmod(0o755)
            self.log("✓ Created: firstboot.sh")
            
            # Create runlist
            runlist_file = bakery_dir / "runlist.txt"
            with open(runlist_file, 'w') as f:
                for script_path in self.scripts:
                    f.write(os.path.basename(script_path) + '\n')
            self.log(f"✓ Created: runlist.txt")
            
            # Create systemd service
            self.create_service(Path(self.root_path))
            
            # Enable SSH if we have boot partition
            if self.boot_path:
                self.enable_ssh(Path(self.boot_path))
            
            self.log("\n" + "=" * 50)
            self.log("✅ SUCCESS! Scripts copied successfully")
            self.log("=" * 50)
            self.log("\nWhat happens on first boot:")
            self.log("  1. firstboot.sh runs automatically")
            self.log("  2. Creates user (if needed)")
            self.log("  3. Executes your scripts in order:")
            for i, script in enumerate(self.scripts, 1):
                self.log(f"     {i}. {os.path.basename(script)}")
            self.log("  4. Logs everything to /opt/bakery/firstboot.log")
            
            # Auto-unmount
            self.log("\nAuto-unmounting SD card...")
            self.unmount_current_disk()
            
            self.log("\n✅ READY! SD card can be safely removed and inserted into Raspberry Pi")
            
            QMessageBox.information(
                self, "Success!",
                f"✅ {len(self.scripts)} scripts copied successfully!\n\n"
                "SD card has been automatically unmounted and is ready for safe removal.\n"
                "Insert into Raspberry Pi and boot - scripts will run automatically."
            )
            
        except PermissionError as e:
            self.log(f"\n❌ Permission Error: {e}")
            QMessageBox.critical(
                self, "Permission Error",
                f"Cannot write to SD card:\n{e}\n\n"
                "Try running with sudo or check disk permissions."
            )
        except Exception as e:
            self.log(f"\n❌ Error: {e}")
            QMessageBox.critical(self, "Error", f"Operation failed:\n\n{e}")
    
    def create_service(self, root_path):
        """Create systemd service"""
        service_dir = root_path / "etc" / "systemd" / "system"
        service_file = service_dir / "firstboot.service"
        
        service_dir.mkdir(parents=True, exist_ok=True)
        
        service_content = """[Unit]
Description=First Boot Setup
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
        with open(service_file, 'w') as f:
            f.write(service_content)
        self.log("✓ Created: firstboot.service")
        
        # Enable service
        wants_dir = service_dir / "multi-user.target.wants"
        wants_dir.mkdir(exist_ok=True)
        symlink = wants_dir / "firstboot.service"
        
        if not symlink.exists():
            symlink.symlink_to("../firstboot.service")
            self.log("✓ Service enabled")
    
    def enable_ssh(self, boot_path):
        """Enable SSH"""
        ssh_file = boot_path / "ssh"
        if not ssh_file.exists():
            ssh_file.touch()
            self.log("✓ SSH enabled")
    
    def log(self, msg):
        """Log message"""
        self.status_log.append(msg)
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        QApplication.processEvents()  # Ensure UI updates


def main():
    """Main entry point"""
    app = QApplication(sys.argv)
    app.setApplicationName("Auto Pi Script Sideloader")
    
    # Check if running as root for mount operations
    if os.geteuid() != 0:
        print("⚠️  Note: Running without root privileges. Mount operations may require sudo.")
    
    window = AutoPiSideloader()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()