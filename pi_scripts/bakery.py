#!/usr/bin/env python3
"""
Raspberry Pi First-Boot Script Sideloader
A simple tool to add custom scripts that run once on first boot
"""

import sys
import os
import json
import subprocess
import tempfile
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QListWidget, QTextEdit,
    QGroupBox, QFileDialog, QMessageBox, QProgressBar, QLineEdit,
    QCheckBox, QFormLayout, QTextBrowser, QListWidgetItem
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont


class ScriptWorker(QThread):
    """Worker thread for script sideloading operations"""
    progress = pyqtSignal(int)
    status = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, device_path, scripts, username, enable_sudo_nopasswd):
        super().__init__()
        self.device_path = device_path
        self.scripts = scripts
        self.username = username
        self.enable_sudo_nopasswd = enable_sudo_nopasswd
    
    def run(self):
        """Execute script sideloading"""
        boot_mount = None
        root_mount = None
        
        try:
            self.status.emit("Starting script sideloading...")
            self.progress.emit(10)
            
            # Detect partitions
            boot_part, root_part = self.detect_partitions()
            if not boot_part or not root_part:
                self.finished.emit(False, "Could not detect boot and root partitions")
                return
            
            self.status.emit(f"Found boot: {boot_part}")
            self.status.emit(f"Found root: {root_part}")
            self.progress.emit(30)
            
            # Mount partitions
            boot_mount, root_mount = self.mount_partitions(boot_part, root_part)
            self.progress.emit(50)
            
            # Setup first-boot service and copy scripts
            self.setup_firstboot_service(root_mount)
            self.progress.emit(80)
            
            # Enable SSH if boot partition is accessible
            if boot_mount:
                self.enable_ssh(boot_mount)
            
            self.progress.emit(100)
            self.status.emit("Script sideloading completed successfully!")
            self.finished.emit(True, "Scripts configured for first boot execution")
            
        except Exception as e:
            self.status.emit(f"Error: {str(e)}")
            self.finished.emit(False, f"Operation failed: {str(e)}")
        finally:
            self.cleanup_mounts(boot_mount, root_mount)
    
    def detect_partitions(self):
        """Detect boot and root partitions"""
        try:
            # Look for common Raspberry Pi partition patterns
            result = subprocess.run(
                ['lsblk', '-ln', '-o', 'NAME', self.device_path],
                capture_output=True, text=True, timeout=10
            )
            
            boot_part = None
            root_part = None
            
            for line in result.stdout.strip().split('\n'):
                if line.strip():
                    part_name = line.strip()
                    part_path = f"/dev/{part_name}"
                    
                    # First partition is usually boot (FAT32)
                    if part_name.endswith('1') or part_name.endswith('p1'):
                        boot_part = part_path
                    # Second partition is usually root (ext4)
                    elif part_name.endswith('2') or part_name.endswith('p2'):
                        root_part = part_path
            
            return boot_part, root_part
            
        except Exception as e:
            raise Exception(f"Partition detection failed: {e}")
    
    def mount_partitions(self, boot_part, root_part):
        """Mount boot and root partitions"""
        boot_mount = tempfile.mkdtemp(prefix='rpi_boot_')
        root_mount = tempfile.mkdtemp(prefix='rpi_root_')
        
        try:
            # Mount boot partition (FAT32)
            subprocess.run(
                ['sudo', 'mount', boot_part, boot_mount],
                check=True, timeout=30
            )
            
            # Mount root partition (ext4)
            subprocess.run(
                ['sudo', 'mount', root_part, root_mount],
                check=True, timeout=30
            )
            
            return boot_mount, root_mount
            
        except Exception as e:
            # Cleanup on failure
            subprocess.run(['sudo', 'umount', boot_mount], stderr=subprocess.DEVNULL)
            subprocess.run(['sudo', 'umount', root_mount], stderr=subprocess.DEVNULL)
            os.rmdir(boot_mount)
            os.rmdir(root_mount)
            raise Exception(f"Mount failed: {e}")
    
    def setup_firstboot_service(self, root_mount):
        """Setup systemd service for first boot script execution"""
        # Create directory for scripts and config
        bakery_dir = os.path.join(root_mount, 'opt', 'bakery')
        custom_dir = os.path.join(bakery_dir, 'custom')
        
        subprocess.run(['sudo', 'mkdir', '-p', custom_dir], check=True)
        
        # Save configuration as JSON that scripts can read
        config = {
            'username': self.username,
            'enable_sudo_nopasswd': self.enable_sudo_nopasswd,
            'scripts_count': len(self.scripts),
            'scripts': [os.path.basename(script) for script in self.scripts]
        }
        
        config_file = os.path.join(bakery_dir, 'baker-config.json')
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
            json.dump(config, f, indent=2)
            temp_config = f.name
        
        subprocess.run(['sudo', 'cp', temp_config, config_file], check=True)
        os.unlink(temp_config)
        
        # Copy user scripts
        for script_path in self.scripts:
            script_name = os.path.basename(script_path)
            dest = os.path.join(custom_dir, script_name)
            subprocess.run(['sudo', 'cp', script_path, dest], check=True)
            subprocess.run(['sudo', 'chmod', '+x', dest], check=True)
            self.status.emit(f"Copied user script: {script_name}")
        
        # Copy the standalone firstboot.sh script (script 0)
        firstboot_script_source = os.path.join(os.path.dirname(__file__), 'firstboot.sh')
        if os.path.exists(firstboot_script_source):
            firstboot_script_dest = os.path.join(bakery_dir, 'firstboot.sh')
            subprocess.run(['sudo', 'cp', firstboot_script_source, firstboot_script_dest], check=True)
            subprocess.run(['sudo', 'chmod', '+x', firstboot_script_dest], check=True)
            self.status.emit("✓ Copied firstboot.sh (script 0)")
        else:
            raise Exception("firstboot.sh not found in script directory")
        
        # Create runlist for user scripts only
        runlist_file = os.path.join(bakery_dir, 'runlist.txt')
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
            for script_path in self.scripts:
                f.write(os.path.basename(script_path) + '\n')
            temp_runlist = f.name
        
        subprocess.run(['sudo', 'cp', temp_runlist, runlist_file], check=True)
        os.unlink(temp_runlist)
        
        # Create systemd service - check if it already exists first
        service_dir = os.path.join(root_mount, 'etc', 'systemd', 'system')
        service_file = os.path.join(service_dir, 'firstboot.service')
        
        # Only create service if it doesn't exist
        if not os.path.exists(service_file):
            service_content = """[Unit]
Description=Raspberry Pi First Boot Setup
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

            with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
                f.write(service_content)
                temp_service = f.name
            
            subprocess.run(['sudo', 'cp', temp_service, service_file], check=True)
            os.unlink(temp_service)
            self.status.emit("First boot service created")
        else:
            self.status.emit("First boot service already exists - preserving existing")
        
        # Enable service
        symlink_dir = os.path.join(root_mount, 'etc', 'systemd', 'system', 'multi-user.target.wants')
        symlink = os.path.join(symlink_dir, 'firstboot.service')
        subprocess.run(['sudo', 'mkdir', '-p', symlink_dir], check=True)
        
        # Only create symlink if it doesn't exist
        if not os.path.exists(symlink):
            subprocess.run(['sudo', 'ln', '-sf', '/etc/systemd/system/firstboot.service', symlink], check=True)
            self.status.emit("First boot service enabled")
        else:
            self.status.emit("First boot service already enabled - preserving existing")
    
    def enable_ssh(self, boot_mount):
        """Enable SSH by creating empty ssh file in boot partition"""
        ssh_file = os.path.join(boot_mount, 'ssh')
        # Only create if it doesn't exist
        if not os.path.exists(ssh_file):
            subprocess.run(['sudo', 'touch', ssh_file], check=True)
            self.status.emit("SSH enabled (empty 'ssh' file created)")
        else:
            self.status.emit("SSH already enabled (ssh file exists)")
    
    def cleanup_mounts(self, boot_mount, root_mount):
        """Unmount partitions and cleanup"""
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
        
        subprocess.run(['sync'], timeout=30)


class PiScriptSideloader(QMainWindow):
    """Main application window for script sideloading"""
    
    def __init__(self):
        super().__init__()
        self.selected_device = None
        self.scripts = []
        self.worker = None
        
        self.init_ui()
        self.setup_device_monitoring()
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Raspberry Pi First-Boot Script Sideloader")
        self.setGeometry(100, 100, 900, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # Header
        header = QLabel("🥧 Raspberry Pi Script Sideloader")
        header.setFont(QFont("Arial", 16, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("color: #C51A4A; padding: 10px;")
        layout.addWidget(header)
        
        # User Configuration
        user_group = QGroupBox("1️⃣ User Configuration")
        user_layout = QFormLayout()
        user_group.setLayout(user_layout)
        
        self.username_input = QLineEdit("pi")
        self.username_input.setPlaceholderText("Leave as 'pi' to use default user")
        user_layout.addRow("Username:", self.username_input)
        
        self.sudo_nopasswd_check = QCheckBox("Enable passwordless sudo for this user")
        self.sudo_nopasswd_check.setChecked(True)
        user_layout.addRow("", self.sudo_nopasswd_check)
        
        user_note = QLabel(
            "Note: If username is not 'pi', a new user will be created with default password 'raspberry'"
        )
        user_note.setStyleSheet("color: blue; font-style: italic;")
        user_layout.addRow("", user_note)
        
        layout.addWidget(user_group)
        
        # Device selection
        device_group = QGroupBox("2️⃣ Select Raspberry Pi SD Card")
        device_layout = QVBoxLayout()
        device_group.setLayout(device_layout)
        
        device_combo_layout = QHBoxLayout()
        self.device_combo = QComboBox()
        self.device_combo.currentTextChanged.connect(self.on_device_changed)
        device_combo_layout.addWidget(self.device_combo, 1)
        
        refresh_btn = QPushButton("🔄 Refresh")
        refresh_btn.clicked.connect(self.refresh_devices)
        device_combo_layout.addWidget(refresh_btn)
        
        device_layout.addLayout(device_combo_layout)
        
        self.device_info = QLabel("Select a Raspberry Pi SD card to see details")
        self.device_info.setStyleSheet("font-family: monospace; padding: 5px;")
        device_layout.addWidget(self.device_info)
        
        layout.addWidget(device_group)
        
        # Script selection
        script_group = QGroupBox("3️⃣ Select User Scripts to Run on First Boot")
        script_layout = QVBoxLayout()
        script_group.setLayout(script_layout)
        
        script_info = QLabel(
            "User scripts will execute in order on first boot with root privileges.\n"
            "The first-boot script (script 0) runs automatically before user scripts.\n"
            "Scripts can access configuration variables - see usage guide below."
        )
        script_info.setStyleSheet("color: blue; font-style: italic;")
        script_layout.addWidget(script_info)
        
        self.script_list = QListWidget()
        script_layout.addWidget(self.script_list)
        
        script_buttons = QHBoxLayout()
        
        add_script_btn = QPushButton("➕ Add Script")
        add_script_btn.clicked.connect(self.add_script)
        script_buttons.addWidget(add_script_btn)
        
        remove_script_btn = QPushButton("➖ Remove")
        remove_script_btn.clicked.connect(self.remove_script)
        script_buttons.addWidget(remove_script_btn)
        
        move_up_btn = QPushButton("⬆️ Move Up")
        move_up_btn.clicked.connect(self.move_script_up)
        script_buttons.addWidget(move_up_btn)
        
        move_down_btn = QPushButton("⬇️ Move Down")
        move_down_btn.clicked.connect(self.move_script_down)
        script_buttons.addWidget(move_down_btn)
        
        clear_scripts_btn = QPushButton("🗑️ Clear All")
        clear_scripts_btn.clicked.connect(self.clear_scripts)
        script_buttons.addWidget(clear_scripts_btn)
        
        script_layout.addLayout(script_buttons)
        
        layout.addWidget(script_group)
        
        # Usage Guide
        guide_group = QGroupBox("📖 Script Usage Guide")
        guide_layout = QVBoxLayout()
        guide_group.setLayout(guide_layout)
        
        guide_text = QTextBrowser()
        guide_text.setMaximumHeight(200)
        guide_text.setHtml("""
<h3>Configuration Variables Available in Scripts:</h3>
<p>Your scripts can access these configuration values:</p>

<pre><code>
# Method 1: Using jq (recommended)
USERNAME=$(jq -r '.username' /tmp/baker-config.json)
SUDO_NOPASSWD=$(jq -r '.enable_sudo_nopasswd' /tmp/baker-config.json)

# Method 2: Using Python
import json
with open('/tmp/baker-config.json') as f:
    config = json.load(f)
username = config['username']
sudo_nopasswd = config['enable_sudo_nopasswd']
</code></pre>

<h3>Execution Order:</h3>
<ul>
<li><b>Script 0:</b> firstboot.sh (automatic - handles network, user setup, sudo configuration)</li>
<li><b>Script 1-N:</b> Your user scripts (run in the order specified)</li>
</ul>

<h3>File Locations:</h3>
<ul>
<li><b>Configuration:</b> /tmp/baker-config.json (during first boot)</li>
<li><b>First-boot script:</b> /opt/bakery/firstboot.sh</li>
<li><b>User scripts:</b> /opt/bakery/custom/</li>
<li><b>Logs:</b> /opt/bakery/firstboot.log</li>
<li><b>Run Order:</b> /opt/bakery/runlist.txt</li>
</ul>
        """)
        guide_layout.addWidget(guide_text)
        
        copy_guide_btn = QPushButton("📋 Copy Usage Guide to Clipboard")
        copy_guide_btn.clicked.connect(self.copy_usage_guide)
        guide_layout.addWidget(copy_guide_btn)
        
        layout.addWidget(guide_group)
        
        # Progress
        progress_group = QGroupBox("4️⃣ Status")
        progress_layout = QVBoxLayout()
        progress_group.setLayout(progress_layout)
        
        self.progress_bar = QProgressBar()
        progress_layout.addWidget(self.progress_bar)
        
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(150)
        self.status_log.setStyleSheet("font-family: monospace;")
        progress_layout.addWidget(self.status_log)
        
        layout.addWidget(progress_group)
        
        # Action buttons
        button_layout = QHBoxLayout()
        
        self.sideload_btn = QPushButton("🚀 Sideload Scripts")
        self.sideload_btn.setEnabled(False)
        self.sideload_btn.clicked.connect(self.start_sideload)
        self.sideload_btn.setStyleSheet("""
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
        button_layout.addWidget(self.sideload_btn)
        
        self.cancel_btn = QPushButton("❌ Cancel")
        self.cancel_btn.setEnabled(False)
        self.cancel_btn.clicked.connect(self.cancel_operation)
        button_layout.addWidget(self.cancel_btn)
        
        layout.addLayout(button_layout)
    
    def update_script_numbers(self):
        """Update the numbers displayed next to scripts"""
        for i in range(self.script_list.count()):
            item = self.script_list.item(i)
            text = item.text()
            # Remove existing number if present
            if text.startswith("#"):
                text = text.split(" ", 1)[1]
            item.setText(f"#{i+1} {text}")
    
    def add_script(self):
        """Add scripts to the list"""
        files, _ = QFileDialog.getOpenFileNames(
            self, "Select Scripts", str(Path.home()),
            "Scripts (*.sh *.bash *.py);;All Files (*)"
        )
        
        for file in files:
            if file not in self.scripts:
                self.scripts.append(file)
                script_name = os.path.basename(file)
                self.script_list.addItem(f"#{len(self.scripts)} 📜 {script_name}")
                self.log(f"Added user script: {script_name}")
        
        self.update_script_numbers()
        self.update_sideload_button()
    
    def remove_script(self):
        """Remove selected script"""
        row = self.script_list.currentRow()
        if row >= 0:
            removed = self.scripts.pop(row)
            self.script_list.takeItem(row)
            self.log(f"Removed user script: {os.path.basename(removed)}")
            self.update_script_numbers()
            self.update_sideload_button()
    
    def move_script_up(self):
        """Move selected script up in execution order"""
        current_row = self.script_list.currentRow()
        if current_row > 0:
            # Swap in list
            self.scripts[current_row], self.scripts[current_row - 1] = \
                self.scripts[current_row - 1], self.scripts[current_row]
            
            # Update UI
            item = self.script_list.takeItem(current_row)
            self.script_list.insertItem(current_row - 1, item)
            self.script_list.setCurrentRow(current_row - 1)
            self.update_script_numbers()
            self.log(f"Moved user script up: #{current_row + 1} → #{current_row}")
    
    def move_script_down(self):
        """Move selected script down in execution order"""
        current_row = self.script_list.currentRow()
        if 0 <= current_row < len(self.scripts) - 1:
            # Swap in list
            self.scripts[current_row], self.scripts[current_row + 1] = \
                self.scripts[current_row + 1], self.scripts[current_row]
            
            # Update UI
            item = self.script_list.takeItem(current_row)
            self.script_list.insertItem(current_row + 1, item)
            self.script_list.setCurrentRow(current_row + 1)
            self.update_script_numbers()
            self.log(f"Moved user script down: #{current_row + 1} → #{current_row + 2}")
    
    def clear_scripts(self):
        """Clear all scripts"""
        self.scripts.clear()
        self.script_list.clear()
        self.log("Cleared all user scripts")
        self.update_sideload_button()
    
    def copy_usage_guide(self):
        """Copy usage guide to clipboard"""
        usage_guide = """
Raspberry Pi Baker - Script Usage Guide

Execution Order:
- Script 0: firstboot.sh (automatic - handles network, user setup, sudo configuration)
- Script 1-N: Your user scripts (run in the order specified)

Configuration Variables Available in Scripts:

Method 1: Using jq (recommended)
USERNAME=$(jq -r '.username' /tmp/baker-config.json)
SUDO_NOPASSWD=$(jq -r '.enable_sudo_nopasswd' /tmp/baker-config.json)

Method 2: Using Python
import json
with open('/tmp/baker-config.json') as f:
    config = json.load(f)
username = config['username']
sudo_nopasswd = config['enable_sudo_nopasswd']

File Locations:
- Configuration: /tmp/baker-config.json (during first boot)
- First-boot script: /opt/bakery/firstboot.sh
- User scripts: /opt/bakery/custom/
- Logs: /opt/bakery/firstboot.log
- Run Order: /opt/bakery/runlist.txt
"""
        clipboard = QApplication.clipboard()
        clipboard.setText(usage_guide)
        self.log("✓ Usage guide copied to clipboard!")
    
    def setup_device_monitoring(self):
        """Setup device monitoring"""
        self.device_timer = QTimer()
        self.device_timer.timeout.connect(self.refresh_devices)
        self.device_timer.start(3000)
        self.refresh_devices()
    
    def refresh_devices(self):
        """Refresh available storage devices"""
        current = self.device_combo.currentText()
        self.device_combo.clear()
        
        try:
            result = subprocess.run(
                ['lsblk', '-d', '-n', '-o', 'NAME,SIZE,TYPE'],
                capture_output=True, text=True, timeout=10
            )
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if len(parts) >= 3 and parts[2] == 'disk' and not parts[0].startswith('loop'):
                    device = f"/dev/{parts[0]}"
                    size = parts[1]
                    # Skip system disks
                    if not self.is_system_disk(device):
                        self.device_combo.addItem(f"{device} ({size})")
            
            # Restore selection
            if current:
                for i in range(self.device_combo.count()):
                    if current in self.device_combo.itemText(i):
                        self.device_combo.setCurrentIndex(i)
                        break
                        
        except Exception as e:
            self.log(f"Error refreshing devices: {e}")
    
    def is_system_disk(self, device):
        """Check if device is a system disk"""
        try:
            result = subprocess.run(
                ['lsblk', '-o', 'MOUNTPOINT', device],
                capture_output=True, text=True, timeout=5
            )
            return '/' in result.stdout or '/boot' in result.stdout
        except:
            return False
    
    def on_device_changed(self, device_text):
        """Handle device selection"""
        if device_text:
            self.selected_device = device_text.split()[0]
            self.update_sideload_button()
            
            # Show device info
            try:
                result = subprocess.run(
                    ['lsblk', '-o', 'NAME,SIZE,TYPE,MOUNTPOINT', self.selected_device],
                    capture_output=True, text=True, timeout=5
                )
                self.device_info.setText(result.stdout.strip())
            except Exception as e:
                self.device_info.setText(f"Error getting device info: {e}")
    
    def update_sideload_button(self):
        """Enable/disable sideload button"""
        can_sideload = bool(self.selected_device) and bool(self.scripts)
        self.sideload_btn.setEnabled(can_sideload)
    
    def start_sideload(self):
        """Start the sideloading process"""
        username = self.username_input.text().strip()
        enable_sudo = self.sudo_nopasswd_check.isChecked()
        
        # Check if firstboot.sh exists
        firstboot_script = os.path.join(os.path.dirname(__file__), 'firstboot.sh')
        if not os.path.exists(firstboot_script):
            QMessageBox.critical(
                self, "Error",
                "firstboot.sh not found in script directory!\n\n"
                "Please ensure firstboot.sh is in the same directory as bakery.py"
            )
            return
        
        # Confirm action
        reply = QMessageBox.question(
            self, "Confirm Sideload",
            f"This will configure {self.selected_device} with:\n\n"
            f"• Username: {username}\n"
            f"• Passwordless sudo: {'Yes' if enable_sudo else 'No'}\n"
            f"• User Scripts: {len(self.scripts)}\n"
            f"• First-boot script: firstboot.sh (automatic)\n\n"
            "The SD card must already have a Raspberry Pi OS image flashed to it.\n\n"
            "Continue?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        # Disable UI and start worker
        self.sideload_btn.setEnabled(False)
        self.cancel_btn.setEnabled(True)
        self.progress_bar.setValue(0)
        self.status_log.clear()
        
        self.log("Starting script sideloading...")
        self.log(f"Device: {self.selected_device}")
        self.log(f"Username: {username}")
        self.log(f"Passwordless sudo: {enable_sudo}")
        self.log(f"User Scripts: {len(self.scripts)}")
        self.log("First-boot script: firstboot.sh (automatic)")
        self.log("")
        self.log("Execution order:")
        self.log("  Script 0: firstboot.sh (automatic)")
        for i, script in enumerate(self.scripts, 1):
            self.log(f"  Script {i}: {os.path.basename(script)}")
        self.log("")
        self.log("Scripts can access configuration via /tmp/baker-config.json")
        
        self.worker = ScriptWorker(self.selected_device, self.scripts, username, enable_sudo)
        self.worker.progress.connect(self.progress_bar.setValue)
        self.worker.status.connect(self.log)
        self.worker.finished.connect(self.on_sideload_finished)
        self.worker.start()
    
    def on_sideload_finished(self, success, message):
        """Handle sideload completion"""
        self.log(message)
        
        if success:
            username = self.username_input.text().strip()
            enable_sudo = self.sudo_nopasswd_check.isChecked()
            
            self.log("✅ Sideloading completed successfully!")
            self.log("")
            self.log("Configuration Summary:")
            self.log(f"  • Username: {username}")
            self.log(f"  • Passwordless sudo: {'Enabled' if enable_sudo else 'Disabled'}")
            self.log(f"  • User Scripts: {len(self.scripts)}")
            self.log(f"  • First-boot script: firstboot.sh (automatic)")
            self.log("")
            self.log("Execution Order:")
            self.log("  1. firstboot.sh (handles network, user setup, sudo)")
            for i, script in enumerate(self.scripts, 1):
                self.log(f"  {i+1}. {os.path.basename(script)}")
            self.log("")
            self.log("Script Access Information:")
            self.log("  • Configuration: /tmp/baker-config.json")
            self.log("  • First-boot script: /opt/bakery/firstboot.sh")
            self.log("  • User Scripts: /opt/bakery/custom/")
            self.log("  • Logs: /opt/bakery/firstboot.log")
            self.log("")
            self.log("Next steps:")
            self.log("1. Safely remove the SD card")
            self.log("2. Insert it into your Raspberry Pi")
            self.log("3. Boot the Pi - scripts will run automatically")
            
            if username != "pi":
                self.log(f"4. Login with username '{username}', password 'raspberry' (change it!)")
            
            QMessageBox.information(
                self, "Success",
                f"Scripts configured for first boot execution!\n\n"
                f"Username: {username}\n"
                f"Passwordless sudo: {'Yes' if enable_sudo else 'No'}\n"
                f"User Scripts: {len(self.scripts)}\n"
                f"First-boot script: firstboot.sh (automatic)\n\n"
                "Scripts can access configuration via /tmp/baker-config.json\n\n"
                "Insert the SD card into your Raspberry Pi and boot it.\n"
                "Scripts will run automatically on first boot."
            )
        else:
            QMessageBox.critical(
                self, "Error",
                f"Sideloading failed:\n\n{message}"
            )
        
        self.reset_ui()
    
    def cancel_operation(self):
        """Cancel ongoing operation"""
        if self.worker and self.worker.isRunning():
            self.worker.terminate()
            self.worker.wait()
            self.log("⚠️ Operation cancelled")
        self.reset_ui()
    
    def reset_ui(self):
        """Reset UI to ready state"""
        self.sideload_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)
        self.worker = None
    
    def log(self, message):
        """Add message to status log"""
        self.status_log.append(message)
        # Auto-scroll
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


def check_root():
    """Check if running with root privileges"""
    if os.geteuid() != 0:
        print("=" * 60)
        print("ERROR: This application requires root privileges")
        print("=" * 60)
        print("\nPlease run with: sudo python3 bakery.py")
        print("=" * 60)
        sys.exit(1)


def main():
    """Main entry point"""
    check_root()
    
    app = QApplication(sys.argv)
    app.setApplicationName("RPi Script Sideloader")
    
    window = PiScriptSideloader()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()