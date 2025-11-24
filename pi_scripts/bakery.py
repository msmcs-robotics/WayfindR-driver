#!/usr/bin/env python3
"""
Raspberry Pi Pre-Baker - GUI tool for flashing and configuring Raspberry Pi images
"""

import sys
import os
import json
import subprocess
import shutil
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional
import threading
import time

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QCheckBox, QLineEdit, QTextEdit,
    QProgressBar, QFileDialog, QGroupBox, QListWidget, QMessageBox,
    QTabWidget, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont


class FlashWorker(QObject):
    """Worker thread for flashing operations"""
    progress = pyqtSignal(int)
    status = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, image_path: str, device_path: str):
        super().__init__()
        self.image_path = image_path
        self.device_path = device_path
        self.cancelled = False
    
    def run(self):
        try:
            self.status.emit(f"Starting flash of {self.image_path} to {self.device_path}")
            
            # Unmount any mounted partitions
            self.status.emit("Unmounting partitions...")
            self._unmount_device()
            
            # Get image size for progress calculation
            image_size = os.path.getsize(self.image_path)
            
            # Flash using dd
            self.status.emit("Flashing image...")
            block_size = 4 * 1024 * 1024  # 4MB blocks
            
            with open(self.image_path, 'rb') as src:
                with open(self.device_path, 'wb') as dst:
                    bytes_written = 0
                    while True:
                        if self.cancelled:
                            self.finished.emit(False, "Flash cancelled by user")
                            return
                        
                        chunk = src.read(block_size)
                        if not chunk:
                            break
                        
                        dst.write(chunk)
                        bytes_written += len(chunk)
                        
                        progress = int((bytes_written / image_size) * 100)
                        self.progress.emit(progress)
                        self.status.emit(f"Written {bytes_written // (1024*1024)} MB / {image_size // (1024*1024)} MB")
            
            # Sync
            self.status.emit("Syncing...")
            subprocess.run(['sync'], check=True)
            
            self.finished.emit(True, "Flash completed successfully")
            
        except Exception as e:
            self.finished.emit(False, f"Flash failed: {str(e)}")
    
    def _unmount_device(self):
        """Unmount all partitions of the device"""
        try:
            result = subprocess.run(
                ['lsblk', '-ln', self.device_path],
                capture_output=True,
                text=True
            )
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if parts:
                    part_path = f"/dev/{parts[0]}"
                    subprocess.run(['sudo', 'umount', part_path], 
                                 stderr=subprocess.DEVNULL)
        except Exception:
            pass
    
    def cancel(self):
        self.cancelled = True


class ConfigWorker(QObject):
    """Worker thread for configuration operations"""
    status = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, device_path: str, config: Dict):
        super().__init__()
        self.device_path = device_path
        self.config = config
    
    def run(self):
        try:
            self.status.emit("Waiting for partitions to be recognized...")
            time.sleep(2)
            
            # Detect partitions
            boot_part, root_part = self._detect_partitions()
            
            if not boot_part or not root_part:
                self.finished.emit(False, "Could not detect boot and root partitions")
                return
            
            # Mount partitions
            self.status.emit("Mounting partitions...")
            boot_mount, root_mount = self._mount_partitions(boot_part, root_part)
            
            try:
                # Apply configurations
                self._apply_boot_config(boot_mount)
                self._apply_root_config(root_mount)
                self._copy_scripts(root_mount)
                self._setup_firstboot_service(root_mount)
                
                self.status.emit("Configuration complete")
                self.finished.emit(True, "Configuration applied successfully")
                
            finally:
                # Unmount
                self.status.emit("Unmounting partitions...")
                subprocess.run(['sudo', 'umount', boot_mount], stderr=subprocess.DEVNULL)
                subprocess.run(['sudo', 'umount', root_mount], stderr=subprocess.DEVNULL)
                
                # Cleanup mount points
                try:
                    os.rmdir(boot_mount)
                    os.rmdir(root_mount)
                except:
                    pass
            
        except Exception as e:
            self.finished.emit(False, f"Configuration failed: {str(e)}")
    
    def _detect_partitions(self) -> tuple:
        """Detect boot and root partitions"""
        result = subprocess.run(
            ['lsblk', '-ln', '-o', 'NAME,MOUNTPOINT', self.device_path],
            capture_output=True,
            text=True
        )
        
        boot_part = None
        root_part = None
        
        for line in result.stdout.strip().split('\n'):
            parts = line.split()
            if len(parts) >= 1:
                part_name = parts[0]
                
                # Try to identify by partition number
                if 'p1' in part_name or part_name.endswith('1'):
                    boot_part = f"/dev/{part_name}"
                elif 'p2' in part_name or part_name.endswith('2'):
                    root_part = f"/dev/{part_name}"
        
        return boot_part, root_part
    
    def _mount_partitions(self, boot_part: str, root_part: str) -> tuple:
        """Mount boot and root partitions"""
        boot_mount = f"/tmp/rpi-boot-{os.getpid()}"
        root_mount = f"/tmp/rpi-root-{os.getpid()}"
        
        os.makedirs(boot_mount, exist_ok=True)
        os.makedirs(root_mount, exist_ok=True)
        
        subprocess.run(['sudo', 'mount', boot_part, boot_mount], check=True)
        subprocess.run(['sudo', 'mount', root_part, root_mount], check=True)
        
        return boot_mount, root_mount
    
    def _apply_boot_config(self, boot_mount: str):
        """Apply configurations to boot partition"""
        self.status.emit("Configuring boot partition...")
        
        # Enable SSH
        if self.config.get('ssh', False):
            ssh_file = os.path.join(boot_mount, 'ssh')
            subprocess.run(['sudo', 'touch', ssh_file], check=True)
            self.status.emit("SSH enabled")
        
        # Configure WiFi if provided
        if self.config.get('wifi_ssid'):
            self._create_wpa_supplicant(boot_mount)
    
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
        
        self.status.emit("WiFi configured")
    
    def _apply_root_config(self, root_mount: str):
        """Apply configurations to root partition"""
        self.status.emit("Configuring root partition...")
        
        # Set hostname
        hostname = self.config.get('hostname', 'raspberrypi')
        timestamp = datetime.now().strftime("%Y%m%d%H%M")
        full_hostname = f"{hostname}{timestamp}"
        
        hostname_file = os.path.join(root_mount, 'etc', 'hostname')
        
        with open('/tmp/hostname', 'w') as f:
            f.write(full_hostname + '\n')
        
        subprocess.run(['sudo', 'cp', '/tmp/hostname', hostname_file], check=True)
        os.remove('/tmp/hostname')
        
        self.status.emit(f"Hostname set to: {full_hostname}")
        
        # Configure user accounts
        if self.config.get('users'):
            self._configure_users(root_mount)
    
    def _configure_users(self, root_mount: str):
        """Configure user accounts"""
        # This would require chroot operations
        # Simplified version - just document what needs to be done
        pass
    
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
        
        # Create runlist
        runlist_file = os.path.join(bakery_dir, 'runlist.txt')
        with open('/tmp/runlist.txt', 'w') as f:
            for script_path in self.config['custom_scripts']:
                f.write(os.path.basename(script_path) + '\n')
        
        subprocess.run(['sudo', 'cp', '/tmp/runlist.txt', runlist_file], check=True)
        os.remove('/tmp/runlist.txt')
        
        self.status.emit("Scripts copied")
    
    def _setup_firstboot_service(self, root_mount: str):
        """Setup systemd service for first boot"""
        if not self.config.get('custom_scripts'):
            return
        
        self.status.emit("Setting up first boot service...")
        
        service_content = """[Unit]
Description=First Boot Setup
After=network.target

[Service]
Type=oneshot
ExecStart=/opt/bakery/firstboot.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
"""
        
        script_content = """#!/bin/bash
set -e

LOG_FILE=/opt/bakery/firstboot.log
echo "First boot setup started at $(date)" > $LOG_FILE

if [ -f /opt/bakery/runlist.txt ]; then
    while IFS= read -r script; do
        if [ -f "/opt/bakery/custom/$script" ]; then
            echo "Running $script..." >> $LOG_FILE
            bash "/opt/bakery/custom/$script" >> $LOG_FILE 2>&1
        fi
    done < /opt/bakery/runlist.txt
fi

echo "First boot setup completed at $(date)" >> $LOG_FILE

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
        symlink = os.path.join(root_mount, 'etc', 'systemd', 'system', 'multi-user.target.wants', 'firstboot.service')
        subprocess.run(['sudo', 'mkdir', '-p', os.path.dirname(symlink)], check=True)
        subprocess.run(['sudo', 'ln', '-sf', '/etc/systemd/system/firstboot.service', symlink], check=True)
        
        self.status.emit("First boot service configured")


class RaspberryPiPreBaker(QMainWindow):
    def __init__(self):
        super().__init__()
        self.image_path = None
        self.selected_device = None
        self.custom_scripts = []
        self.flash_worker = None
        self.config_worker = None
        self.flash_button = None  # Add this line
        self.cancel_button = None  # Add this line
        
        self.init_ui()
        self.setup_device_monitoring()
    
    def init_ui(self):
        self.setWindowTitle("Raspberry Pi Pre-Baker")
        self.setGeometry(100, 100, 900, 700)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # Header
        header = QLabel("Raspberry Pi Image Flasher & Configurator")
        header.setFont(QFont("Arial", 16, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        layout.addWidget(header)
        
        # WSL Attachment Section
        wsl_group = QGroupBox("WSL USB Device Attachment")
        wsl_layout = QVBoxLayout()
        wsl_group.setLayout(wsl_layout)
        
        wsl_instructions = QLabel(
            "If using WSL and your USB device is not visible:\n"
            "1. Run in Windows PowerShell (Admin): 'usbipd list'\n"
            "2. Find your device BUSID (e.g., 1-5)\n"
            "3. Run: 'usbipd attach --wsl --busid YOUR_BUSID'"
        )
        wsl_instructions.setWordWrap(True)
        wsl_layout.addWidget(wsl_instructions)
        
        wsl_attach_layout = QHBoxLayout()
        wsl_attach_layout.addWidget(QLabel("Attach BUSID:"))
        self.wsl_busid_input = QLineEdit()
        self.wsl_busid_input.setPlaceholderText("e.g., 1-5")
        wsl_attach_layout.addWidget(self.wsl_busid_input)
        
        attach_btn = QPushButton("Attach to WSL")
        attach_btn.clicked.connect(self.attach_wsl_device)
        wsl_attach_layout.addWidget(attach_btn)
        
        wsl_layout.addLayout(wsl_attach_layout)
        layout.addWidget(wsl_group)
        
        # Tabs
        tabs = QTabWidget()
        layout.addWidget(tabs)
        
        # Flash tab
        flash_tab = self.create_flash_tab()
        tabs.addTab(flash_tab, "Flash & Configure")
        
        # Advanced tab
        advanced_tab = self.create_advanced_tab()
        tabs.addTab(advanced_tab, "Advanced Options")
        
        # Progress section
        progress_group = QGroupBox("Progress")
        progress_layout = QVBoxLayout()
        progress_group.setLayout(progress_layout)
        
        self.progress_bar = QProgressBar()
        progress_layout.addWidget(self.progress_bar)
        
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(150)
        progress_layout.addWidget(self.status_text)
        
        layout.addWidget(progress_group)
        
        # Action buttons
        button_layout = QHBoxLayout()
        
        self.flash_button = QPushButton("Flash && Configure")
        self.flash_button.setEnabled(False)
        self.flash_button.clicked.connect(self.start_flash)
        self.flash_button.setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }")
        button_layout.addWidget(self.flash_button)
        
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self.cancel_operation)
        button_layout.addWidget(self.cancel_button)
        
        layout.addLayout(button_layout)
    
    def create_flash_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # Image selection
        image_group = QGroupBox("Image Selection")
        image_layout = QVBoxLayout()
        image_group.setLayout(image_layout)
        
        image_button_layout = QHBoxLayout()
        self.image_label = QLabel("No image selected")
        image_button_layout.addWidget(self.image_label)
        
        select_image_btn = QPushButton("Browse...")
        select_image_btn.clicked.connect(self.select_image)
        image_button_layout.addWidget(select_image_btn)
        
        image_layout.addLayout(image_button_layout)
        layout.addWidget(image_group)
        
        # Device selection
        device_group = QGroupBox("Target Device")
        device_layout = QVBoxLayout()
        device_group.setLayout(device_layout)
        
        device_button_layout = QHBoxLayout()
        self.device_combo = QComboBox()
        self.device_combo.currentTextChanged.connect(self.on_device_changed)
        device_button_layout.addWidget(self.device_combo)
        
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.refresh_devices)
        device_button_layout.addWidget(refresh_btn)
        
        device_layout.addLayout(device_button_layout)
        
        self.device_info_label = QLabel("Select a device")
        device_layout.addWidget(self.device_info_label)
        
        layout.addWidget(device_group)
        
        # Basic config
        config_group = QGroupBox("Basic Configuration")
        config_layout = QVBoxLayout()
        config_group.setLayout(config_layout)
        
        self.ssh_check = QCheckBox("Enable SSH")
        self.ssh_check.setChecked(True)
        config_layout.addWidget(self.ssh_check)
        
        hostname_layout = QHBoxLayout()
        hostname_layout.addWidget(QLabel("Hostname:"))
        self.hostname_input = QLineEdit("raspberrypi")
        hostname_layout.addWidget(self.hostname_input)
        config_layout.addLayout(hostname_layout)
        
        layout.addWidget(config_group)
        
        layout.addStretch()
        
        return tab
    
    def create_advanced_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # WiFi config
        wifi_group = QGroupBox("WiFi Configuration")
        wifi_layout = QVBoxLayout()
        wifi_group.setLayout(wifi_layout)
        
        ssid_layout = QHBoxLayout()
        ssid_layout.addWidget(QLabel("SSID:"))
        self.wifi_ssid_input = QLineEdit()
        ssid_layout.addWidget(self.wifi_ssid_input)
        wifi_layout.addLayout(ssid_layout)
        
        password_layout = QHBoxLayout()
        password_layout.addWidget(QLabel("Password:"))
        self.wifi_password_input = QLineEdit()
        self.wifi_password_input.setEchoMode(QLineEdit.Password)
        password_layout.addWidget(self.wifi_password_input)
        wifi_layout.addLayout(password_layout)
        
        layout.addWidget(wifi_group)
        
        # Services
        services_group = QGroupBox("Services")
        services_layout = QVBoxLayout()
        services_group.setLayout(services_layout)
        
        self.vnc_check = QCheckBox("Enable VNC")
        services_layout.addWidget(self.vnc_check)
        
        self.i2c_check = QCheckBox("Enable I2C")
        services_layout.addWidget(self.i2c_check)
        
        self.spi_check = QCheckBox("Enable SPI")
        services_layout.addWidget(self.spi_check)
        
        layout.addWidget(services_group)
        
        # Custom scripts
        scripts_group = QGroupBox("Custom Scripts (First Boot)")
        scripts_layout = QVBoxLayout()
        scripts_group.setLayout(scripts_layout)
        
        self.scripts_list = QListWidget()
        scripts_layout.addWidget(self.scripts_list)
        
        script_buttons = QHBoxLayout()
        
        add_script_btn = QPushButton("Add Script")
        add_script_btn.clicked.connect(self.add_script)
        script_buttons.addWidget(add_script_btn)
        
        remove_script_btn = QPushButton("Remove Script")
        remove_script_btn.clicked.connect(self.remove_script)
        script_buttons.addWidget(remove_script_btn)
        
        scripts_layout.addLayout(script_buttons)
        
        layout.addWidget(scripts_group)
        
        layout.addStretch()
        
        return tab
    
    def attach_wsl_device(self):
        """Attach USB device to WSL using usbipd"""
        busid = self.wsl_busid_input.text().strip()
        if not busid:
            QMessageBox.warning(self, "Missing BUSID", "Please enter a BUSID (e.g., 1-5)")
            return
        
        try:
            # Run usbipd command through PowerShell
            cmd = f"powershell.exe -Command \"usbipd attach --wsl --busid {busid}\""
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.log(f"Successfully attached device {busid} to WSL")
                QMessageBox.information(self, "Success", f"Device {busid} attached to WSL")
                self.refresh_devices()
            else:
                error_msg = result.stderr.strip()
                self.log(f"Failed to attach device: {error_msg}")
                QMessageBox.critical(self, "Error", f"Failed to attach device:\n{error_msg}")
                
        except Exception as e:
            self.log(f"Error attaching device: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to attach device:\n{str(e)}")
    
    def setup_device_monitoring(self):
        """Setup timer for monitoring device changes"""
        self.device_timer = QTimer()
        self.device_timer.timeout.connect(self.refresh_devices)
        self.device_timer.start(2000)  # Refresh every 2 seconds
        
        # Initial refresh
        self.refresh_devices()
    
    def refresh_devices(self):
        """Refresh the list of available storage devices"""
        current_device = self.device_combo.currentText()
        
        self.device_combo.clear()
        
        try:
            # Get list of block devices with detailed information
            result = subprocess.run(
                ['lsblk', '-d', '-n', '-o', 'NAME,SIZE,TYPE,MODEL,VENDOR,MOUNTPOINT,LABEL'],
                capture_output=True,
                text=True
            )
            
            devices = []
            system_mounts = self._get_system_mounts()
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if len(parts) >= 3:
                    name = parts[0]
                    size = parts[1]
                    dtype = parts[2]
                    
                    if dtype == 'disk':
                        # Skip loop devices and very small devices
                        if not name.startswith('loop'):
                            device_path = f"/dev/{name}"
                            
                            # Get additional info
                            model = parts[3] if len(parts) > 3 else "Unknown"
                            vendor = parts[4] if len(parts) > 4 else "Unknown"
                            mountpoint = parts[5] if len(parts) > 5 else ""
                            label = parts[6] if len(parts) > 6 else ""
                            
                            # Check if this is a system disk
                            is_system = device_path in system_mounts
                            system_flag = " [SYSTEM]" if is_system else ""
                            
                            device_info = f"{device_path} ({size}) - {vendor} {model}{system_flag}"
                            devices.append(device_info)
            
            self.device_combo.addItems(devices)
            
            # Try to restore previous selection
            index = self.device_combo.findText(current_device)
            if index >= 0:
                self.device_combo.setCurrentIndex(index)
            
        except Exception as e:
            self.log(f"Error refreshing devices: {str(e)}")
    
    def _get_system_mounts(self):
        """Get system disk devices (root, home, boot, etc.)"""
        system_mounts = set()
        try:
            with open('/proc/mounts', 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        device = parts[0]
                        mountpoint = parts[1]
                        
                        # Check for system mount points
                        if mountpoint in ['/', '/boot', '/home', '/var', '/usr']:
                            # Resolve device to base disk
                            if device.startswith('/dev/'):
                                # For partitions, get the base disk
                                base_device = self._get_base_device(device)
                                if base_device:
                                    system_mounts.add(base_device)
        
        except Exception as e:
            self.log(f"Error getting system mounts: {str(e)}")
        
        return system_mounts
    
    def _get_base_device(self, device_path: str) -> str:
        """Get the base disk device from a partition"""
        try:
            result = subprocess.run(
                ['lsblk', '-ndo', 'PKNAME', device_path],
                capture_output=True,
                text=True
            )
            base_device = result.stdout.strip()
            if base_device:
                return f"/dev/{base_device}"
        except Exception:
            pass
        return device_path
    
    def on_device_changed(self, device_text):
        """Handle device selection change"""
        if device_text:
            device_path = device_text.split()[0]
            self.selected_device = device_path
            
            # Get detailed device info
            try:
                result = subprocess.run(
                    ['lsblk', '-o', 'NAME,SIZE,MODEL,VENDOR,MOUNTPOINT,LABEL,FSTYPE', device_path],
                    capture_output=True,
                    text=True
                )
                
                info_lines = []
                for i, line in enumerate(result.stdout.strip().split('\n')):
                    if i == 0:  # Header
                        continue
                    if line.strip():
                        info_lines.append(line.strip())
                
                self.device_info_label.setText("\n".join(info_lines))
            except Exception as e:
                self.device_info_label.setText(f"Could not get device info: {str(e)}")
        
        # Only update flash button if it exists
        if hasattr(self, 'flash_button') and self.flash_button is not None:
            if not self.flash_button:
                return
            self.update_flash_button()

    def select_image(self):
        """Open file dialog to select image"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Raspberry Pi Image",
            str(Path.home()),
            "Image Files (*.img *.img.gz *.img.xz *.zip);;All Files (*)"
        )
        
        if file_path:
            self.image_path = file_path
            self.image_label.setText(os.path.basename(file_path))
            self.log(f"Selected image: {file_path}")
            self.update_flash_button()
    
    def add_script(self):
        """Add custom script"""
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Select Scripts",
            str(Path.home()),
            "Scripts (*.sh *.py *.bash);;All Files (*)"
        )
        
        for file_path in file_paths:
            if file_path and file_path not in self.custom_scripts:
                self.custom_scripts.append(file_path)
                self.scripts_list.addItem(os.path.basename(file_path))
                self.log(f"Added script: {file_path}")
    
    def remove_script(self):
        """Remove selected script"""
        current_row = self.scripts_list.currentRow()
        if current_row >= 0:
            removed = self.custom_scripts.pop(current_row)
            self.scripts_list.takeItem(current_row)
            self.log(f"Removed script: {removed}")
    
    def update_flash_button(self):
        """Enable/disable flash button based on selections"""
        if self.flash_button is None:
            return  # Button not initialized yet
        
        can_flash = bool(self.image_path) and bool(self.selected_device)
        self.flash_button.setEnabled(can_flash)

    
    def start_flash(self):
        """Start the flashing and configuration process"""
        # Confirm
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle("Confirm Flash")
        msg.setText(f"This will COMPLETELY ERASE {self.selected_device}!")
        msg.setInformativeText("Are you absolutely sure you want to continue?")
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        
        if msg.exec_() != QMessageBox.Yes:
            return
        
        # Disable UI
        self.flash_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.progress_bar.setValue(0)
        
        # Start flash
        self.log("=" * 50)
        self.log("Starting flash operation...")
        
        self.flash_worker = FlashWorker(self.image_path, self.selected_device)
        self.flash_worker.progress.connect(self.on_flash_progress)
        self.flash_worker.status.connect(self.log)
        self.flash_worker.finished.connect(self.on_flash_finished)
        
        thread = threading.Thread(target=self.flash_worker.run)
        thread.daemon = True
        thread.start()
    
    def on_flash_progress(self, value):
        """Update progress bar"""
        self.progress_bar.setValue(value)
    
    def on_flash_finished(self, success, message):
        """Handle flash completion"""
        self.log(message)
        
        if success:
            # Start configuration
            self.log("=" * 50)
            self.log("Starting configuration...")
            
            config = {
                'ssh': self.ssh_check.isChecked(),
                'hostname': self.hostname_input.text(),
                'wifi_ssid': self.wifi_ssid_input.text(),
                'wifi_password': self.wifi_password_input.text(),
                'vnc': self.vnc_check.isChecked(),
                'i2c': self.i2c_check.isChecked(),
                'spi': self.spi_check.isChecked(),
                'custom_scripts': self.custom_scripts
            }
            
            self.config_worker = ConfigWorker(self.selected_device, config)
            self.config_worker.status.connect(self.log)
            self.config_worker.finished.connect(self.on_config_finished)
            
            thread = threading.Thread(target=self.config_worker.run)
            thread.daemon = True
            thread.start()
        else:
            self.reset_ui()
    
    def on_config_finished(self, success, message):
        """Handle configuration completion"""
        self.log(message)
        self.log("=" * 50)
        
        if success:
            QMessageBox.information(
                self,
                "Success",
                "Flash and configuration completed successfully!\n\n"
                "You can now safely remove the SD card."
            )
        else:
            QMessageBox.critical(
                self,
                "Error",
                f"Configuration failed:\n{message}"
            )
        
        self.reset_ui()
    
    def cancel_operation(self):
        """Cancel ongoing operation"""
        if self.flash_worker:
            self.flash_worker.cancel()
            self.log("Cancelling operation...")
    
    def reset_ui(self):
        """Reset UI to ready state"""
        self.flash_button.setEnabled(True)
        self.cancel_button.setEnabled(False)
        self.progress_bar.setValue(0)
        self.flash_worker = None
        self.config_worker = None
    
    def log(self, message):
        """Add message to status text"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.append(f"[{timestamp}] {message}")
        
        # Auto-scroll to bottom
        scrollbar = self.status_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


def main():
    # Check if running as root
    if os.geteuid() != 0:
        print("This application requires root privileges.")
        print("Please run with: sudo python3 pi_prebaker.py")
        sys.exit(1)
    
    app = QApplication(sys.argv)
    window = RaspberryPiPreBaker()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()