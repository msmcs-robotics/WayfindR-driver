"""
Main GUI window for Raspberry Pi Pre-Baker
"""

import os
import threading
from pathlib import Path
from datetime import datetime

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QComboBox, QCheckBox, QLineEdit, QTextEdit, QProgressBar, QFileDialog,
    QGroupBox, QListWidget, QMessageBox, QTabWidget, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from utils.disk_utils import DiskUtils
from workers.wipe_worker import WipeWorker
from workers.flash_worker import FlashWorker
from workers.config_worker import ConfigWorker


class RaspberryPiPreBaker(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        
        # State
        self.image_path = None
        self.selected_device = None
        self.custom_scripts = []
        
        # Workers
        self.wipe_worker = None
        self.flash_worker = None
        self.config_worker = None
        self.current_operation = None
        
        # UI components
        self.flash_button = None
        self.cancel_button = None
        
        self.init_ui()
        self.setup_device_monitoring()
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Raspberry Pi Pre-Baker v2.0")
        self.setGeometry(100, 100, 1000, 800)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # Header
        header = QLabel("🥧 Raspberry Pi Image Flasher & Configurator")
        header.setFont(QFont("Arial", 18, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("color: #C51A4A; padding: 10px;")
        layout.addWidget(header)
        
        # Tabs
        tabs = QTabWidget()
        layout.addWidget(tabs)
        
        # Create tabs
        tabs.addTab(self.create_flash_tab(), "📀 Flash & Configure")
        tabs.addTab(self.create_advanced_tab(), "⚙️ Advanced Options")
        tabs.addTab(self.create_help_tab(), "❓ Help")
        
        # Progress section
        progress_group = QGroupBox("Progress & Status")
        progress_layout = QVBoxLayout()
        progress_group.setLayout(progress_layout)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
            }
        """)
        progress_layout.addWidget(self.progress_bar)
        
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(180)
        self.status_text.setStyleSheet("font-family: monospace;")
        progress_layout.addWidget(self.status_text)
        
        layout.addWidget(progress_group)
        
        # Action buttons
        button_layout = QHBoxLayout()
        
        self.flash_button = QPushButton("🚀 Wipe, Flash && Configure")
        self.flash_button.setEnabled(False)
        self.flash_button.clicked.connect(self.start_wipe_and_flash)
        self.flash_button.setStyleSheet("""
            QPushButton {
                padding: 15px;
                font-size: 16px;
                font-weight: bold;
                background-color: #4CAF50;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        button_layout.addWidget(self.flash_button)
        
        self.cancel_button = QPushButton("❌ Cancel")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self.cancel_operation)
        self.cancel_button.setStyleSheet("""
            QPushButton {
                padding: 15px;
                font-size: 14px;
                background-color: #f44336;
                color: white;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)
        button_layout.addWidget(self.cancel_button)
        
        layout.addLayout(button_layout)
    
    def create_flash_tab(self):
        """Create the main flash & configure tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # Disk Wipe Options
        wipe_group = QGroupBox("1️⃣ Disk Wipe Method")
        wipe_layout = QVBoxLayout()
        wipe_group.setLayout(wipe_layout)
        
        self.wipe_button_group = QButtonGroup()
        
        self.quick_wipe_radio = QRadioButton("⚡ Quick Wipe (Recommended)")
        self.quick_wipe_radio.setChecked(True)
        self.quick_wipe_radio.setToolTip("Erases partition table and first 50MB\nFast and sufficient for most cases")
        wipe_layout.addWidget(self.quick_wipe_radio)
        self.wipe_button_group.addButton(self.quick_wipe_radio)
        
        self.secure_wipe_radio = QRadioButton("🔒 Secure Wipe (1-pass random)")
        self.secure_wipe_radio.setToolTip("Overwrites entire disk with random data\nMore secure but much slower")
        wipe_layout.addWidget(self.secure_wipe_radio)
        self.wipe_button_group.addButton(self.secure_wipe_radio)
        
        self.zero_wipe_radio = QRadioButton("💾 Complete Zero Wipe")
        self.zero_wipe_radio.setToolTip("Overwrites entire disk with zeros\nSlowest but ensures complete erasure")
        wipe_layout.addWidget(self.zero_wipe_radio)
        self.wipe_button_group.addButton(self.zero_wipe_radio)
        
        layout.addWidget(wipe_group)
        
        # Image selection
        image_group = QGroupBox("2️⃣ Select Raspberry Pi Image")
        image_layout = QVBoxLayout()
        image_group.setLayout(image_layout)
        
        image_button_layout = QHBoxLayout()
        self.image_label = QLabel("No image selected")
        self.image_label.setStyleSheet("padding: 5px; font-weight: bold;")
        image_button_layout.addWidget(self.image_label, 1)
        
        select_image_btn = QPushButton("📁 Browse...")
        select_image_btn.clicked.connect(self.select_image)
        select_image_btn.setStyleSheet("padding: 8px;")
        image_button_layout.addWidget(select_image_btn)
        
        image_layout.addLayout(image_button_layout)
        
        # Image safety info
        image_safety_label = QLabel(
            "✅ Source image file will NOT be modified\n"
            "✅ Only the target device will be erased and written to"
        )
        image_safety_label.setStyleSheet("color: green; font-weight: bold; padding: 5px;")
        image_layout.addWidget(image_safety_label)
        
        layout.addWidget(image_group)
        
        # Device selection
        device_group = QGroupBox("3️⃣ Select Target Device")
        device_layout = QVBoxLayout()
        device_group.setLayout(device_layout)
        
        device_button_layout = QHBoxLayout()
        self.device_combo = QComboBox()
        self.device_combo.currentTextChanged.connect(self.on_device_changed)
        self.device_combo.setStyleSheet("padding: 5px;")
        device_button_layout.addWidget(self.device_combo, 1)
        
        refresh_btn = QPushButton("🔄 Refresh")
        refresh_btn.clicked.connect(self.refresh_devices)
        refresh_btn.setStyleSheet("padding: 8px;")
        device_button_layout.addWidget(refresh_btn)
        
        device_layout.addLayout(device_button_layout)
        
        self.device_info_label = QLabel("Select a device to see details")
        self.device_info_label.setStyleSheet("font-family: monospace; padding: 5px;")
        device_layout.addWidget(self.device_info_label)
        
        # Warning
        warning_label = QLabel(
            "⚠️ WARNING: All data on the selected device will be COMPLETELY ERASED!\n"
            "❗ After flashing, device will be safely unmounted and ready to remove"
        )
        warning_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
        device_layout.addWidget(warning_label)
        
        layout.addWidget(device_group)
        
        # Basic config
        config_group = QGroupBox("4️⃣ Basic Configuration")
        config_layout = QVBoxLayout()
        config_group.setLayout(config_layout)
        
        self.ssh_check = QCheckBox("Enable SSH on first boot")
        self.ssh_check.setChecked(True)
        config_layout.addWidget(self.ssh_check)
        
        hostname_layout = QHBoxLayout()
        hostname_layout.addWidget(QLabel("Hostname:"))
        self.hostname_input = QLineEdit("raspberrypi")
        self.hostname_input.setPlaceholderText("Device hostname (timestamp will be appended)")
        hostname_layout.addWidget(self.hostname_input)
        config_layout.addLayout(hostname_layout)
        
        layout.addWidget(config_group)
        
        layout.addStretch()
        
        return tab
    
    def create_advanced_tab(self):
        """Create the advanced options tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # User Configuration
        user_group = QGroupBox("👤 User Configuration")
        user_layout = QVBoxLayout()
        user_group.setLayout(user_layout)
        
        username_layout = QHBoxLayout()
        username_layout.addWidget(QLabel("Username:"))
        self.username_input = QLineEdit("pi")
        self.username_input.setPlaceholderText("Default user account name")
        username_layout.addWidget(self.username_input)
        user_layout.addLayout(username_layout)
        
        password_layout = QHBoxLayout()
        password_layout.addWidget(QLabel("Password:"))
        self.password_input = QLineEdit("raspberry")
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setPlaceholderText("User password")
        password_layout.addWidget(self.password_input)
        user_layout.addLayout(password_layout)
        
        self.nopasswd_sudo_check = QCheckBox("Enable passwordless sudo")
        self.nopasswd_sudo_check.setChecked(True)
        self.nopasswd_sudo_check.setToolTip("Allows user to run sudo without entering password")
        user_layout.addWidget(self.nopasswd_sudo_check)
        
        self.create_dirs_check = QCheckBox("Create working directories in user home")
        self.create_dirs_check.setChecked(True)
        self.create_dirs_check.setToolTip("Creates workspace, projects, downloads, documents, scripts, bin")
        user_layout.addWidget(self.create_dirs_check)
        
        layout.addWidget(user_group)
        
        # WiFi Configuration
        wifi_group = QGroupBox("📡 WiFi Configuration")
        wifi_layout = QVBoxLayout()
        wifi_group.setLayout(wifi_layout)
        
        ssid_layout = QHBoxLayout()
        ssid_layout.addWidget(QLabel("SSID:"))
        self.wifi_ssid_input = QLineEdit()
        self.wifi_ssid_input.setPlaceholderText("WiFi network name")
        ssid_layout.addWidget(self.wifi_ssid_input)
        wifi_layout.addLayout(ssid_layout)
        
        password_layout = QHBoxLayout()
        password_layout.addWidget(QLabel("Password:"))
        self.wifi_password_input = QLineEdit()
        self.wifi_password_input.setEchoMode(QLineEdit.Password)
        self.wifi_password_input.setPlaceholderText("WiFi password")
        password_layout.addWidget(self.wifi_password_input)
        wifi_layout.addLayout(password_layout)
        
        layout.addWidget(wifi_group)
        
        # Hardware Interfaces
        services_group = QGroupBox("🔧 Hardware Interfaces")
        services_layout = QVBoxLayout()
        services_group.setLayout(services_layout)
        
        self.i2c_check = QCheckBox("Enable I2C")
        services_layout.addWidget(self.i2c_check)
        
        self.spi_check = QCheckBox("Enable SPI")
        services_layout.addWidget(self.spi_check)
        
        layout.addWidget(services_group)
        
        # Custom Scripts
        scripts_group = QGroupBox("📜 Custom Scripts (Run on First Boot as ROOT)")
        scripts_layout = QVBoxLayout()
        scripts_group.setLayout(scripts_layout)
        
        scripts_info = QLabel(
            "Scripts will be executed in order on first boot with root privileges.\n"
            "They can install packages, configure system, create directories, etc."
        )
        scripts_info.setStyleSheet("color: blue; font-style: italic;")
        scripts_layout.addWidget(scripts_info)
        
        self.scripts_list = QListWidget()
        scripts_layout.addWidget(self.scripts_list)
        
        script_buttons = QHBoxLayout()
        
        add_script_btn = QPushButton("➕ Add Script")
        add_script_btn.clicked.connect(self.add_script)
        script_buttons.addWidget(add_script_btn)
        
        remove_script_btn = QPushButton("➖ Remove Script")
        remove_script_btn.clicked.connect(self.remove_script)
        script_buttons.addWidget(remove_script_btn)
        
        move_up_btn = QPushButton("⬆️ Move Up")
        move_up_btn.clicked.connect(self.move_script_up)
        script_buttons.addWidget(move_up_btn)
        
        move_down_btn = QPushButton("⬇️ Move Down")
        move_down_btn.clicked.connect(self.move_script_down)
        script_buttons.addWidget(move_down_btn)
        
        scripts_layout.addLayout(script_buttons)
        
        layout.addWidget(scripts_group)
        
        layout.addStretch()
        
        return tab
    
    def create_help_tab(self):
        """Create the help tab"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        help_text = QTextEdit()
        help_text.setReadOnly(True)
        help_text.setHtml("""
<h2>📘 Raspberry Pi Pre-Baker - User Guide</h2>

<h3>🎯 Purpose</h3>
<p>This tool allows you to create fully configured Raspberry Pi SD cards with custom scripts that run on first boot.</p>

<h3>📋 Process Overview</h3>
<ol>
    <li><b>Wipe:</b> Securely erase the target device</li>
    <li><b>Flash:</b> Write the Raspberry Pi image to the device</li>
    <li><b>Configure:</b> Apply your custom settings and scripts</li>
</ol>

<h3>⚙️ Configuration Options</h3>
<ul>
    <li><b>SSH:</b> Enable SSH server for remote access</li>
    <li><b>WiFi:</b> Pre-configure WiFi credentials</li>
    <li><b>Hostname:</b> Set a custom hostname (timestamp will be appended)</li>
    <li><b>User Account:</b> Configure username, password, and sudo privileges</li>
    <li><b>Hardware:</b> Enable I2C and SPI interfaces</li>
    <li><b>Custom Scripts:</b> Add bash scripts that run on first boot as root</li>
</ul>

<h3>🔒 Safety Features</h3>
<ul>
    <li>Source image file is never modified</li>
    <li>System disks are clearly marked with [SYSTEM] flag</li>
    <li>Confirmation required before wiping</li>
    <li>Device is safely unmounted when complete</li>
</ul>

<h3>📜 First Boot Scripts</h3>
<p>Custom scripts you add will:</p>
<ul>
    <li>Run automatically on first boot</li>
    <li>Execute with root privileges</li>
    <li>Run in the order you specify</li>
    <li>Log output to <code>/opt/bakery/firstboot.log</code></li>
    <li>Run only once (service disables itself after completion)</li>
</ul>

<h3>💡 Tips</h3>
<ul>
    <li>Use Quick Wipe for normal use - it's fast and effective</li>
    <li>Secure or Zero wipe for sensitive data or problematic devices</li>
    <li>Keep the device connected throughout the entire process</li>
    <li>Wait for "ready to remove" message before unplugging</li>
</ul>

<h3>🐛 Troubleshooting</h3>
<ul>
    <li><b>Partitions not detected:</b> Try flashing again, verify image integrity</li>
    <li><b>Device not showing:</b> Click Refresh, check USB connection</li>
    <li><b>Permission errors:</b> Ensure running as root (sudo)</li>
</ul>
        """)
        layout.addWidget(help_text)
        
        return tab
    
    """
Main GUI window methods (continued from Part 1)
Add these methods to the RaspberryPiPreBaker class
"""

    # Device Management Methods

    def setup_device_monitoring(self):
        """Setup timer for monitoring device changes"""
        self.device_timer = QTimer()
        self.device_timer.timeout.connect(self.refresh_devices)
        self.device_timer.start(3000)  # Refresh every 3 seconds
        
        # Initial refresh
        self.refresh_devices()

    def refresh_devices(self):
        """Refresh the list of available storage devices"""
        current_selection = self.device_combo.currentText()
        
        self.device_combo.clear()
        
        try:
            disks = DiskUtils.get_available_disks()
            
            for disk in disks:
                self.device_combo.addItem(str(disk))
            
            # Try to restore previous selection
            for i in range(self.device_combo.count()):
                if current_selection in self.device_combo.itemText(i):
                    self.device_combo.setCurrentIndex(i)
                    break
            
        except Exception as e:
            self.log(f"Error refreshing devices: {str(e)}")

    def on_device_changed(self, device_text):
        """Handle device selection change"""
        if device_text:
            device_path = device_text.split()[0]
            self.selected_device = device_path
            
            # Get detailed device info
            info = DiskUtils.get_device_info(device_path)
            self.device_info_label.setText(info)
        
        self.update_flash_button()

    def update_flash_button(self):
        """Enable/disable flash button based on selections"""
        can_flash = bool(self.image_path) and bool(self.selected_device)
        self.flash_button.setEnabled(can_flash)

    # Image and Script Management

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
            filename = os.path.basename(file_path)
            # Show truncated filename if too long
            if len(filename) > 50:
                filename = filename[:47] + "..."
            self.image_label.setText(f"📀 {filename}")
            self.log(f"Selected image: {file_path}")
            self.update_flash_button()

    def add_script(self):
        """Add custom scripts"""
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Select Scripts to Run on First Boot",
            str(Path.home()),
            "Scripts (*.sh *.bash *.py);;All Files (*)"
        )
        
        for file_path in file_paths:
            if file_path and file_path not in self.custom_scripts:
                self.custom_scripts.append(file_path)
                self.scripts_list.addItem(f"📜 {os.path.basename(file_path)}")
                self.log(f"Added script: {os.path.basename(file_path)}")

    def remove_script(self):
        """Remove selected script"""
        current_row = self.scripts_list.currentRow()
        if current_row >= 0:
            removed = self.custom_scripts.pop(current_row)
            self.scripts_list.takeItem(current_row)
            self.log(f"Removed script: {os.path.basename(removed)}")

    def move_script_up(self):
        """Move selected script up in execution order"""
        current_row = self.scripts_list.currentRow()
        if current_row > 0:
            # Swap in list
            self.custom_scripts[current_row], self.custom_scripts[current_row - 1] = \
                self.custom_scripts[current_row - 1], self.custom_scripts[current_row]
            
            # Update UI
            item = self.scripts_list.takeItem(current_row)
            self.scripts_list.insertItem(current_row - 1, item)
            self.scripts_list.setCurrentRow(current_row - 1)

    def move_script_down(self):
        """Move selected script down in execution order"""
        current_row = self.scripts_list.currentRow()
        if 0 <= current_row < len(self.custom_scripts) - 1:
            # Swap in list
            self.custom_scripts[current_row], self.custom_scripts[current_row + 1] = \
                self.custom_scripts[current_row + 1], self.custom_scripts[current_row]
            
            # Update UI
            item = self.scripts_list.takeItem(current_row)
            self.scripts_list.insertItem(current_row + 1, item)
            self.scripts_list.setCurrentRow(current_row + 1)

    def get_selected_wipe_method(self) -> str:
        """Get the currently selected wipe method"""
        if self.quick_wipe_radio.isChecked():
            return "quick"
        elif self.secure_wipe_radio.isChecked():
            return "secure"
        elif self.zero_wipe_radio.isChecked():
            return "zero"
        return "quick"

    # Main Operation Flow

    def start_wipe_and_flash(self):
        """Start the complete wipe, flash, and configure process"""
        # Get wipe method
        wipe_method = self.get_selected_wipe_method()
        wipe_methods = {
            "quick": "Quick Wipe (partition table + 50MB)",
            "secure": "Secure Wipe (1-pass random data)",
            "zero": "Complete Zero Wipe (entire disk)"
        }
        
        # Confirm with user
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setWindowTitle("⚠️ Confirm Disk Wipe and Flash")
        msg.setText(f"<b>This will COMPLETELY ERASE {self.selected_device}!</b>")
        msg.setInformativeText(
            f"<b>Wipe Method:</b> {wipe_methods[wipe_method]}<br>"
            f"<b>Target:</b> {self.selected_device}<br>"
            f"<b>Image:</b> {os.path.basename(self.image_path)}<br><br>"
            "✅ Source image file will NOT be modified<br>"
            "❌ Target device will be completely erased<br>"
            "✅ Device will be safely unmounted when done<br><br>"
            "<b>Are you absolutely sure you want to continue?</b>"
        )
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        
        if msg.exec_() != QMessageBox.Yes:
            return
        
        # Check if device is system disk
        system_disks = DiskUtils.get_system_disks()
        if self.selected_device in system_disks:
            error_msg = QMessageBox()
            error_msg.setIcon(QMessageBox.Critical)
            error_msg.setWindowTitle("❌ System Disk Detected")
            error_msg.setText("<b>Cannot proceed: This is a SYSTEM DISK!</b>")
            error_msg.setInformativeText(
                f"The selected device {self.selected_device} contains system partitions.<br><br>"
                "Flashing this device would destroy your operating system.<br><br>"
                "Please select a different device."
            )
            error_msg.exec_()
            return
        
        # Disable UI
        self.flash_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.progress_bar.setValue(0)
        self.status_text.clear()
        
        # Start wipe operation
        self.log("=" * 60)
        self.log("🚀 STARTING RASPBERRY PI PRE-BAKE PROCESS")
        self.log("=" * 60)
        self.log(f"Wipe Method: {wipe_methods[wipe_method]}")
        self.log(f"Target Device: {self.selected_device}")
        self.log(f"Image: {os.path.basename(self.image_path)}")
        self.log("=" * 60)
        
        self.current_operation = 'wipe'
        self.wipe_worker = WipeWorker(self.selected_device, wipe_method)
        self.wipe_worker.progress.connect(self.on_wipe_progress)
        self.wipe_worker.status.connect(self.log)
        self.wipe_worker.finished.connect(self.on_wipe_finished)
        
        thread = threading.Thread(target=self.wipe_worker.run)
        thread.daemon = True
        thread.start()

    def on_wipe_progress(self, value):
        """Update progress bar during wipe"""
        self.progress_bar.setValue(value)

    def on_wipe_finished(self, success, message):
        """Handle wipe completion and start flash"""
        self.log(message)
        
        if success:
            self.log("=" * 60)
            self.log("📀 STARTING FLASH OPERATION")
            self.log("=" * 60)
            
            self.current_operation = 'flash'
            self.progress_bar.setValue(0)
            
            self.flash_worker = FlashWorker(self.image_path, self.selected_device)
            self.flash_worker.progress.connect(self.on_flash_progress)
            self.flash_worker.status.connect(self.log)
            self.flash_worker.finished.connect(self.on_flash_finished)
            
            thread = threading.Thread(target=self.flash_worker.run)
            thread.daemon = True
            thread.start()
        else:
            self.log("❌ Wipe failed - aborting operation")
            self.reset_ui()

    def on_flash_progress(self, value):
        """Update progress bar during flash"""
        self.progress_bar.setValue(value)

    def on_flash_finished(self, success, message):
        """Handle flash completion and start configuration"""
        self.log(message)
        
        if success:
            self.log("=" * 60)
            self.log("⚙️ STARTING CONFIGURATION")
            self.log("=" * 60)
            
            self.current_operation = 'config'
            self.progress_bar.setValue(0)
            
            # Build configuration
            config = {
                'ssh': self.ssh_check.isChecked(),
                'hostname': self.hostname_input.text(),
                'wifi_ssid': self.wifi_ssid_input.text(),
                'wifi_password': self.wifi_password_input.text(),
                'i2c': self.i2c_check.isChecked(),
                'spi': self.spi_check.isChecked(),
                'custom_scripts': self.custom_scripts,
                'username': self.username_input.text(),
                'password': self.password_input.text(),
                'nopasswd_sudo': self.nopasswd_sudo_check.isChecked(),
                'create_dirs': self.create_dirs_check.isChecked()
            }
            
            self.config_worker = ConfigWorker(self.selected_device, config)
            self.config_worker.status.connect(self.log)
            self.config_worker.finished.connect(self.on_config_finished)
            
            thread = threading.Thread(target=self.config_worker.run)
            thread.daemon = True
            thread.start()
        else:
            self.log("❌ Flash failed - aborting operation")
            self.reset_ui()

    def on_config_finished(self, success, message):
        """Handle configuration completion"""
        self.log(message)
        self.log("=" * 60)
        
        if success:
            self.progress_bar.setValue(100)
            
            # Build completion summary
            summary = [
                "✅ RASPBERRY PI PRE-BAKE COMPLETED SUCCESSFULLY!",
                "",
                "Configuration Summary:",
                f"  • Device: {self.selected_device}",
                f"  • Hostname: {self.hostname_input.text()}-{datetime.now().strftime('%Y%m%d-%H%M')}",
                f"  • SSH: {'Enabled' if self.ssh_check.isChecked() else 'Disabled'}",
            ]
            
            if self.wifi_ssid_input.text():
                summary.append(f"  • WiFi: {self.wifi_ssid_input.text()}")
            
            if self.username_input.text():
                summary.append(f"  • User: {self.username_input.text()}")
                if self.nopasswd_sudo_check.isChecked():
                    summary.append(f"  • Sudo: Passwordless")
            
            if self.custom_scripts:
                summary.append(f"  • First Boot Scripts: {len(self.custom_scripts)}")
            
            summary.extend([
                "",
                "The device has been safely unmounted.",
                "You can now remove it and insert it into your Raspberry Pi!",
                "",
                "First boot may take longer as custom scripts execute."
            ])
            
            for line in summary:
                self.log(line)
            
            # Show success dialog
            QMessageBox.information(
                self,
                "✅ Success",
                "\n".join(summary)
            )
        else:
            self.log("❌ CONFIGURATION FAILED")
            QMessageBox.critical(
                self,
                "❌ Configuration Error",
                f"Configuration failed:\n\n{message}\n\n"
                "The device may be in an inconsistent state.\n"
                "You may need to re-flash."
            )
        
        self.reset_ui()

    def cancel_operation(self):
        """Cancel ongoing operation"""
        if self.current_operation == 'wipe' and self.wipe_worker:
            self.wipe_worker.cancel()
            self.log("⚠️ Cancelling disk wipe...")
        elif self.current_operation == 'flash' and self.flash_worker:
            self.flash_worker.cancel()
            self.log("⚠️ Cancelling flash...")
        elif self.current_operation == 'config':
            self.log("⚠️ Configuration cannot be cancelled once started")

    def reset_ui(self):
        """Reset UI to ready state"""
        self.flash_button.setEnabled(True)
        self.cancel_button.setEnabled(False)
        self.wipe_worker = None
        self.flash_worker = None
        self.config_worker = None
        self.current_operation = None

    def log(self, message):
        """Add message to status text"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.append(f"[{timestamp}] {message}")
        
        # Auto-scroll to bottom
        scrollbar = self.status_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())