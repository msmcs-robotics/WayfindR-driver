#!/usr/bin/env python3
"""
Simple Raspberry Pi Script Sideloader
Just copies scripts to an already-mounted SD card for first-boot execution
"""

import sys
import os
import json
import shutil
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QTextEdit, QGroupBox, 
    QFileDialog, QMessageBox, QLineEdit, QFormLayout
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class SimplePiSideloader(QMainWindow):
    """Simple script sideloader - just copy files"""
    
    def __init__(self):
        super().__init__()
        self.scripts = []
        self.root_path = None
        self.boot_path = None
        self.init_ui()
    
    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle("Simple Pi Script Sideloader")
        self.setGeometry(100, 100, 800, 700)
        
        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout(widget)
        
        # Header
        header = QLabel("🥧 Raspberry Pi Script Sideloader")
        header.setFont(QFont("Arial", 16, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        layout.addWidget(header)
        
        # Instructions
        info = QLabel(
            "1. Flash your Pi OS to SD card using any tool\n"
            "2. SD card should auto-mount (or manually mount it)\n"
            "3. Select the mounted root partition below\n"
            "4. Add your scripts in execution order\n"
            "5. Click 'Copy Scripts' - Done!\n\n"
            "⚠️  Scripts run as ROOT via systemd on first boot\n"
            "⚠️  Scripts must get username from config for user operations"
        )
        info.setStyleSheet("background-color: #e3f2fd; padding: 10px; border-radius: 5px;")
        layout.addWidget(info)
        
        # Config section
        config_group = QGroupBox("Configuration")
        config_layout = QFormLayout()
        
        self.username_input = QLineEdit("pi")
        config_layout.addRow("Username:", self.username_input)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Path selection
        path_group = QGroupBox("SD Card Location")
        path_layout = QVBoxLayout()
        
        path_row = QHBoxLayout()
        self.path_label = QLabel("No path selected")
        self.path_label.setStyleSheet("font-family: monospace; padding: 5px;")
        path_row.addWidget(self.path_label, 1)
        
        browse_btn = QPushButton("📁 Browse")
        browse_btn.clicked.connect(self.browse_root)
        path_row.addWidget(browse_btn)
        
        path_layout.addLayout(path_row)
        
        hint = QLabel(
            "Select the ROOT partition mount point (e.g., /media/user/rootfs or D:\\)\n"
            "Common locations: /media/$USER/rootfs, /Volumes/rootfs, or Windows drive letter"
        )
        hint.setStyleSheet("color: gray; font-size: 10px; font-style: italic;")
        path_layout.addWidget(hint)
        
        path_group.setLayout(path_layout)
        layout.addWidget(path_group)
        
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
        
        # Usage Guide Section
        guide_group = QGroupBox("📖 Scripting Guide")
        guide_layout = QVBoxLayout()
        
        guide_info = QLabel(
            "Your scripts must be IDEMPOTENT (safe to run multiple times).\n"
            "Always check if work is done before doing it. Click below for complete guide."
        )
        guide_info.setStyleSheet("color: #1976d2; font-style: italic; padding: 5px;")
        guide_layout.addWidget(guide_info)
        
        guide_btn = QPushButton("📋 Copy Complete Scripting Guide (Use as LLM Prompt)")
        guide_btn.clicked.connect(self.copy_usage_guide)
        guide_btn.setStyleSheet("""
            QPushButton {
                padding: 10px;
                background-color: #2196F3;
                color: white;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1976D2; }
        """)
        guide_layout.addWidget(guide_btn)
        
        # Quick reference
        quick_ref = QLabel(
            "<b>Quick Checklist for Scripts:</b><br>"
            "⚠️ <b>Scripts run as ROOT via systemd service</b><br>"
            "• Get username: <code>USERNAME=$(jq -r '.username' /tmp/baker-config.json)</code><br>"
            "• User operations: <code>usermod -aG docker $USERNAME</code><br>"
            "• User files: <code>chown $USERNAME:$USERNAME /home/$USERNAME/file</code><br>"
            "• Check before installing: <code>if ! command -v app; then ... fi</code><br>"
            "• Check before appending: <code>if ! grep -q 'line' file; then ... fi</code><br>"
            "• Check before creating: <code>if [ ! -d /dir ]; then mkdir -p /dir; fi</code><br>"
            "• Exit 0 on success, exit 1 on failure<br>"
            "• Use <code>apt-get install -y</code> (no prompts!)"
        )
        quick_ref.setStyleSheet("""
            background-color: #fff3cd; 
            padding: 10px; 
            border-left: 4px solid #ffc107;
            border-radius: 4px;
            font-size: 10px;
        """)
        quick_ref.setWordWrap(True)
        guide_layout.addWidget(quick_ref)
        
        guide_group.setLayout(guide_layout)
        layout.addWidget(guide_group)
        
        # Status
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMaximumHeight(150)
        self.status_log.setStyleSheet("font-family: monospace; font-size: 11px;")
        status_layout.addWidget(self.status_log)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Action button
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
        layout.addWidget(self.copy_btn)
    
    def browse_root(self):
        """Browse for SD card root partition"""
        path = QFileDialog.getExistingDirectory(
            self, "Select SD Card Root Partition Mount Point"
        )
        
        if path:
            self.root_path = path
            self.path_label.setText(f"Root: {path}")
            
            # Try to detect boot partition
            parent = Path(path).parent
            boot_candidates = [
                parent / "boot",
                parent / "bootfs", 
                Path(path) / "boot",
                parent / "BOOT"
            ]
            
            for boot in boot_candidates:
                if boot.exists() and boot.is_dir():
                    self.boot_path = str(boot)
                    self.log(f"✓ Auto-detected boot partition: {boot}")
                    break
            
            self.log(f"✓ Root partition selected: {path}")
            self.update_button()
    
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
        
        self.update_button()
    
    def remove_script(self):
        """Remove script"""
        row = self.script_list.currentRow()
        if row >= 0:
            removed = self.scripts.pop(row)
            self.script_list.takeItem(row)
            self.update_script_numbers()
            self.log(f"Removed: {os.path.basename(removed)}")
            self.update_button()
    
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
        self.update_button()
    
    def update_script_numbers(self):
        """Update script numbering"""
        for i in range(self.script_list.count()):
            item = self.script_list.item(i)
            text = item.text()
            if text.startswith("#"):
                text = " ".join(text.split()[1:])
            item.setText(f"#{i+1} {text}")
    
    def update_button(self):
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
                "Continue?",
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                return
            
            self.status_log.clear()
            self.log("=" * 50)
            self.log("Starting script copy operation...")
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
            
            # Copy firstboot.sh
            firstboot_src = Path(__file__).parent / "firstboot.sh"
            if firstboot_src.exists():
                firstboot_dest = bakery_dir / "firstboot.sh"
                shutil.copy2(firstboot_src, firstboot_dest)
                firstboot_dest.chmod(0o755)
                self.log(f"✓ Copied: firstboot.sh")
            else:
                self.log("⚠ Warning: firstboot.sh not found - you'll need to copy it manually")
            
            # Create runlist
            runlist_file = bakery_dir / "runlist.txt"
            with open(runlist_file, 'w') as f:
                for script_path in self.scripts:
                    f.write(os.path.basename(script_path) + '\n')
            self.log(f"✓ Created: runlist.txt")
            
            # Create systemd service
            self.create_service(Path(self.root_path))
            
            # Enable SSH if we can find boot partition
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
            self.log("\nNext steps:")
            self.log("  1. Safely eject SD card")
            self.log("  2. Insert into Raspberry Pi")
            self.log("  3. Boot (can be headless)")
            self.log("  4. Scripts run automatically!")
            
            QMessageBox.information(
                self, "Success!",
                f"✅ {len(self.scripts)} scripts copied successfully!\n\n"
                "Scripts will run automatically on first boot.\n"
                "Check /opt/bakery/firstboot.log for results."
            )
            
        except PermissionError as e:
            self.log(f"\n❌ Permission Error: {e}")
            self.log("\nTip: Make sure the SD card is mounted with write permissions")
            QMessageBox.critical(
                self, "Permission Error",
                f"Cannot write to SD card:\n{e}\n\n"
                "Make sure:\n"
                "• SD card is properly mounted\n"
                "• You have write permissions\n"
                "• The partition is not read-only"
            )
        except Exception as e:
            self.log(f"\n❌ Error: {e}")
            QMessageBox.critical(self, "Error", f"Operation failed:\n\n{e}")
    
    def create_service(self, root_path):
        """Create systemd service"""
        service_dir = root_path / "etc" / "systemd" / "system"
        service_file = service_dir / "firstboot.service"
        
        service_dir.mkdir(parents=True, exist_ok=True)
        
        if not service_file.exists():
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
        else:
            self.log("✓ Service already exists (preserved)")
        
        # Enable service
        wants_dir = service_dir / "multi-user.target.wants"
        wants_dir.mkdir(exist_ok=True)
        symlink = wants_dir / "firstboot.service"
        
        if not symlink.exists():
            symlink.symlink_to("../firstboot.service")
            self.log("✓ Service enabled")
        else:
            self.log("✓ Service already enabled")
    
    def enable_ssh(self, boot_path):
        """Enable SSH"""
        ssh_file = boot_path / "ssh"
        if not ssh_file.exists():
            ssh_file.touch()
            self.log("✓ SSH enabled")
        else:
            self.log("✓ SSH already enabled")
    
    def log(self, msg):
        """Log message"""
        self.status_log.append(msg)
        scrollbar = self.status_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def copy_usage_guide(self):
        """Copy usage guide to clipboard"""
        usage_guide = """
═══════════════════════════════════════════════════════════════════════════════
  RASPBERRY PI SCRIPT SIDELOADER - COMPLETE SCRIPTING GUIDE
═══════════════════════════════════════════════════════════════════════════════

This guide explains how to write scripts for the Pi Script Sideloader system.
Use this as a prompt/reference when creating scripts with LLMs or manually.

═══════════════════════════════════════════════════════════════════════════════
  SYSTEM OVERVIEW
═══════════════════════════════════════════════════════════════════════════════

The sideloader copies your scripts to /opt/bakery/custom/ on an SD card.
On first boot, firstboot.sh executes your scripts in order and tracks success.

EXECUTION CONTEXT:
- ⚠️  CRITICAL: Scripts run as ROOT (UID 0) via systemd service
- ⚠️  CRITICAL: You must explicitly get the configured username from config
- ⚠️  CRITICAL: User-specific operations require the username variable
- Scripts run in sequential order (1, 2, 3, ...)
- Failed scripts are retried on next boot
- Successful scripts are marked complete and skipped on retry
- Network may or may not be available when scripts run
- Current working directory is / (root directory)
- Environment is minimal (systemd service context)

FILE LOCATIONS:
- Configuration: /tmp/baker-config.json (available during execution)
- Your scripts: /opt/bakery/custom/your-script.sh
- Logs: /opt/bakery/firstboot.log (all output goes here)
- State tracking: /opt/bakery/firstboot.state
- Execution order: /opt/bakery/runlist.txt

═══════════════════════════════════════════════════════════════════════════════
  CRITICAL REQUIREMENTS - ALL SCRIPTS MUST FOLLOW THESE RULES
═══════════════════════════════════════════════════════════════════════════════

1. IDEMPOTENCY (Most Important!)
   Scripts MUST be safe to run multiple times without causing errors or 
   duplicating work. Always check if something exists before creating/installing.

2. PROPER EXIT CODES
   - Exit 0 on success
   - Exit non-zero (1-255) on failure
   - Failed scripts will be retried on next boot

3. NO INTERACTIVE COMMANDS
   - Use -y flag for apt-get
   - No prompts for user input
   - All operations must be automated

4. HANDLE MISSING NETWORK
   - Check if network is available before network operations
   - Gracefully skip or queue network-dependent tasks
   - Don't fail entirely if network is unavailable

5. LOG ALL ACTIONS
   - Echo what you're doing
   - All output goes to /opt/bakery/firstboot.log
   - Use descriptive messages (success/failure/skip)

6. GET USERNAME FROM CONFIG
   - ALWAYS: USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")
   - Use $USERNAME for all user-specific operations
   - Never hardcode "pi" or assume username

═══════════════════════════════════════════════════════════════════════════════
  SCRIPT TEMPLATE - START WITH THIS
═══════════════════════════════════════════════════════════════════════════════

#!/bin/bash
# script-name.sh - Brief description of what this script does

set -e  # Exit on any error (remove if you want to continue on errors)

# Configuration
SCRIPT_NAME="script-name"
USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")

# Logging function
log() {
    echo "[$SCRIPT_NAME] $1"
}

# Network check function (if needed)
check_network() {
    ping -c 1 -W 3 8.8.8.8 &>/dev/null
}

# Main logic
main() {
    log "Starting $SCRIPT_NAME for user: $USERNAME"
    
    # YOUR IDEMPOTENT CODE HERE
    
    log "$SCRIPT_NAME completed successfully"
}

# Run main function
main "$@"

═══════════════════════════════════════════════════════════════════════════════
  IDEMPOTENCY PATTERNS - CHECK BEFORE ACTING
═══════════════════════════════════════════════════════════════════════════════

PATTERN 1: Package Installation
--------------------------------
# ❌ BAD - Always installs even if present
apt-get install -y docker.io

# ✅ GOOD - Check first
if ! command -v docker &> /dev/null; then
    log "Installing Docker..."
    apt-get update
    apt-get install -y docker.io
    log "✓ Docker installed"
else
    log "✓ Docker already installed, skipping"
fi

PATTERN 2: Adding Lines to Files
---------------------------------
# ❌ BAD - Creates duplicates every run
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# ✅ GOOD - Check if line exists
BASHRC="/home/$USERNAME/.bashrc"
if ! grep -q "export ROS_DOMAIN_ID=42" "$BASHRC"; then
    log "Adding ROS_DOMAIN_ID to .bashrc..."
    echo "export ROS_DOMAIN_ID=42" >> "$BASHRC"
    chown "$USERNAME:$USERNAME" "$BASHRC"
    log "✓ Configuration added"
else
    log "✓ ROS_DOMAIN_ID already configured"
fi

PATTERN 3: Creating Directories/Files
--------------------------------------
# ❌ BAD - Fails if exists
mkdir /opt/myapp

# ✅ GOOD - Check or use -p flag
if [ ! -d "/opt/myapp" ]; then
    log "Creating directory..."
    mkdir -p /opt/myapp
    log "✓ Directory created"
else
    log "✓ Directory already exists"
fi

PATTERN 4: User/Group Operations
---------------------------------
# ❌ BAD - Fails if user already in group OR uses wrong user
usermod -aG docker pi

# ✅ GOOD - Check group membership first AND use configured username
USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")
if ! groups "$USERNAME" | grep -q docker; then
    log "Adding $USERNAME to docker group..."
    usermod -aG docker "$USERNAME"
    log "✓ User added to docker group"
else
    log "✓ $USERNAME already in docker group"
fi

═══════════════════════════════════════════════════════════════════════════════
  COMPLETE EXAMPLE: Docker Installation
═══════════════════════════════════════════════════════════════════════════════

#!/bin/bash
# install-docker.sh - Install Docker and configure for user

set -e

log() { echo "[install-docker] $1"; }

# Get configured username
USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")
log "Setting up Docker for user: $USERNAME"

# Check if Docker installed
if ! command -v docker &> /dev/null; then
    log "Installing Docker..."
    apt-get update
    apt-get install -y docker.io
    log "✓ Docker installed"
else
    log "✓ Docker already installed"
fi

# Add user to docker group
if ! groups "$USERNAME" | grep -q docker; then
    log "Adding $USERNAME to docker group..."
    usermod -aG docker "$USERNAME"
    log "✓ User added to docker group"
else
    log "✓ $USERNAME already in docker group"
fi

# Enable Docker service
if ! systemctl is-enabled docker &>/dev/null; then
    systemctl enable docker
    systemctl start docker
    log "✓ Docker service enabled"
else
    log "✓ Docker already enabled"
fi

log "Docker setup complete for $USERNAME"
exit 0

═══════════════════════════════════════════════════════════════════════════════
  LLM PROMPT TEMPLATE
═══════════════════════════════════════════════════════════════════════════════

Use this when asking an LLM to write sideloader scripts:

"Write a bash script for Raspberry Pi Script Sideloader with these requirements:

EXECUTION CONTEXT:
- Script runs as ROOT via systemd on first boot
- Must get username: USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo 'pi')
- $HOME=/root, $USER=root (don't rely on these for user operations)

CRITICAL CONSTRAINTS:
- Idempotent (safe to run multiple times - CHECK before acting!)
- Exit 0 on success, non-zero on failure
- No interactive commands (use -y flags)
- Handle missing network gracefully
- Use $USERNAME for user operations, never hardcode 'pi'
- Set correct ownership: chown $USERNAME:$USERNAME for user files

TASK: [Describe what you want]

Include:
1. USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo 'pi')
2. Idempotency checks for all operations
3. Clear logging
4. Network checks if needed
5. Proper user file ownership"

═══════════════════════════════════════════════════════════════════════════════
  SUMMARY CHECKLIST
═══════════════════════════════════════════════════════════════════════════════

Before sideloading, verify your script:

✅ Starts with #!/bin/bash
✅ Gets username: USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")
✅ Uses 'set -e' or manual error handling
✅ Checks if work already done (idempotent)
✅ Uses non-interactive commands (-y flags)
✅ Handles network unavailability
✅ Logs all actions clearly
✅ Exits with proper codes (0 = success)
✅ Uses $USERNAME for user operations (not hardcoded "pi")
✅ Sets correct ownership (chown $USERNAME:$USERNAME)
✅ Tested multiple times successfully

═══════════════════════════════════════════════════════════════════════════════
"""
        clipboard = QApplication.clipboard()
        clipboard.setText(usage_guide)
        self.log("✓ Complete scripting guide copied to clipboard!")


def main():
    """Main entry point"""
    app = QApplication(sys.argv)
    app.setApplicationName("Pi Script Sideloader")
    
    # Show platform info
    import platform
    print(f"Running on: {platform.system()}")
    
    window = SimplePiSideloader()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()