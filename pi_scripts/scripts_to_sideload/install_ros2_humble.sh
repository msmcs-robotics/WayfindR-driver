#!/bin/bash
# install-ros2-humble.sh
# Sideloader script to install ROS2 Humble on Raspberry Pi
# Idempotent - safe to run multiple times

set -e

SCRIPT_NAME="install-ros2-humble"

# Logging function
log() {
    echo "[$SCRIPT_NAME] $1"
}

# Network check function
check_network() {
    ping -c 1 -W 3 8.8.8.8 &>/dev/null
}

log "========================================"
log "ROS2 Humble Installation Started"
log "========================================"

# ═══════════════════════════════════════════════════════════════════════════
# STEP 1: Get configured username (CRITICAL - script runs as root!)
# ═══════════════════════════════════════════════════════════════════════════
USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")

log "Configuring ROS2 for user: $USERNAME"

# ═══════════════════════════════════════════════════════════════════════════
# STEP 2: Check network availability (ROS2 requires network)
# ═══════════════════════════════════════════════════════════════════════════
if ! check_network; then
    log "❌ Network not available - cannot install ROS2"
    log "Will retry on next boot"
    exit 1
fi

log "✓ Network connectivity confirmed"

# ═══════════════════════════════════════════════════════════════════════════
# STEP 3: Check if ROS2 is already installed (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
if dpkg -l | grep -q "^ii  ros-humble-desktop"; then
    log "✓ ROS2 Humble already installed, skipping installation"
    SKIP_INSTALL=true
else
    log "ROS2 Humble not found, will install"
    SKIP_INSTALL=false
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 4: Handle duplicate/deprecated ROS2 apt source files (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
ROS2_SOURCES="/etc/apt/sources.list.d/ros2.sources"
BACKUP_DIR="/var/backups/ros2-sources"

if [ -f "$ROS2_SOURCES" ]; then
    log "⚠ Found deprecated ros2.sources file"
    
    # Check if already backed up
    if [ ! -f "${ROS2_SOURCES}.backup" ]; then
        mkdir -p "$BACKUP_DIR"
        TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
        mv "$ROS2_SOURCES" "${BACKUP_DIR}/ros2.sources.${TIMESTAMP}"
        log "✓ Moved deprecated file to backup: ${BACKUP_DIR}/ros2.sources.${TIMESTAMP}"
    else
        log "✓ Deprecated file already backed up"
    fi
else
    log "✓ No deprecated ros2.sources file found"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 5: Install ROS2 GPG keyring (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"

if [ ! -f "$KEYRING" ]; then
    log "Installing ROS2 GPG keyring..."
    
    if curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$KEYRING"; then
        chmod 644 "$KEYRING"
        log "✓ ROS2 GPG keyring installed"
    else
        log "❌ Failed to download ROS2 GPG key"
        exit 1
    fi
else
    log "✓ ROS2 GPG keyring already present"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 6: Configure ROS2 apt source (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
ROS2_LIST="/etc/apt/sources.list.d/ros2.list"

# Get Ubuntu codename
if [ -f /etc/os-release ]; then
    . /etc/os-release
    UBUNTU_CODENAME="${UBUNTU_CODENAME:-jammy}"
else
    UBUNTU_CODENAME="jammy"
    log "⚠ Could not determine Ubuntu codename, defaulting to: $UBUNTU_CODENAME"
fi

log "Ubuntu codename: $UBUNTU_CODENAME"

# Check if ros2.list exists and is correct
if [ -f "$ROS2_LIST" ]; then
    if grep -q "packages.ros.org/ros2/ubuntu" "$ROS2_LIST"; then
        log "✓ ROS2 apt source already configured"
    else
        log "⚠ ROS2 apt source exists but may be incorrect, recreating"
        rm -f "$ROS2_LIST"
    fi
fi

# Create ros2.list if it doesn't exist
if [ ! -f "$ROS2_LIST" ]; then
    log "Creating ROS2 apt source list..."
    
    ARCH=$(dpkg --print-architecture)
    cat > "$ROS2_LIST" <<EOF
# ROS2 Humble apt source
deb [arch=$ARCH signed-by=$KEYRING] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main
EOF
    
    log "✓ ROS2 apt source created"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 7: Update apt cache
# ═══════════════════════════════════════════════════════════════════════════
if [ "$SKIP_INSTALL" = false ]; then
    log "Updating apt cache..."
    
    if apt-get update -y; then
        log "✓ Apt cache updated"
    else
        log "⚠ Apt update had warnings but continuing"
    fi
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 8: Install ROS2 Humble Desktop (IDEMPOTENCY - already checked)
# ═══════════════════════════════════════════════════════════════════════════
if [ "$SKIP_INSTALL" = false ]; then
    log "Installing ROS2 Humble Desktop (this may take 10-20 minutes)..."
    
    if apt-get install -y ros-humble-desktop; then
        log "✓ ROS2 Humble Desktop installed successfully"
    else
        log "❌ ROS2 installation failed"
        exit 1
    fi
else
    log "Skipping ROS2 installation (already installed)"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 9: Install additional ROS2 tools (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
if ! dpkg -l | grep -q "^ii  python3-colcon-common-extensions"; then
    log "Installing colcon build tools..."
    apt-get install -y python3-colcon-common-extensions
    log "✓ Colcon tools installed"
else
    log "✓ Colcon tools already installed"
fi

if ! dpkg -l | grep -q "^ii  python3-rosdep"; then
    log "Installing rosdep..."
    apt-get install -y python3-rosdep
    log "✓ rosdep installed"
else
    log "✓ rosdep already installed"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 10: Initialize rosdep (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "Initializing rosdep..."
    rosdep init || log "⚠ rosdep init failed (may already be initialized)"
    log "✓ rosdep initialized"
else
    log "✓ rosdep already initialized"
fi

# Update rosdep as the configured user (not root!)
if ! su - "$USERNAME" -c "rosdep update" &>/dev/null; then
    log "Updating rosdep for user $USERNAME..."
    su - "$USERNAME" -c "rosdep update" || log "⚠ rosdep update had warnings"
    log "✓ rosdep updated for $USERNAME"
else
    log "✓ rosdep already updated"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 11: Configure ROS2 environment in user's .bashrc (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
USER_HOME="/home/$USERNAME"
BASHRC="$USER_HOME/.bashrc"

if [ ! -f "$BASHRC" ]; then
    log "Creating .bashrc for $USERNAME..."
    touch "$BASHRC"
    chown "$USERNAME:$USERNAME" "$BASHRC"
fi

if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    log "Adding ROS2 environment to .bashrc for $USERNAME..."
    
    cat >> "$BASHRC" <<'EOF'

# ═══ ROS2 Humble Environment ═══
source /opt/ros/humble/setup.bash

# ROS2 Configuration
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Optional: Colorize ROS2 logs
export RCUTILS_COLORIZED_OUTPUT=1

# Source workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
EOF
    
    chown "$USERNAME:$USERNAME" "$BASHRC"
    log "✓ ROS2 environment configured in .bashrc"
else
    log "✓ ROS2 environment already configured in .bashrc"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 12: Success summary
# ═══════════════════════════════════════════════════════════════════════════
log ""
log "========================================"
log "✅ ROS2 Humble Installation Complete!"
log "========================================"
log "User: $USERNAME"
log "ROS2 version: Humble"
log "Workspace ready: ~/ros2_ws/src"
log ""
log "Next steps:"
log "1. Create a workspace:"
log "   mkdir -p ~/ros2_ws/src"
log "   cd ~/ros2_ws"
log "   colcon build"
log ""
log "2. Test installation:"
log "   source /opt/ros/humble/setup.bash"
log "   ros2 run demo_nodes_cpp talker"
log "========================================"

exit 0