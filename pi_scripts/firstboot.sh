#!/bin/bash
# firstboot.sh - Raspberry Pi Baker First Boot Script
# This script runs automatically before any user scripts

set -e

LOG_FILE="/opt/bakery/firstboot.log"
CONFIG_FILE="/opt/bakery/baker-config.json"
STATE_FILE="/opt/bakery/firstboot.state"
MAX_RETRIES=5
RETRY_DELAY=60

# Function to log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# Function to check network connectivity
check_network() {
    log "Checking network connectivity..."
    
    # Try multiple methods to check connectivity
    if ping -c 1 -W 5 8.8.8.8 >/dev/null 2>&1; then
        log "✓ Network connectivity confirmed via ping"
        return 0
    fi
    
    if curl -s --connect-timeout 5 http://www.google.com >/dev/null 2>&1; then
        log "✓ Network connectivity confirmed via HTTP"
        return 0
    fi
    
    log "✗ No network connectivity detected"
    return 1
}

# Function to wait for network with retries
wait_for_network() {
    local retries=0
    
    while [ $retries -lt $MAX_RETRIES ]; do
        if check_network; then
            return 0
        fi
        
        log "Waiting for network... (attempt $((retries + 1))/$MAX_RETRIES)"
        sleep $RETRY_DELAY
        ((retries++))
    done
    
    log "❌ Failed to establish network connection after $MAX_RETRIES attempts"
    return 1
}

# Function to setup retry mechanism
setup_retry_mechanism() {
    log "Setting up retry mechanism for next boot..."
    
    # Create a systemd service that will retry on next boot
    cat > /etc/systemd/system/firstboot-retry.service << 'EOF'
[Unit]
Description=Retry First Boot Setup
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
EOF

    systemctl enable firstboot-retry.service
    log "✓ Retry service configured for next boot"
}

# Function to cleanup and disable services
cleanup_services() {
    log "Cleaning up first-boot services..."
    systemctl disable firstboot.service 2>/dev/null || true
    systemctl disable firstboot-retry.service 2>/dev/null || true
    rm -f /etc/systemd/system/firstboot.service
    rm -f /etc/systemd/system/firstboot-retry.service
    systemctl daemon-reload
    log "✓ First-boot services cleaned up"
}

# Function to mark completion
mark_complete() {
    echo "COMPLETED $(date)" > "$STATE_FILE"
    log "✓ First-boot setup marked as completed"
    cleanup_services
}

# Function to mark failure
mark_failure() {
    echo "FAILED $(date)" >> "$STATE_FILE"
    log "⚠ First-boot setup failed - will retry on next boot"
}

# Function to execute user scripts
execute_user_scripts() {
    if [ -f "/opt/bakery/runlist.txt" ]; then
        SCRIPT_NUM=1
        while IFS= read -r script; do
            if [ -f "/opt/bakery/custom/$script" ]; then
                log ""
                log ">>> [User Script $SCRIPT_NUM] Running: $script"
                log "---"
                
                # Make script executable
                chmod +x "/opt/bakery/custom/$script"
                
                if bash "/opt/bakery/custom/$script" >> "$LOG_FILE" 2>&1; then
                    log "✓ $script completed successfully"
                else
                    log "✗ $script failed with exit code $?"
                    # Continue with other scripts even if one fails
                fi
                ((SCRIPT_NUM++))
            else
                log "✗ Script not found: $script"
            fi
        done < "/opt/bakery/runlist.txt"
    else
        log "No user scripts to run"
    fi
}

# Main execution
main() {
    log "========================================"
    log "RPi First Boot Setup - Starting"
    log "========================================"
    
    # Check if already completed
    if [ -f "$STATE_FILE" ] && grep -q "COMPLETED" "$STATE_FILE"; then
        log "First-boot setup already completed - skipping"
        cleanup_services
        exit 0
    fi
    
    # Wait for network connectivity
    if ! wait_for_network; then
        log "Network not available - setting up retry mechanism"
        setup_retry_mechanism
        mark_failure
        exit 1
    fi
    
    log "✓ Network connectivity established"
    
    # Load configuration
    if [ ! -f "$CONFIG_FILE" ]; then
        log "❌ Configuration file not found: $CONFIG_FILE"
        mark_failure
        exit 1
    fi
    
    USERNAME=$(jq -r '.username' "$CONFIG_FILE")
    ENABLE_SUDO_NOPASSWD=$(jq -r '.enable_sudo_nopasswd' "$CONFIG_FILE")
    
    log "Configuration:"
    log "  Username: $USERNAME"
    log "  Passwordless sudo: $ENABLE_SUDO_NOPASSWD"
    
    # Make config available to all scripts
    cp "$CONFIG_FILE" "/tmp/baker-config.json"
    chmod 644 "/tmp/baker-config.json"
    
    # User configuration
    if [ -n "$USERNAME" ]; then
        log ""
        log ">>> Configuring user: $USERNAME"
        
        if ! id "$USERNAME" &>/dev/null; then
            log "Creating user: $USERNAME"
            useradd -m -G sudo,adm,dialout,cdrom,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi -s /bin/bash "$USERNAME"
            echo "$USERNAME:raspberry" | chpasswd
            log "✓ User created with default password 'raspberry'"
        else
            log "User $USERNAME already exists"
        fi
    fi
    
    # Passwordless sudo configuration
    if [ "$ENABLE_SUDO_NOPASSWD" = "true" ]; then
        log ""
        log ">>> Configuring passwordless sudo"
        
        SUDOERS_FILE="/etc/sudoers.d/010_$USERNAME-nopasswd"
        if [ ! -f "$SUDOERS_FILE" ]; then
            log "Configuring passwordless sudo for $USERNAME"
            echo "# Allow $USERNAME to run sudo without password" > "$SUDOERS_FILE"
            echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> "$SUDOERS_FILE"
            chmod 0440 "$SUDOERS_FILE"
            log "✓ Passwordless sudo configured for $USERNAME"
        else
            log "✓ Passwordless sudo already configured for $USERNAME"
        fi
    fi
    
    # Execute user scripts
    log ""
    log ">>> Executing user scripts"
    execute_user_scripts
    
    log ""
    log "========================================"
    log "First Boot Setup completed successfully!"
    log "========================================"
    
    mark_complete
}

# Run main function and capture all output
main "$@"