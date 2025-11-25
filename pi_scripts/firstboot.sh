#!/bin/bash
# firstboot.sh - Simple First Boot Orchestrator
# Just runs your scripts in order and tracks success

set -e

LOG_FILE="/opt/bakery/firstboot.log"
CONFIG_FILE="/opt/bakery/baker-config.json"
STATE_FILE="/opt/bakery/firstboot.state"
RUNLIST="/opt/bakery/runlist.txt"

# Log with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# Check if script already ran successfully
script_completed() {
    local script=$1
    grep -q "^SUCCESS:$script$" "$STATE_FILE" 2>/dev/null
}

# Mark script as complete
mark_script_complete() {
    local script=$1
    echo "SUCCESS:$script" >> "$STATE_FILE"
}

# Check if all scripts completed
all_scripts_complete() {
    [ -f "$RUNLIST" ] || return 1
    
    while IFS= read -r script; do
        if ! script_completed "$script"; then
            return 1
        fi
    done < "$RUNLIST"
    
    return 0
}

# Create user if configured
setup_user() {
    if [ ! -f "$CONFIG_FILE" ]; then
        log "No config file found, skipping user setup"
        return
    fi
    
    # Check if jq is available, if not skip
    if ! command -v jq &> /dev/null; then
        log "jq not installed, skipping config parsing"
        return
    fi
    
    local username=$(jq -r '.username // "pi"' "$CONFIG_FILE")
    
    if [ "$username" != "pi" ]; then
        if ! id "$username" &>/dev/null; then
            log "Creating user: $username"
            useradd -m -G sudo,adm,dialout,cdrom,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi \
                    -s /bin/bash "$username"
            echo "$username:raspberry" | chpasswd
            log "✓ User created (password: raspberry)"
        else
            log "✓ User $username exists"
        fi
    fi
}

# Run a single script with proper error handling
run_script() {
    local script=$1
    local script_path="/opt/bakery/custom/$script"
    
    if [ ! -f "$script_path" ]; then
        log "❌ Script not found: $script"
        return 1
    fi
    
    # Make executable
    chmod +x "$script_path"
    
    log ""
    log ">>> Running: $script"
    log "---"
    
    # Determine how to run the script
    if [[ "$script" == *.py ]]; then
        if python3 "$script_path" >> "$LOG_FILE" 2>&1; then
            log "✓ $script completed"
            return 0
        else
            local exit_code=$?
            log "❌ $script failed (exit code: $exit_code)"
            return 1
        fi
    else
        if bash "$script_path" >> "$LOG_FILE" 2>&1; then
            log "✓ $script completed"
            return 0
        else
            local exit_code=$?
            log "❌ $script failed (exit code: $exit_code)"
            return 1
        fi
    fi
}

# Main execution
main() {
    log "========================================"
    log "First Boot Setup - Starting"
    log "========================================"
    
    # Initialize state file
    [ -f "$STATE_FILE" ] || touch "$STATE_FILE"
    
    # Check if already complete
    if all_scripts_complete; then
        log "All scripts already completed successfully"
        log "Disabling first-boot service..."
        systemctl disable firstboot.service 2>/dev/null || true
        exit 0
    fi
    
    # Setup user if needed
    setup_user
    
    # Make config available to scripts
    if [ -f "$CONFIG_FILE" ]; then
        cp "$CONFIG_FILE" "/tmp/baker-config.json" 2>/dev/null || true
        chmod 644 "/tmp/baker-config.json" 2>/dev/null || true
    fi
    
    # Run scripts in order
    if [ ! -f "$RUNLIST" ]; then
        log "No scripts to run (runlist.txt not found)"
        exit 0
    fi
    
    log ""
    log ">>> Executing Scripts"
    
    local script_num=1
    local failed_count=0
    local success_count=0
    
    while IFS= read -r script; do
        # Skip if already completed
        if script_completed "$script"; then
            log "[$script_num] $script - already completed, skipping"
            ((script_num++))
            ((success_count++))
            continue
        fi
        
        log ""
        log "[$script_num] Starting: $script"
        
        if run_script "$script"; then
            mark_script_complete "$script"
            ((success_count++))
        else
            ((failed_count++))
            log "⚠ Script will be retried on next boot"
        fi
        
        ((script_num++))
    done < "$RUNLIST"
    
    # Summary
    log ""
    log "========================================"
    log "First Boot Summary"
    log "========================================"
    log "✓ Successful: $success_count"
    log "❌ Failed: $failed_count"
    
    if [ $failed_count -eq 0 ]; then
        log ""
        log "✅ All scripts completed successfully!"
        log "Disabling first-boot service..."
        systemctl disable firstboot.service 2>/dev/null || true
    else
        log ""
        log "⚠ Some scripts failed - will retry on next boot"
        log "Service remains enabled for retry"
    fi
    
    log "========================================"
}

# Run main
main "$@"