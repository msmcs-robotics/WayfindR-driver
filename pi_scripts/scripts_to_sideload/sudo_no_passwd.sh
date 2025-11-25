#!/bin/bash
# setup-passwordless-sudo.sh
# Sideloader script to enable passwordless sudo for configured user
# Idempotent - safe to run multiple times

set -e

SCRIPT_NAME="setup-passwordless-sudo"

# Logging function
log() {
    echo "[$SCRIPT_NAME] $1"
}

# ═══════════════════════════════════════════════════════════════════════════
# STEP 1: Get configured username (CRITICAL - script runs as root!)
# ═══════════════════════════════════════════════════════════════════════════
USERNAME=$(jq -r '.username' /tmp/baker-config.json 2>/dev/null || echo "pi")

log "Starting passwordless sudo setup for user: $USERNAME"

# ═══════════════════════════════════════════════════════════════════════════
# STEP 2: Verify user exists
# ═══════════════════════════════════════════════════════════════════════════
if ! id "$USERNAME" &>/dev/null; then
    log "❌ User '$USERNAME' does not exist on the system"
    log "User must be created before running this script"
    exit 1
fi

log "✓ User '$USERNAME' exists"

# ═══════════════════════════════════════════════════════════════════════════
# STEP 3: Check if passwordless sudo is already configured (IDEMPOTENCY)
# ═══════════════════════════════════════════════════════════════════════════
SUDOERS_DIR="/etc/sudoers.d"
SUDOERS_FILE="${SUDOERS_DIR}/${USERNAME}"

# Check if the file exists and contains the NOPASSWD rule
if [[ -f "$SUDOERS_FILE" ]]; then
    if grep -q "^${USERNAME}.*NOPASSWD.*ALL" "$SUDOERS_FILE"; then
        log "✓ Passwordless sudo already configured for $USERNAME"
        log "File: $SUDOERS_FILE"
        exit 0
    else
        log "⚠ Sudoers file exists but doesn't have NOPASSWD rule, will update"
    fi
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 4: Create backup directory
# ═══════════════════════════════════════════════════════════════════════════
BACKUP_DIR="/var/backups/sudoers.d"
mkdir -p "$BACKUP_DIR"

# Backup existing file if present
if [[ -e "$SUDOERS_FILE" ]]; then
    TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
    BACKUP_FILE="${BACKUP_DIR}/${USERNAME}.${TIMESTAMP}"
    log "Backing up existing sudoers file to: $BACKUP_FILE"
    cp -a "$SUDOERS_FILE" "$BACKUP_FILE"
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 5: Create sudoers file with NOPASSWD rule
# ═══════════════════════════════════════════════════════════════════════════
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"

log "Creating passwordless sudo configuration..."

cat > "$SUDOERS_FILE" <<EOF
# Created by Pi Script Sideloader on $TIMESTAMP
# Allow $USERNAME to run any command via sudo without a password
$USERNAME ALL=(ALL) NOPASSWD:ALL
EOF

# ═══════════════════════════════════════════════════════════════════════════
# STEP 6: Set correct permissions (CRITICAL for sudoers files!)
# ═══════════════════════════════════════════════════════════════════════════
chmod 0440 "$SUDOERS_FILE"
chown root:root "$SUDOERS_FILE"

log "✓ Sudoers file created with correct permissions (0440)"

# ═══════════════════════════════════════════════════════════════════════════
# STEP 7: Validate sudoers file syntax (CRITICAL for system stability!)
# ═══════════════════════════════════════════════════════════════════════════
log "Validating sudoers file syntax..."

if visudo -cf "$SUDOERS_FILE"; then
    log "✓ Sudoers syntax validation passed"
else
    log "❌ Sudoers syntax validation FAILED!"
    log "This is a critical error - restoring backup if available"
    
    # Restore backup if it exists
    if [[ -e "$BACKUP_FILE" ]]; then
        cp -a "$BACKUP_FILE" "$SUDOERS_FILE"
        chmod 0440 "$SUDOERS_FILE"
        chown root:root "$SUDOERS_FILE"
        log "✓ Backup restored"
    else
        # No backup, remove the invalid file
        rm -f "$SUDOERS_FILE"
        log "Invalid file removed"
    fi
    
    exit 1
fi

# ═══════════════════════════════════════════════════════════════════════════
# STEP 8: Success!
# ═══════════════════════════════════════════════════════════════════════════
log ""
log "════════════════════════════════════════════════════════════"
log "✅ Passwordless sudo successfully configured!"
log "════════════════════════════════════════════════════════════"
log "User: $USERNAME"
log "Config file: $SUDOERS_FILE"
log ""
log "The user can now run sudo commands without a password."
log "To test: sudo -u $USERNAME sudo whoami"
log ""
log "To remove this configuration later:"
log "  sudo rm -f $SUDOERS_FILE"
log "  sudo visudo -c"
log "════════════════════════════════════════════════════════════"

exit 0