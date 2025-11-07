#!/usr/bin/env bash
set -euo pipefail

# allow-nopasswd-sudo.sh
# Usage: sudo ./allow-nopasswd-sudo.sh [username]
# If username is omitted, defaults to the invoking user ($SUDO_USER) or $USER.

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root (sudo)." >&2
  exit 2
fi

# Determine target user
TARGET_USER="${1:-}"

if [[ -z "$TARGET_USER" ]]; then
  # Prefer SUDO_USER if available, else USER
  TARGET_USER="${SUDO_USER:-${USER:-}}"
fi

if [[ -z "$TARGET_USER" ]]; then
  echo "Unable to determine target username. Provide one: $0 <username>" >&2
  exit 2
fi

# verify user exists on system
if ! id "$TARGET_USER" &>/dev/null; then
  echo "User '$TARGET_USER' does not exist." >&2
  exit 3
fi

SUDOERS_DIR="/etc/sudoers.d"
OUTFILE="${SUDOERS_DIR}/${TARGET_USER}"
BACKUP_DIR="/var/backups/sudoers.d"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"

# create backup dir if needed
mkdir -p "$BACKUP_DIR"

# if file exists, back it up
if [[ -e "$OUTFILE" ]]; then
  echo "Backing up existing $OUTFILE -> ${BACKUP_DIR}/$(basename "$OUTFILE").${TIMESTAMP}"
  cp -a "$OUTFILE" "${BACKUP_DIR}/$(basename "$OUTFILE").${TIMESTAMP}"
fi

# Write the nopasswd rule
cat > "$OUTFILE" <<EOF
# Added by allow-nopasswd-sudo.sh on $TIMESTAMP
# Allow $TARGET_USER to run any command via sudo without a password
$TARGET_USER ALL=(ALL) NOPASSWD:ALL
EOF

# set strict permissions
chmod 0440 "$OUTFILE"
chown root:root "$OUTFILE"

# Validate new sudoers file(s)
if visudo -cf "$OUTFILE"; then
  echo "Syntax OK for $OUTFILE"
else
  echo "visudo reported syntax errors. Restoring backup (if present) and aborting." >&2
  if [[ -e "${BACKUP_DIR}/$(basename "$OUTFILE").${TIMESTAMP}" ]]; then
    cp -a "${BACKUP_DIR}/$(basename "$OUTFILE").${TIMESTAMP}" "$OUTFILE"
    chmod 0440 "$OUTFILE"
    chown root:root "$OUTFILE"
    echo "Backup restored."
  fi
  exit 4
fi

echo "Passwordless sudo granted permanently for user: $TARGET_USER"
echo "To remove it, delete the file: sudo rm -f $OUTFILE"


# undo this quickly
# remove the nopasswd entry for user 'alice'
# sudo rm -f /etc/sudoers.d/alice
# then optionally run:
# sudo visudo -c
