#!/usr/bin/env bash

### Idempotent installer for xRDP + XFCE on Ubuntu 22.04+
### Argument-driven: --install to apply changes, --check to just report status
### Ensures xRDP always uses XFCE, local login can choose GNOME/Ubuntu
### Installs/configures LightDM if needed for XFCE

set -e

show_help() {
    echo "Usage: $0 [--install | --check]"
    echo
    echo "Options:"
    echo "  --install    Install/configure xRDP + XFCE (idempotent)"
    echo "  --check      Check current configuration without making changes"
    echo
    exit 1
}

if [[ $# -eq 0 ]]; then
    show_help
fi

MODE="$1"
if [[ "$MODE" != "--install" && "$MODE" != "--check" ]]; then
    show_help
fi

DO_INSTALL=false
[[ "$MODE" == "--install" ]] && DO_INSTALL=true

echo "==== Mode: $MODE ===="

################################################################################
# REQUIRED PACKAGES
################################################################################

PKGS=(
    xrdp
    xfce4
    xorgxrdp
)

echo "==== Checking required packages ===="
for pkg in "${PKGS[@]}"; do
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo "[OK] $pkg already installed"
    else
        if $DO_INSTALL; then
            echo "[INSTALL] $pkg"
            sudo apt install -y "$pkg"
        else
            echo "[MISSING] $pkg not installed"
        fi
    fi
done

################################################################################
# Install LightDM if missing
################################################################################
if dpkg -l | grep -q "^ii  lightdm"; then
    echo "[OK] LightDM already installed"
else
    if $DO_INSTALL; then
        echo "==== Installing LightDM ===="
        sudo apt install -y lightdm
        echo "==== Configuring LightDM as default display manager ===="
        sudo dpkg-reconfigure lightdm
    else
        echo "[MISSING] LightDM not installed (would install)"
    fi
fi

################################################################################
# Fix Wayland issue in GDM (still for RDP compatibility)
################################################################################
GDM_CONF="/etc/gdm3/custom.conf"
if grep -q "^WaylandEnable=false" "$GDM_CONF"; then
    echo "[OK] Wayland already disabled in $GDM_CONF"
else
    if $DO_INSTALL; then
        echo "==== Disabling Wayland for GDM ===="
        sudo sed -i 's/^#WaylandEnable=false/WaylandEnable=false/' "$GDM_CONF"
    else
        echo "[MISSING] WaylandEnable=false not set in $GDM_CONF"
    fi
fi

################################################################################
# Remove ~/.xsession (force for all logins)
################################################################################
if [[ -f "$HOME/.xsession" ]]; then
    if $DO_INSTALL; then
        echo "==== Removing ~/.xsession to allow local DE choice ===="
        rm -f "$HOME/.xsession"
    else
        echo "[FOUND] ~/.xsession exists (would remove)"
    fi
else
    echo "[OK] No ~/.xsession to remove"
fi

################################################################################
# Ensure XFCE session file exists
################################################################################
XFCE_DESKTOP="/usr/share/xsessions/xfce.desktop"
if [[ -f "$XFCE_DESKTOP" ]]; then
    echo "[OK] $XFCE_DESKTOP already exists"
else
    if $DO_INSTALL; then
        echo "==== Creating $XFCE_DESKTOP ===="
        sudo tee "$XFCE_DESKTOP" > /dev/null <<EOF
[Desktop Entry]
Name=Xfce
Comment=Use the Xfce Desktop environment
Exec=startxfce4
Type=Application
EOF
    else
        echo "[MISSING] $XFCE_DESKTOP does not exist (would create)"
    fi
fi

################################################################################
# Configure xRDP to force XFCE
################################################################################
XRDP_STARTWM="/etc/xrdp/startwm.sh"
if grep -q "startxfce4" "$XRDP_STARTWM" 2>/dev/null; then
    echo "[OK] xRDP already configured to use XFCE in $XRDP_STARTWM"
else
    if $DO_INSTALL; then
        echo "==== Configuring xRDP to start XFCE ===="
        sudo tee "$XRDP_STARTWM" > /dev/null <<'EOF'
#!/bin/sh
# Force XFCE for xRDP sessions

# Load environment
if test -r /etc/profile; then
    . /etc/profile
fi

# Start XFCE
startxfce4
EOF
        sudo chmod +x "$XRDP_STARTWM"
    else
        echo "[MISSING] xRDP startwm.sh not configured to use XFCE (would configure)"
    fi
fi

################################################################################
# Ensure Xwrapper settings allow xRDP
################################################################################
XWRAPPER="/etc/X11/Xwrapper.config"
if [[ -f "$XWRAPPER" ]]; then
    grep -q "^allowed_users=anybody" "$XWRAPPER" \
        && echo "[OK] allowed_users already set to anybody" \
        || { echo "[CHECK] allowed_users not set to anybody"; $DO_INSTALL && sudo sed -i 's/^allowed_users=.*/allowed_users=anybody/' "$XWRAPPER"; }
    grep -q "^needs_root_rights=no" "$XWRAPPER" \
        && echo "[OK] needs_root_rights already set to no" \
        || { echo "[CHECK] needs_root_rights not set to no"; $DO_INSTALL && sudo sed -i 's/^needs_root_rights=.*/needs_root_rights=no/' "$XWRAPPER"; }
else
    if $DO_INSTALL; then
        echo "==== Creating Xwrapper default config ===="
        cat <<EOF | sudo tee "$XWRAPPER"
allowed_users=anybody
needs_root_rights=no
EOF
    else
        echo "[MISSING] $XWRAPPER does not exist (would create)"
    fi
fi

################################################################################
# Add user to ssl-cert
################################################################################
if groups "$USER" | grep -q "\bssl-cert\b"; then
    echo "[OK] $USER already in ssl-cert group"
else
    if $DO_INSTALL; then
        echo "==== Adding $USER to ssl-cert group ===="
        sudo adduser "$USER" ssl-cert
    else
        echo "[MISSING] $USER not in ssl-cert group (would add)"
    fi
fi

################################################################################
# Enable + start xRDP
################################################################################
if systemctl is-active --quiet xrdp; then
    echo "[OK] xRDP service active"
else
    if $DO_INSTALL; then
        echo "==== Enabling + starting xRDP ===="
        sudo systemctl enable xrdp
        sudo systemctl restart xrdp
    else
        echo "[CHECK] xRDP service not active (would enable/start)"
    fi
fi

################################################################################
# Status
################################################################################
echo
echo "==== STATUS ===="
systemctl is-active --quiet xrdp && \
    echo "xRDP is ACTIVE" || echo "xRDP is NOT active"

echo
echo "==== DONE ===="
if $DO_INSTALL; then
    echo "✅ xRDP + XFCE installation completed"
    echo "✅ Local logins can choose GNOME or XFCE at login screen"
    echo "✅ RDP logins will automatically use XFCE"
    echo "✅ LightDM installed for XFCE stability"
    echo "✅ Reboot recommended"
else
    echo "✅ Configuration check complete (no changes made)"
fi
