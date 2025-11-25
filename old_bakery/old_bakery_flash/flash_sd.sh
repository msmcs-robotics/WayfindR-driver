#!/usr/bin/env bash

set -euo pipefail

### ---- Helper Functions ----

err() {
    echo "ERROR: $*" >&2
}

confirm() {
    read -p "$1 [y/N]: " ans
    [[ "${ans,,}" == "y" ]]
}

### ---- Collect Inputs ----

read -rp "Full path to OS image: " IMG
[[ ! -f "$IMG" ]] && err "Image file not found." && exit 1

clear
echo "Available storage devices:"
lsblk -o NAME,SIZE,TYPE,MOUNTPOINT

echo
confirm "Have you connected the SD card/device?" || exit 1
clear
lsblk -o NAME,SIZE,TYPE,MOUNTPOINT

echo
read -rp "Target device (ex: sda, sdb, nvme0n1, mmcblk0): " DEV
DEV="/dev/$DEV"

### ---- Safety Checks ----

# Check device exists
if [[ ! -b "$DEV" ]]; then
    err "Device $DEV not found."
    exit 1
fi

# Auto-block flashing currently booted root device
ROOT_DEV=$(df / | tail -1 | awk '{print $1}' | sed 's/[0-9]*$//')
if [[ "$ROOT_DEV" == "$DEV" ]]; then
    err "Refusing to flash the currently-running root device: $DEV"
    exit 1
fi

echo
echo "!!! WARNING — This will COMPLETELY WIPE $DEV !!!"
lsblk "$DEV"
confirm "Are you absolutely sure you want to continue flashing this device?" || exit 1


### ---- Unmount partitions if needed ----

echo
echo "Checking mounted partitions..."
while read -r part; do
    if mount | grep -q "^$part"; then
        echo "Unmounting $part..."
        sudo umount "$part"
    fi
done < <(lsblk -ln "$DEV" | awk '{print "/dev/"$1}')


### ---- Flash Image ----

echo
echo "Flashing image..."
sudo dd if="$IMG" of="$DEV" bs=4M conv=fsync status=progress
sync
echo "Flash complete."

echo
echo "✅ Done."
echo "Image flashed: $IMG"
echo "Drive: $DEV"
