#!/usr/bin/env python3
import json
import os
import shutil
import sys
from datetime import datetime

CONFIG_NAME = "bakery_config"
BAKERY_DIR = "bakery"
TARGET_OPT = "opt"
CUSTOM_DIR = "custom"


def load_config(path):
    with open(path, 'r') as f:
        return json.load(f)


def timestamp():
    return datetime.now().strftime("%Y%m%d%H%M")


def set_hostname(mount_point, base_name=None):
    name = base_name if base_name else "raspberrypi"
    name = f"{name}{timestamp()}"
    with open(os.path.join(mount_point, "etc/hostname"), 'w') as f:
        f.write(name + "\n")
    return name


def copy_bakery(mount_point):
    src = BAKERY_DIR
    dst = os.path.join(mount_point, TARGET_OPT, BAKERY_DIR)
    shutil.copytree(src, dst, dirs_exist_ok=True)


def copy_custom_scripts(mount_point, scripts):
    if not scripts:
        return
    dest_root = os.path.join(mount_point, TARGET_OPT, BAKERY_DIR, CUSTOM_DIR)
    os.makedirs(dest_root, exist_ok=True)
    for script in scripts:
        if os.path.exists(script):
            dst = os.path.join(dest_root, os.path.basename(script))
            shutil.copy2(script, dst)
        else:
            print(f"Warning: Custom script not found: {script}")


def store_runlist(mount_point, scripts):
    if not scripts:
        return
    runlist_path = os.path.join(mount_point, TARGET_OPT, BAKERY_DIR, "runlist.txt")
    with open(runlist_path, 'w') as f:
        for script in scripts:
            f.write(os.path.basename(script) + "\n")


def main():
    if len(sys.argv) < 3:
        print("Usage: pre-bake.py <config.json> <mount-point>")
        sys.exit(1)

    config_path = sys.argv[1]
    mount_point = sys.argv[2]

    cfg = load_config(config_path)

    # Copy bakery folder
    copy_bakery(mount_point)

    # Write config
    cfg_path = os.path.join(mount_point, TARGET_OPT, BAKERY_DIR, CONFIG_NAME)
    with open(cfg_path, 'w') as f:
        json.dump(cfg, f, indent=2)

    # Copy custom scripts if exist
    custom_scripts = cfg.get("custom_scripts", [])
    copy_custom_scripts(mount_point, custom_scripts)

    # Write runlist
    store_runlist(mount_point, custom_scripts)

    # Set hostname
    new_name = set_hostname(mount_point, cfg.get("hostname"))
    print(f"Hostname set to: {new_name}")

    print("Pre-bake completed.")


if __name__ == "__main__":
    main()
