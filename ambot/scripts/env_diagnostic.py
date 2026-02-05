#!/usr/bin/env python3
"""
Environment Diagnostic Script for AMBOT

Run this script from BOTH SSH and the desktop terminal to compare environments.
Helps diagnose why packages work in one but not the other.

Usage:
    python3 scripts/env_diagnostic.py          # Human-readable output
    python3 scripts/env_diagnostic.py --json   # JSON output for automated comparison
    python3 scripts/env_diagnostic.py --fix    # Attempt to fix common issues
"""

import sys
import os
import json
import subprocess
from pathlib import Path


def get_diagnostics():
    """Collect all environment diagnostics."""
    diag = {}

    # Basic info
    diag["python_version"] = sys.version
    diag["python_executable"] = sys.executable
    diag["python_prefix"] = sys.prefix
    diag["python_base_prefix"] = sys.base_prefix
    diag["in_virtualenv"] = sys.prefix != sys.base_prefix
    diag["user"] = os.environ.get("USER", "unknown")
    diag["home"] = os.environ.get("HOME", "unknown")
    diag["cwd"] = os.getcwd()
    diag["uid"] = os.getuid()
    diag["euid"] = os.geteuid()

    # Shell and terminal
    diag["shell"] = os.environ.get("SHELL", "unknown")
    diag["term"] = os.environ.get("TERM", "unknown")
    diag["display"] = os.environ.get("DISPLAY", "not set")
    diag["wayland_display"] = os.environ.get("WAYLAND_DISPLAY", "not set")
    diag["xdg_session_type"] = os.environ.get("XDG_SESSION_TYPE", "not set")
    diag["desktop_session"] = os.environ.get("DESKTOP_SESSION", "not set")
    diag["ssh_connection"] = os.environ.get("SSH_CONNECTION", "not set")

    # Python paths
    diag["sys_path"] = sys.path
    diag["path_env"] = os.environ.get("PATH", "")
    diag["pythonpath"] = os.environ.get("PYTHONPATH", "not set")
    diag["pythonnousersite"] = os.environ.get("PYTHONNOUSERSITE", "not set")
    diag["pythonhome"] = os.environ.get("PYTHONHOME", "not set")

    # Site packages
    try:
        import site
        diag["user_site"] = site.getusersitepackages()
        diag["user_site_enabled"] = site.ENABLE_USER_SITE
        diag["site_packages"] = site.getsitepackages()
    except Exception as e:
        diag["site_error"] = str(e)

    # Check user .local directory
    local_site = Path.home() / ".local" / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
    diag["user_local_exists"] = local_site.exists()
    diag["user_local_path"] = str(local_site)
    if local_site.exists():
        diag["user_local_contents"] = sorted([p.name for p in local_site.iterdir() if not p.name.startswith("__")])[:30]

    # Package checks
    packages = {
        "cv2": "opencv-python-headless",
        "numpy": "numpy",
        "matplotlib": "matplotlib",
        "serial": "pyserial",
        "tkinter": "python3-tk (apt)",
        "rplidar": "rplidar-roboticia",
        "RPi.GPIO": "RPi.GPIO",
    }

    diag["packages"] = {}
    for import_name, pkg_name in packages.items():
        try:
            mod = __import__(import_name)
            version = getattr(mod, "__version__", getattr(mod, "VERSION", "unknown"))
            location = getattr(mod, "__file__", "unknown")
            diag["packages"][import_name] = {
                "status": "ok",
                "version": str(version),
                "location": str(location),
                "pkg_name": pkg_name,
            }
        except ImportError as e:
            diag["packages"][import_name] = {
                "status": "missing",
                "error": str(e),
                "pkg_name": pkg_name,
            }
        except Exception as e:
            diag["packages"][import_name] = {
                "status": "error",
                "error": str(e),
                "pkg_name": pkg_name,
            }

    # Check system vs user package locations for key packages
    # Maps import name -> possible directory/file names to look for
    pkg_dir_names = {
        "cv2": ["cv2"],
        "numpy": ["numpy"],
        "matplotlib": ["matplotlib"],
        "serial": ["serial"],
        "rplidar": ["rplidar.py", "rplidar"],
    }
    diag["package_locations"] = {}
    for check_dir, label in [
        (str(local_site), "user_local"),
        ("/usr/local/lib/python3.13/dist-packages", "system_local"),
        ("/usr/lib/python3/dist-packages", "system"),
    ]:
        found = []
        if Path(check_dir).exists():
            for import_name, dir_names in pkg_dir_names.items():
                for dn in dir_names:
                    if (Path(check_dir) / dn).exists():
                        found.append(import_name)
                        break
        diag["package_locations"][label] = {"path": check_dir, "found": found}

    return diag


def print_human_readable(diag):
    """Print diagnostics in human-readable format."""
    print("=" * 60)
    print("  AMBOT Environment Diagnostic")
    print("=" * 60)
    print()

    print("--- System ---")
    print(f"  User:            {diag['user']} (uid={diag['uid']}, euid={diag['euid']})")
    print(f"  Home:            {diag['home']}")
    print(f"  Shell:           {diag['shell']}")
    print(f"  Terminal:        {diag['term']}")
    print(f"  CWD:             {diag['cwd']}")
    print()

    print("--- Session ---")
    print(f"  DISPLAY:         {diag['display']}")
    print(f"  WAYLAND_DISPLAY: {diag['wayland_display']}")
    print(f"  XDG_SESSION:     {diag['xdg_session_type']}")
    print(f"  DESKTOP_SESSION: {diag['desktop_session']}")
    ssh = diag['ssh_connection']
    print(f"  SSH_CONNECTION:   {ssh}")
    is_ssh = ssh != "not set"
    print(f"  Running via:     {'SSH' if is_ssh else 'LOCAL TERMINAL'}")
    print()

    print("--- Python ---")
    print(f"  Version:         {diag['python_version'].split()[0]}")
    print(f"  Executable:      {diag['python_executable']}")
    print(f"  Prefix:          {diag['python_prefix']}")
    print(f"  In virtualenv:   {diag['in_virtualenv']}")
    print(f"  PYTHONPATH:      {diag['pythonpath']}")
    print(f"  PYTHONNOUSERSITE:{diag['pythonnousersite']}")
    print(f"  PYTHONHOME:      {diag['pythonhome']}")
    print()

    # User site packages
    user_site_enabled = diag.get("user_site_enabled", "unknown")
    user_local_exists = diag.get("user_local_exists", False)
    print("--- User Site Packages ---")
    print(f"  Enabled:         {user_site_enabled}")
    print(f"  Path:            {diag.get('user_site', 'unknown')}")
    print(f"  Dir exists:      {user_local_exists}")
    if not user_site_enabled:
        print("  ** WARNING: User site packages DISABLED! **")
        print("  ** This is likely why packages aren't found from desktop! **")
    print()

    print("--- sys.path ---")
    for i, p in enumerate(diag["sys_path"]):
        marker = ""
        if ".local" in p:
            marker = " <-- USER LOCAL"
        elif "dist-packages" in p:
            marker = " <-- SYSTEM"
        print(f"  [{i}] {p}{marker}")
    print()

    print("--- Package Status ---")
    all_ok = True
    for name, info in diag["packages"].items():
        if info["status"] == "ok":
            loc = info["location"]
            if ".local" in loc:
                loc_type = "(user-local)"
            elif "/usr/local" in loc:
                loc_type = "(system-local)"
            elif "/usr/lib" in loc:
                loc_type = "(system)"
            else:
                loc_type = ""
            print(f"  OK    {name:15s} v{info['version']:15s} {loc_type}")
        elif info["status"] == "missing":
            print(f"  MISS  {name:15s} -- {info['error']}")
            all_ok = False
        else:
            print(f"  ERR   {name:15s} -- {info['error']}")
            all_ok = False
    print()

    print("--- Package Locations ---")
    for label, info in diag["package_locations"].items():
        found = info["found"]
        print(f"  {label:15s} ({info['path']})")
        if found:
            print(f"                  Found: {', '.join(found)}")
        else:
            print(f"                  (none of our packages)")
    print()

    # Diagnosis
    print("--- Diagnosis ---")
    issues = []

    if diag.get("in_virtualenv"):
        issues.append("Running inside a virtual environment - packages may be in a different location")

    if diag.get("pythonnousersite") not in ("not set", None, ""):
        issues.append(f"PYTHONNOUSERSITE is set to '{diag['pythonnousersite']}' - user site-packages DISABLED")

    if diag.get("pythonhome") not in ("not set", None, ""):
        issues.append(f"PYTHONHOME is set to '{diag['pythonhome']}' - may redirect package search")

    if not user_site_enabled:
        issues.append("site.ENABLE_USER_SITE is False - ~/.local packages won't be found")

    if not user_local_exists:
        issues.append(f"User local directory doesn't exist: {diag.get('user_local_path')}")

    # Check if key packages are ONLY in user-local
    user_only = []
    for name, info in diag["packages"].items():
        if info["status"] == "ok" and ".local" in info.get("location", ""):
            # Check if it's also in system paths
            in_system = False
            for loc_info in diag["package_locations"].values():
                if loc_info["path"] != diag.get("user_local_path") and name in loc_info.get("found", []):
                    in_system = True
                    break
            if not in_system:
                user_only.append(name)

    if user_only:
        issues.append(
            f"These packages are ONLY in user-local (~/.local): {', '.join(user_only)}\n"
            "    If the desktop session disables user site-packages, they won't be found.\n"
            "    Fix: Install system-wide with: sudo pip3 install --break-system-packages <package>"
        )

    if not all_ok:
        missing = [n for n, i in diag["packages"].items() if i["status"] != "ok"]
        issues.append(f"Missing packages: {', '.join(missing)}")

    if issues:
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")
    else:
        print("  No issues detected! All packages available.")
    print()

    # Recommended fix
    if user_only or not all_ok:
        print("--- Recommended Fix ---")
        print("  Run the following on the RPi to install packages system-wide:")
        print()
        print("  sudo pip3 install --break-system-packages \\")
        print("    opencv-python-headless matplotlib numpy pyserial rplidar-roboticia")
        print()
        print("  Or run: sudo ./install.sh --gui --pathfinder")
        print()

    return len(issues) == 0


def attempt_fix():
    """Attempt to fix common issues by installing packages system-wide."""
    print("Attempting to fix package availability...")
    print()

    if os.geteuid() != 0:
        print("ERROR: This fix requires root. Run with: sudo python3 scripts/env_diagnostic.py --fix")
        return False

    packages = [
        "opencv-python-headless",
        "matplotlib",
        "numpy",
        "pyserial",
        "rplidar-roboticia",
    ]

    for pkg in packages:
        print(f"  Installing {pkg} system-wide...")
        result = subprocess.run(
            [sys.executable, "-m", "pip", "install", "--break-system-packages", pkg],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"    OK: {pkg}")
        else:
            print(f"    FAILED: {pkg}")
            print(f"    {result.stderr.strip()[:200]}")

    print()
    print("Done. Re-run this script without --fix to verify.")
    return True


def main():
    json_mode = "--json" in sys.argv
    fix_mode = "--fix" in sys.argv

    if fix_mode:
        attempt_fix()
        return 0

    diag = get_diagnostics()

    if json_mode:
        print(json.dumps(diag, indent=2, default=str))
    else:
        ok = print_human_readable(diag)
        return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main() or 0)
