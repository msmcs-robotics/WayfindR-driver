# Old Bakery - Scope Documentation

**Created:** 2026-01-11
**Status:** Deprecated / Legacy Code
**Replacement:** See `/new_bakery` directory

---

## What This Folder Is For

The `old_bakery` folder contains the original, feature-rich approach to automating Raspberry Pi SD card preparation for the WayfindR robotics project. It consists of two main components:

1. **SD Card Sideloader** (`bakery_old.py`) - A PyQt5 GUI application for copying scripts to already-flashed SD cards
2. **Complete Pre-Baker** (`old_bakery_flash/`) - A comprehensive GUI tool for wiping, flashing, and configuring Raspberry Pi SD cards

### Primary Use Cases
- Flash Raspberry Pi OS images to SD cards with GUI workflow
- Automatically configure first-boot scripts and settings
- Manage custom scripts that run on first boot
- Pre-configure networking, SSH, hardware interfaces (I2C, SPI)
- Support multiple wipe methods (quick, secure, zero-fill)

---

## Architectural Overview

### Component 1: SD Card Sideloader (`bakery_old.py`)
**Lines of Code:** 714
**Dependencies:** PyQt5, subprocess, json, pathlib

**Purpose:** A GUI application that:
- Automatically detects and mounts SD cards
- Allows adding custom scripts in execution order
- Copies scripts to `/opt/bakery/` on mounted SD cards
- Creates systemd service for first-boot execution
- Generates configuration files (`baker-config.json`, `runlist.txt`)
- Enables SSH on boot partition

**Key Features:**
- Automatic disk detection using `lsblk`
- Safe mounting/unmounting with temporary mount points
- Script ordering and management (move up/down, add/remove)
- Username configuration
- Auto-unmounting after successful copy

### Component 2: Complete Pre-Baker (`old_bakery_flash/`)
**Total Lines of Code:** ~1,683 (workers/utils) + 801 (GUI) = ~2,484+ lines

**Architecture:**
```
old_bakery_flash/
├── main.py                      # Entry point (40 lines)
├── gui/main_window.py           # Main GUI (801 lines)
├── workers/
│   ├── wipe_worker.py          # Disk wiping (412 lines)
│   ├── flash_worker.py         # Image flashing (382 lines)
│   └── config_worker.py        # Post-flash config (569 lines)
├── utils/
│   └── disk_utils.py           # Disk operations (320 lines)
├── firstboot.sh                # First-boot orchestrator (194 lines)
├── pre-bake.py                 # Alternative config tool (93 lines)
├── flash_sd.sh                 # Standalone flash script (79 lines)
├── prompts*.md                 # Development conversation logs
└── notes.md                    # Technical notes (17KB)
```

**Key Features:**
- **Three-phase process:** Wipe → Flash → Configure
- **Multiple wipe methods:**
  - Quick: Partition table + first 50MB
  - Secure: 1-pass random data
  - Zero: Complete disk overwrite
- **Comprehensive configuration:**
  - User creation with password/sudo settings
  - WiFi pre-configuration (SSID/password)
  - Hardware interface enablement (I2C, SPI)
  - Hostname with timestamp
  - Custom script management with execution order
  - Working directory creation
- **Safety features:**
  - System disk detection and blocking
  - Confirmation dialogs
  - Source image protection
  - Safe unmounting
- **Progress tracking:**
  - Real-time progress bars
  - Detailed status logging
  - Operation cancellation support

---

## How It Differs from `new_bakery`

| Aspect | Old Bakery | New Bakery |
|--------|------------|------------|
| **Approach** | GUI-based (PyQt5) | CLI-based (bash) |
| **Complexity** | High (~2,500+ LOC Python) | Low (~571 LOC bash) |
| **Dependencies** | PyQt5, Python 3, multiple modules | Standard Linux tools, cloud-init |
| **Flash Method** | Custom Python with dd | Direct dd with compression support |
| **Configuration** | Custom systemd + manual file copying | Ubuntu cloud-init (built-in) |
| **First Boot** | Custom state tracking system | Cloud-init + simple systemd service |
| **Network Setup** | Manual wpa_supplicant editing | Cloud-init network-config |
| **User Experience** | Point-and-click GUI | Command-line arguments |
| **Maintenance** | Difficult (GUI + workers + state) | Simple (single bash script) |
| **OS Support** | Any Raspberry Pi OS | Ubuntu 22.04 for Pi (cloud-init) |
| **Script State** | Tracks individual script completion | All-or-nothing approach |
| **Disk Wiping** | 3 methods with GUI selection | Not included (relies on flash) |
| **WiFi Config** | GUI form inputs | Command-line flags |
| **Testing** | Requires GUI environment | Easy to script/automate |

### Key Philosophical Differences

**Old Bakery Philosophy:**
- "Do everything in one comprehensive tool"
- "Provide maximum flexibility and options"
- "Guide users through every step with GUI"
- "Handle all edge cases"

**New Bakery Philosophy:**
- "Leverage existing Ubuntu infrastructure (cloud-init)"
- "Keep it simple and maintainable"
- "One tool, one job, done well"
- "Shell scripts are self-documenting"

---

## Why It's Marked as "Old"

### Reasons for Deprecation

1. **Over-Engineering**
   - 2,500+ lines of Python for a task achievable in ~600 lines of bash
   - Complex worker threading for operations that could be linear
   - GUI overhead for a setup task typically done once per SD card

2. **Maintenance Burden**
   - PyQt5 dependency management
   - Multiple Python files requiring coordination
   - State tracking complexity (firstboot.state, baker-config.json, runlist.txt)
   - Difficult to debug GUI threading issues

3. **Ubuntu Cloud-Init Makes It Redundant**
   - Ubuntu 22.04 for Raspberry Pi has cloud-init built-in
   - Cloud-init is the standard way to configure Ubuntu on first boot
   - No need to reinvent user creation, SSH setup, network config
   - Industry-standard, well-documented, well-tested

4. **Development Friction**
   - Hard to test without SD card and GUI environment
   - Difficult to automate in CI/CD
   - Prompt files suggest iterative development struggles
   - Multiple rewrites (prompts2.md, prompts3.md, pre-bake-prompts.md)

5. **Scope Creep**
   - Started as simple script sideloader
   - Grew into full disk management suite
   - Feature bloat (HTML config generator, network testing)
   - Mission drift from core robotics focus

### What Was Learned

The old_bakery development process taught valuable lessons:

- **Use existing standards** - Don't reinvent cloud-init
- **CLI over GUI for setup tasks** - More scriptable, testable
- **Simple bash beats complex Python** - For system tasks
- **State tracking is complex** - Avoid if possible
- **Documentation in prompts** - Good for understanding evolution

---

## Key Files and Their Purposes

### Root Level

| File | Purpose | Status |
|------|---------|--------|
| `bakery_old.py` | Original SD card script sideloader GUI | Replaced by new_bakery |
| `test_net.sh` | Network diagnostic testing script | Standalone utility - KEEP |
| `prompts4.md` | Fleet management planning document | Historical - ARCHIVE |

### old_bakery_flash/

| File | Purpose | Replacement in new_bakery |
|------|---------|---------------------------|
| `main.py` | GUI application entry point | `bake.sh` (main script) |
| `gui/main_window.py` | Main PyQt5 interface | Command-line interface |
| `workers/wipe_worker.py` | Threaded disk wiping | Not needed (dd handles) |
| `workers/flash_worker.py` | Threaded image flashing | Inline in bake.sh |
| `workers/config_worker.py` | Post-flash configuration | cloud-init user-data |
| `utils/disk_utils.py` | Disk detection utilities | lsblk + findmnt |
| `firstboot.sh` | First-boot orchestrator | Simpler version in cloud-init |
| `pre-bake.py` | Config-file based setup | bake.sh flags |
| `flash_sd.sh` | Standalone flash script | bake.sh flash phase |
| `bakery_config_generator.html` | Web-based config UI | Not needed |
| `notes.md` | Technical documentation | FINDINGS.md, WORKFLOW.md |
| `prompts*.md` | Development conversations | Historical reference |

---

## Should It Be Archived or Kept?

### Recommendation: **ARCHIVE**

**Keep in version control but mark as deprecated.**

### What to Keep Accessible

1. **`test_net.sh`** - Move to `/utils` or `/scripts` directory
   - Useful standalone network diagnostic tool
   - Well-written, no dependencies on bakery system
   - Works independently

2. **Lessons Learned**
   - Document key insights in main project README
   - Reference this scope.md for historical understanding

### What to Archive

1. **All Python GUI code** - Superseded by new_bakery
2. **Worker and utility modules** - No longer needed
3. **Prompt/conversation logs** - Interesting history but not operational
4. **HTML config generator** - Never integrated, not used

### Suggested Actions

```bash
# Create archive directory
mkdir -p /home/devel/Desktop/WayfindR-driver/archive/old_bakery_2025

# Move old_bakery there
mv /home/devel/Desktop/WayfindR-driver/old_bakery/* \
   /home/devel/Desktop/WayfindR-driver/archive/old_bakery_2025/

# Keep this documentation
cp /home/devel/Desktop/WayfindR-driver/archive/old_bakery_2025/scope.md \
   /home/devel/Desktop/WayfindR-driver/docs/old_bakery_scope.md

# Extract test_net.sh to utils
cp /home/devel/Desktop/WayfindR-driver/archive/old_bakery_2025/test_net.sh \
   /home/devel/Desktop/WayfindR-driver/utils/

# Update .gitignore or add .archived file
echo "This directory contains deprecated code. See new_bakery for current implementation." \
   > /home/devel/Desktop/WayfindR-driver/archive/old_bakery_2025/.archived
```

---

## Migration Path

If you need functionality from old_bakery:

### Old → New Equivalents

| Old Bakery Feature | New Bakery Equivalent |
|--------------------|-----------------------|
| GUI disk selection | `lsblk` + `--device` flag |
| Wipe methods | Not included (flash overwrites anyway) |
| Image selection | `--image` flag |
| WiFi config form | `--wifi-ssid` + `--wifi-pass` flags |
| User config form | `--username` + `--password` flags |
| Custom scripts GUI | Copy to `scripts/` directory with numbered names |
| SSH enable checkbox | Automatic via cloud-init |
| Hostname with timestamp | `--hostname` flag (auto-timestamp) |
| I2C/SPI checkboxes | Add to custom scripts if needed |
| Progress bars | Shell `dd status=progress` |
| Script ordering | Alphabetical by filename (01_, 02_, etc.) |
| State tracking | Cloud-init handles once-only execution |

### Example Migration

**Old way (bakery_old.py):**
1. Run Python GUI
2. Select disk from dropdown
3. Click mount
4. Add scripts via file dialog
5. Reorder with up/down buttons
6. Configure username in form
7. Click copy
8. Wait for progress bar
9. Click unmount

**New way (bake.sh):**
```bash
sudo ./new_bakery/bake.sh \
  --image ~/ubuntu-22.04.5-preinstalled-server-arm64+raspi.img.xz \
  --device /dev/sdb \
  --ssh-key ~/.ssh/id_rsa.pub \
  --hostname wayfinder-01 \
  --wifi-ssid "MyNetwork" \
  --wifi-pass "password123"
```

**Result:** Same outcome, 95% less code, easier to understand and maintain.

---

## Historical Context

### Development Timeline (inferred from files)

1. **Initial concept** - Fleet management vision (prompts4.md)
2. **First iteration** - Simple sideloader (bakery_old.py)
3. **Feature expansion** - Full flash + config GUI (old_bakery_flash)
4. **Complexity peak** - Workers, threading, state tracking
5. **Realization** - Cloud-init exists, simplify
6. **Refactor** - New bakery with cloud-init
7. **Deprecation** - Old code marked as legacy

### Key Insights from Conversation Logs

The prompt files reveal:
- Original goal: Fleet management for multiple Raspberry Pis
- Inspiration: "Miniature version of Ansible"
- Pain point: SD card flashing was tedious
- Evolution: Started simple, grew complex, then simplified
- Philosophy shift: Build vs. leverage existing tools

---

## Technical Debt Avoided by New Bakery

By moving to new_bakery, we avoid:

1. **PyQt5 version compatibility issues**
2. **Python 3.x migration headaches**
3. **Threading/GUI event loop bugs**
4. **State file synchronization problems**
5. **Complex mount point management**
6. **Duplicate configuration (JSON + shell + Python)**
7. **Testing GUI applications**
8. **Maintaining worker process communication**
9. **Cross-platform disk detection quirks**
10. **User confusion from too many options**

---

## Conclusion

The `old_bakery` represents an important evolutionary step in the WayfindR project. It demonstrates:

- **Good intentions** - Automation, ease of use, comprehensive features
- **Over-engineering** - Solution more complex than problem
- **Learning process** - Valuable lessons about simplicity
- **Successful pivot** - Recognition and course correction

**Verdict:** Archive with documentation. The code served its purpose as a learning tool and proof-of-concept. The new_bakery approach is superior for maintenance, testing, and long-term sustainability.

**Value preserved:**
- `test_net.sh` - Extracted to utils
- Documentation - This scope.md file
- Lessons learned - Applied to new_bakery design

**Total preserved value:** High (knowledge) + Low (code reuse) = Medium archival priority

---

## References

- New Bakery: `/home/devel/Desktop/WayfindR-driver/new_bakery/`
- Ubuntu Cloud-Init: https://cloudinit.readthedocs.io/
- ROS2 Humble: https://docs.ros.org/en/humble/
- Raspberry Pi Ubuntu: https://ubuntu.com/download/raspberry-pi

---

**End of Scope Documentation**
