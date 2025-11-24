I want to convert this bash program to be a python program with a GUI using pyqt5, or would it be better to use rust so i could make multiple build versions for different platforms? I want to have a gui that will be able to select a drive to flash, and be able to hotplug drives and have it auto-update. then i want to have all the features like enabling ssh, enabling services like VNC and more and adding user accounts just like the raspbian flasher, except i want to be able to side load custom scripts onto the pi so that on first boot after the system is setup, then it will run the scripts once does this make sense? i want everything to be optional and have a progress bar loading on the gui. I want to start with a python program then later i will move it to rust please make a comprehensive python program for pre-baking a raspberry pi - don't worry about the scripts that will be sideloaded for now, just make a feature to side load scripts to run on first boot. I also want to clarify that i want to be able to use the GUI to select an image to flash, like be prompted with my system's file explorer to select and image



I have the following python script, on different systems, disks are named differently, so i want to remove any features that prevent overwriting system disk and reconfigure them to do the following:

show all disks available
show disks mount point like /dev/sda
show disks name like manufacturer etc...
show which disks are the systems disks like based on where the system's root, home, and more are mounted to.
this script is only for linux systems so don't worry about windows commands
please give me the new refactored script


I just want to add a simple dropdown for usage and then include commands for attatching the usb to wsl

WSL can access USB storage devices (including microSD cards in USB readers), but not automatically — you must attach the device to WSL first.
Below is the clean, definitive guide to check if your microSD is visible and how to make it usable by your Python flashing program.
✅ 1. Check whether Windows sees the microSD
Before WSL matters, make sure Windows can see the device.
1. Insert the microSD into a USB reader (SD card slots built into laptops do not always expose themselves as USB devices — some appear as PCI devices, which WSL cannot access).
2. In Windows:
   * Press Win + X ➝ Disk Management
   * You should see something like Disk 2 – Removable – 64GB
Make sure it appears as a removable disk.
✅ 2. Check whether WSL can detect attachable devices
Open Windows PowerShell (not WSL) as Administrator and run:

wsl --list --all
Then list available USB devices for WSL:

usbipd list
If you see your microSD reader listed similar to:

BUSID  2-3   Generic MassStorageDevice  Attached: No
Then it's USB and attachable.
✅ 3. Attach the device to WSL
Still in PowerShell:

usbipd attach --wsl --busid 2-3
(Replace 2-3 with your device’s busid.)
You should see:

usbipd: Successfully attached.
✅ 4. Check inside WSL
Now open your WSL distro and run:

lsblk -p
You should now see something like:

/dev/sdb       59G  disk  
├─/dev/sdb1     256M part
└─/dev/sdb2     58G  part
If you see /dev/sdX, then YES, the Python GUI can manage it, flash to it, mount it, etc.
❌ If you don’t see the microSD in usbipd list
Your built-in SD slot might be PCI-based, not USB. WSL cannot access PCI devices. If so, you must use a USB microSD reader.
🧪 Verify write access
Before running the GUI app, manually verify you can write:

sudo dd if=/dev/zero of=/dev/sdb bs=1M count=1
⚠️ This will wipe the first MB of the SD card, so only do this on disposable media or after confirming it's the correct device.
If that succeeds, flashing will work.
🚀 Running your PyQt5 GUI in WSL
You need Windows X11 support:
Windows 11 (recommended)
You already have WSLg built in — GUI works immediately.
Just run:

python3 gui_prebake.py
Windows 10 (requires X Server)
Install:
* VcXsrv or Xming
* In WSL:

export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=1
Then run:

python3 gui_prebake.py
💡 Summary
Question Answer Does WSL automatically see microSD cards? ❌ No Can WSL access USB microSD readers? ✅ Yes, using usbipd Can WSL access built-in SD slots? ❌ Usually no (PCI, not USB) How do I attach a USB microSD reader? usbipd list → usbipd attach --wsl --busid X-Y How do I check inside WSL? lsblk -p
If you'd like, I can also:
➤ Adapt your GUI program to only use existing mounted Windows paths (/mnt/d/...)
— no sudo, no raw block writes.
➤ Add a “WSL Mode” toggle that disables flashing and only prebakes a mounted folder.
➤ Create an auto-detection helper script that tells the user whether their card is attachable.
Just tell me what direction you want!

this is the correcct one:
BUSID  VID:PID    DEVICE                                                        STATE
1-5    14cd:1212  USB Mass Storage Device                                       Not shared

Great — that is your microSD reader:

BUSID  VID:PID  DEVICE                    STATE
1-5    14cd:1212  USB Mass Storage Device  Not shared
This means:
✔ Windows does see it as a USB device
…but
❌ It is NOT attached to WSL yet
so your Python GUI won’t see /dev/sdX until you attach it.
Let’s finish the process.
✅ Step 1 — Attach the microSD reader into WSL
Run this in PowerShell (Admin):

usbipd bind --force --busid 1-5
usbipd attach --wsl --busid 1-5
If successful, you'll see:

usbipd: Successfully attached USB device on busid 1-5 to WSL
✅ Step 2 — Check inside WSL
Now open your WSL terminal and run:

lsblk -p
You should see something like:

/dev/sda       29G  disk
├─/dev/sda1     256M part
└─/dev/sda2     28G  part
If you see /dev/sda or /dev/sdb, your microSD card is now accessible to Python.
⚠️ If lsblk shows nothing
Then run this inside WSL:

dmesg | tail -50
You should see something like:

usb-storage: device found at 1-5
sd 0:0:0:0: [sda] Attached SCSI removable disk
If not, the issue is:
* bad SD card
* bad USB reader
* Windows storage driver blocking enumeration
But 14cd:1212 devices generally work reliably.
💡 Once it appears under /dev/sdX
Your PyQt prebake GUI will work exactly as intended, including:
* listing the device
* flashing images using dd
* mounting partitions
* copying bakery files
* enabling SSH
* generating bakery_config
* adding custom scripts
🔧 If you want, I can add a “WSL Mode” to your GUI:
Enables:
* Detect whether /dev/sdX is present
* Warn if no attached USB storage devices
* Show instructions for usbipd attach inside the GUI
* Optional: auto-run a PowerShell command through wsl.exe to fetch device list
Just tell me:
Do you want me to add WSL Mode to your prebake GUI?

I just want the simple dropdown for usage and then include commands to attatch to wsl

Show less


