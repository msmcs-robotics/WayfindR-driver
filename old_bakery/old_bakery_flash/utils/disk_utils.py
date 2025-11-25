"""
Disk utilities for device detection, wiping, and partitioning
"""

import subprocess
import time
from pathlib import Path
from typing import List, Dict, Optional, Tuple


class DiskInfo:
    """Container for disk information"""
    def __init__(self, device: str, size: str, model: str, vendor: str, 
                 is_system: bool, mountpoint: str = ""):
        self.device = device
        self.size = size
        self.model = model
        self.vendor = vendor
        self.is_system = is_system
        self.mountpoint = mountpoint
    
    def __str__(self):
        system_flag = " [SYSTEM]" if self.is_system else ""
        return f"{self.device} ({self.size}) - {self.vendor} {self.model}{system_flag}"


class DiskUtils:
    """Utilities for disk operations"""
    
    @staticmethod
    def get_system_disks() -> set:
        """Get set of system disk devices"""
        system_disks = set()
        
        try:
            with open('/proc/mounts', 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        device = parts[0]
                        mountpoint = parts[1]
                        
                        # Check for system mount points
                        if mountpoint in ['/', '/boot', '/home', '/var', '/usr', '/opt']:
                            if device.startswith('/dev/'):
                                base_device = DiskUtils.get_base_device(device)
                                if base_device:
                                    system_disks.add(base_device)
        except Exception as e:
            print(f"Warning: Could not determine system disks: {e}")
        
        return system_disks
    
    @staticmethod
    def get_base_device(device_path: str) -> Optional[str]:
        """Get base disk device from partition"""
        try:
            result = subprocess.run(
                ['lsblk', '-ndo', 'PKNAME', device_path],
                capture_output=True,
                text=True,
                timeout=5
            )
            base_device = result.stdout.strip()
            if base_device:
                return f"/dev/{base_device}"
        except Exception:
            pass
        
        # Fallback: strip partition number
        import re
        match = re.match(r'(/dev/[a-z]+)', device_path)
        if match:
            return match.group(1)
        
        return device_path
    
    @staticmethod
    def get_available_disks() -> List[DiskInfo]:
        """Get list of available disk devices"""
        disks = []
        system_disks = DiskUtils.get_system_disks()
        
        try:
            result = subprocess.run(
                ['lsblk', '-d', '-n', '-o', 'NAME,SIZE,TYPE,MODEL,VENDOR'],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split(maxsplit=4)
                if len(parts) >= 3:
                    name = parts[0]
                    size = parts[1]
                    dtype = parts[2]
                    model = parts[3] if len(parts) > 3 else "Unknown"
                    vendor = parts[4] if len(parts) > 4 else "Unknown"
                    
                    if dtype == 'disk' and not name.startswith('loop'):
                        device_path = f"/dev/{name}"
                        is_system = device_path in system_disks
                        
                        disk_info = DiskInfo(
                            device=device_path,
                            size=size,
                            model=model,
                            vendor=vendor,
                            is_system=is_system
                        )
                        disks.append(disk_info)
        
        except Exception as e:
            print(f"Error getting available disks: {e}")
        
        return disks
    
    @staticmethod
    def get_device_size_mb(device_path: str) -> Optional[int]:
        """Get device size in MB"""
        try:
            result = subprocess.run(
                ['lsblk', '-b', '-n', '-o', 'SIZE', device_path],
                capture_output=True,
                text=True,
                timeout=5
            )
            size_bytes = int(result.stdout.strip())
            return size_bytes // (1024 * 1024)
        except Exception:
            return None
    
    @staticmethod
    def unmount_all_partitions(device_path: str) -> bool:
        """Unmount all partitions of a device"""
        try:
            # Get all partitions
            result = subprocess.run(
                ['lsblk', '-ln', '-o', 'NAME', device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if parts:
                    part_name = parts[0]
                    part_path = f"/dev/{part_name}"
                    
                    # Try multiple unmount methods
                    for _ in range(3):
                        subprocess.run(
                            ['sudo', 'umount', part_path],
                            stderr=subprocess.DEVNULL,
                            timeout=5
                        )
                        subprocess.run(
                            ['sudo', 'umount', '-l', part_path],
                            stderr=subprocess.DEVNULL,
                            timeout=5
                        )
            
            time.sleep(1)
            return True
            
        except Exception as e:
            print(f"Warning during unmount: {e}")
            return False
    
    @staticmethod
    def wipe_partition_table(device_path: str) -> bool:
        """Wipe partition table and filesystem signatures"""
        try:
            # Unmount first
            DiskUtils.unmount_all_partitions(device_path)
            
            # Wipe filesystem signatures
            subprocess.run(
                ['sudo', 'wipefs', '-a', device_path],
                check=True,
                timeout=30
            )
            
            # Zero out first 50MB (partition tables and boot sectors)
            subprocess.run(
                ['sudo', 'dd', 'if=/dev/zero', f'of={device_path}',
                 'bs=1M', 'count=50', 'status=none'],
                check=True,
                timeout=60
            )
            
            # Sync
            subprocess.run(['sync'], check=True, timeout=30)
            
            # Force kernel to reread partition table
            subprocess.run(
                ['sudo', 'partprobe', device_path],
                stderr=subprocess.DEVNULL,
                timeout=10
            )
            
            time.sleep(2)
            return True
            
        except Exception as e:
            print(f"Error wiping partition table: {e}")
            return False
    
    @staticmethod
    def detect_partitions(device_path: str, max_attempts: int = 10) -> Tuple[Optional[str], Optional[str]]:
        """
        Detect boot and root partitions after flashing
        Returns: (boot_partition, root_partition)
        """
        for attempt in range(max_attempts):
            # Force kernel to reread partition table
            subprocess.run(
                ['sudo', 'partprobe', device_path],
                stderr=subprocess.DEVNULL,
                timeout=10
            )
            
            time.sleep(2)
            
            # Try to detect partitions
            boot_part, root_part = DiskUtils._detect_partitions_by_number(device_path)
            
            if boot_part and root_part:
                # Verify partitions exist
                if Path(boot_part).exists() and Path(root_part).exists():
                    return boot_part, root_part
            
            # Try alternative detection
            boot_part, root_part = DiskUtils._detect_partitions_by_label(device_path)
            if boot_part and root_part:
                if Path(boot_part).exists() and Path(root_part).exists():
                    return boot_part, root_part
            
            if attempt < max_attempts - 1:
                time.sleep(3)
        
        return None, None
    
    @staticmethod
    def _detect_partitions_by_number(device_path: str) -> Tuple[Optional[str], Optional[str]]:
        """Detect partitions by partition number"""
        try:
            result = subprocess.run(
                ['lsblk', '-ln', '-o', 'NAME,TYPE', device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            boot_part = None
            root_part = None
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if len(parts) >= 2 and parts[1] == 'part':
                    part_name = parts[0]
                    part_path = f"/dev/{part_name}"
                    
                    # First partition is usually boot
                    if '1' in part_name or 'p1' in part_name:
                        boot_part = part_path
                    # Second partition is usually root
                    elif '2' in part_name or 'p2' in part_name:
                        root_part = part_path
            
            return boot_part, root_part
            
        except Exception:
            return None, None
    
    @staticmethod
    def _detect_partitions_by_label(device_path: str) -> Tuple[Optional[str], Optional[str]]:
        """Detect partitions by filesystem label"""
        try:
            result = subprocess.run(
                ['lsblk', '-ln', '-o', 'NAME,LABEL', device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            boot_part = None
            root_part = None
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split(maxsplit=1)
                if len(parts) >= 2:
                    part_name = parts[0]
                    label = parts[1].lower() if len(parts) > 1 else ""
                    part_path = f"/dev/{part_name}"
                    
                    if 'boot' in label or 'bootfs' in label:
                        boot_part = part_path
                    elif 'root' in label or 'rootfs' in label:
                        root_part = part_path
            
            return boot_part, root_part
            
        except Exception:
            return None, None
    
    @staticmethod
    def get_device_info(device_path: str) -> str:
        """Get detailed device information"""
        try:
            result = subprocess.run(
                ['lsblk', '-o', 'NAME,SIZE,TYPE,FSTYPE,LABEL,MOUNTPOINT', device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            return result.stdout.strip()
        except Exception as e:
            return f"Could not get device info: {e}"