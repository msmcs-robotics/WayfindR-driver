"""
Worker thread for disk wiping operations with filesystem detection
"""

import subprocess
import time
from typing import Optional

from PyQt5.QtCore import QObject, pyqtSignal

from utils.disk_utils import DiskUtils


class WipeWorker(QObject):
    """Worker thread for disk wiping operations"""
    progress = pyqtSignal(int)
    status = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, device_path: str, wipe_method: str):
        super().__init__()
        self.device_path = device_path
        self.wipe_method = wipe_method
        self.cancelled = False
    
    def run(self):
        """Execute wipe operation"""
        try:
            self.status.emit(f"Starting disk wipe using {self.wipe_method} method...")
            self.progress.emit(5)
            
            # Detect current filesystem state
            self._detect_filesystem_state()
            
            # Unmount all partitions first
            self.status.emit("Unmounting all partitions...")
            DiskUtils.unmount_all_partitions(self.device_path)
            
            # Force unmount with lazy option
            self._force_unmount_all()
            
            self.progress.emit(10)
            
            if self.cancelled:
                self.finished.emit(False, "Wipe cancelled by user")
                return
            
            # Always wipe signatures first, regardless of method
            self._wipe_signatures()
            self.progress.emit(15)
            
            # Perform wipe based on method
            if self.wipe_method == "quick":
                success = self._quick_wipe()
            elif self.wipe_method == "secure":
                success = self._secure_wipe()
            elif self.wipe_method == "zero":
                success = self._zero_wipe()
            else:
                success = self._quick_wipe()
            
            if not success:
                self.finished.emit(False, "Wipe operation failed")
                return
            
            if self.cancelled:
                self.finished.emit(False, "Wipe cancelled by user")
                return
            
            # Final cleanup and verification
            self._final_cleanup()
            
            self.progress.emit(100)
            self.status.emit("Disk wipe completed successfully")
            self.finished.emit(True, "Disk wipe completed successfully")
            
        except Exception as e:
            import traceback
            error_msg = f"Disk wipe failed: {str(e)}\n{traceback.format_exc()}"
            self.status.emit(error_msg)
            self.finished.emit(False, error_msg)
    
    def _detect_filesystem_state(self):
        """Detect and report current filesystem state"""
        try:
            self.status.emit("Analyzing device state...")
            
            # Check for partitions
            result = subprocess.run(
                ['lsblk', '-o', 'NAME,FSTYPE,LABEL,SIZE,MOUNTPOINT', self.device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            self.status.emit("Current device state:")
            for line in result.stdout.strip().split('\n'):
                self.status.emit(f"  {line}")
            
            # Check for filesystem signatures
            result = subprocess.run(
                ['sudo', 'blkid', self.device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.stdout.strip():
                self.status.emit(f"Detected filesystem signatures:")
                self.status.emit(f"  {result.stdout.strip()}")
            
        except Exception as e:
            self.status.emit(f"Note: Could not fully analyze device state: {e}")
    
    def _force_unmount_all(self):
        """Aggressively unmount all partitions"""
        try:
            # Get all partitions
            result = subprocess.run(
                ['lsblk', '-ln', '-o', 'NAME', self.device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if parts:
                    part_name = parts[0]
                    part_path = f"/dev/{part_name}"
                    
                    # Try multiple unmount strategies
                    for attempt in range(3):
                        # Normal unmount
                        subprocess.run(
                            ['sudo', 'umount', part_path],
                            stderr=subprocess.DEVNULL,
                            timeout=5
                        )
                        
                        # Lazy unmount
                        subprocess.run(
                            ['sudo', 'umount', '-l', part_path],
                            stderr=subprocess.DEVNULL,
                            timeout=5
                        )
                        
                        # Force unmount
                        subprocess.run(
                            ['sudo', 'umount', '-f', part_path],
                            stderr=subprocess.DEVNULL,
                            timeout=5
                        )
                        
                        time.sleep(0.5)
            
            # Final sync
            subprocess.run(['sync'], timeout=10)
            time.sleep(2)
            
        except Exception as e:
            self.status.emit(f"Note during unmounting: {e}")
    
    def _wipe_signatures(self):
        """Wipe all filesystem and partition signatures"""
        try:
            self.status.emit("Wiping filesystem signatures...")
            
            # Wipe signatures from main device
            subprocess.run(
                ['sudo', 'wipefs', '--all', '--force', self.device_path],
                check=True,
                timeout=30,
                stderr=subprocess.PIPE
            )
            
            # Try to wipe from any partition devices too
            result = subprocess.run(
                ['lsblk', '-ln', '-o', 'NAME', self.device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            for line in result.stdout.strip().split('\n'):
                parts = line.split()
                if parts:
                    part_name = parts[0]
                    part_path = f"/dev/{part_name}"
                    if part_path != self.device_path:
                        subprocess.run(
                            ['sudo', 'wipefs', '--all', '--force', part_path],
                            stderr=subprocess.DEVNULL,
                            timeout=10
                        )
            
            subprocess.run(['sync'], timeout=10)
            self.status.emit("✓ Filesystem signatures wiped")
            
        except Exception as e:
            self.status.emit(f"Note during signature wipe: {e}")
    
    def _quick_wipe(self) -> bool:
        """Quick wipe - erase partition table and first 50MB"""
        try:
            self.status.emit("Performing quick wipe...")
            self.progress.emit(20)
            
            # Zero out first 50MB (covers MBR, GPT, and boot sectors)
            self.status.emit("Wiping 50 MB...")
            result = subprocess.run(
                ['sudo', 'dd', 'if=/dev/zero', f'of={self.device_path}',
                 'bs=1M', 'count=50', 'conv=fsync', 'oflag=direct', 'status=none'],
                capture_output=True,
                timeout=120
            )
            
            if result.returncode != 0:
                self.status.emit(f"Warning: {result.stderr.decode()}")
            
            self.progress.emit(80)
            
            if self.cancelled:
                return False
            
            self.status.emit("Quick wipe completed")
            return True
            
        except Exception as e:
            self.status.emit(f"Quick wipe error: {e}")
            return False
    
    def _secure_wipe(self) -> bool:
        """Secure wipe - one pass with random data"""
        try:
            self.status.emit("Performing secure wipe (1-pass random data)...")
            
            # Get device size
            device_size = DiskUtils.get_device_size_mb(self.device_path)
            if not device_size:
                raise Exception("Could not determine device size")
            
            self.progress.emit(15)
            
            if self.cancelled:
                return False
            
            # One pass with random data
            self.status.emit(f"Writing random data to {device_size} MB...")
            self.status.emit("This will take a while depending on device size...")
            
            success = self._run_dd_with_progress(
                ['sudo', 'dd', 'if=/dev/urandom', f'of={self.device_path}',
                 'bs=1M', 'conv=fsync', 'oflag=direct', 'status=none'],
                device_size,
                15, 90
            )
            
            if not success or self.cancelled:
                return False
            
            self.status.emit("Secure wipe completed")
            return True
            
        except Exception as e:
            self.status.emit(f"Secure wipe error: {e}")
            return False
    
    def _zero_wipe(self) -> bool:
        """Complete wipe with zeros"""
        try:
            self.status.emit("Performing complete zero wipe...")
            
            # Get device size
            device_size = DiskUtils.get_device_size_mb(self.device_path)
            if not device_size:
                raise Exception("Could not determine device size")
            
            self.progress.emit(15)
            
            if self.cancelled:
                return False
            
            # Full wipe with zeros
            self.status.emit(f"Writing zeros to {device_size} MB...")
            self.status.emit("This will take a while depending on device size...")
            
            success = self._run_dd_with_progress(
                ['sudo', 'dd', 'if=/dev/zero', f'of={self.device_path}',
                 'bs=1M', 'conv=fsync', 'oflag=direct', 'status=none'],
                device_size,
                15, 90
            )
            
            if not success or self.cancelled:
                return False
            
            self.status.emit("Zero wipe completed")
            return True
            
        except Exception as e:
            self.status.emit(f"Zero wipe error: {e}")
            return False
    
    def _run_dd_with_progress(self, cmd: list, total_mb: int, 
                             start_progress: int, end_progress: int) -> bool:
        """Run dd command with simulated progress"""
        try:
            process = subprocess.Popen(
                cmd,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            # Simulate progress based on time
            start_time = time.time()
            # Estimate: 30 MB/s for random, 50 MB/s for zeros
            estimated_speed = 30 if 'urandom' in ' '.join(cmd) else 50
            estimated_time = total_mb / estimated_speed
            
            progress_range = end_progress - start_progress
            last_log_time = start_time
            
            while process.poll() is None:
                if self.cancelled:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except:
                        process.kill()
                    return False
                
                elapsed = time.time() - start_time
                if estimated_time > 0:
                    progress_pct = min(elapsed / estimated_time, 0.99)
                    current_progress = start_progress + int(progress_pct * progress_range)
                    self.progress.emit(current_progress)
                
                # Log progress every 10 seconds
                if time.time() - last_log_time >= 10:
                    mb_written = int((elapsed / estimated_time) * total_mb)
                    self.status.emit(f"  Progress: ~{mb_written} / {total_mb} MB")
                    last_log_time = time.time()
                
                time.sleep(0.5)
            
            return_code = process.wait()
            
            if return_code != 0:
                stderr = process.stderr.read()
                self.status.emit(f"DD error: {stderr}")
                return False
            
            return True
            
        except Exception as e:
            self.status.emit(f"DD operation error: {e}")
            return False
    
    def _final_cleanup(self):
        """Final cleanup and verification after wipe"""
        try:
            self.status.emit("Performing final cleanup...")
            
            # Multiple syncs
            for i in range(3):
                subprocess.run(['sync'], timeout=30)
                time.sleep(1)
            
            # Flush device buffers
            subprocess.run(
                ['sudo', 'blockdev', '--flushbufs', self.device_path],
                timeout=30,
                stderr=subprocess.DEVNULL
            )
            
            # Force partition table re-read
            subprocess.run(
                ['sudo', 'blockdev', '--rereadpt', self.device_path],
                timeout=10,
                stderr=subprocess.DEVNULL
            )
            
            subprocess.run(
                ['sudo', 'partprobe', self.device_path],
                timeout=10,
                stderr=subprocess.DEVNULL
            )
            
            # Let system settle
            subprocess.run(['sudo', 'udevadm', 'settle'], timeout=30, stderr=subprocess.DEVNULL)
            time.sleep(2)
            
            # Verify clean state
            result = subprocess.run(
                ['sudo', 'blkid', self.device_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.stdout.strip():
                self.status.emit("⚠ Warning: Some signatures may still be detected")
                self.status.emit(f"  {result.stdout.strip()}")
            else:
                self.status.emit("✓ Device successfully wiped - no signatures detected")
            
        except Exception as e:
            self.status.emit(f"Note during final cleanup: {e}")
    
    def cancel(self):
        """Cancel the wipe operation"""
        self.cancelled = True