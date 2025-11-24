"""
Worker thread for image flashing operations with compression support
"""

import os
import subprocess
import time
import gzip
import lzma
import zipfile
from pathlib import Path

from PyQt5.QtCore import QObject, pyqtSignal

from utils.disk_utils import DiskUtils


class FlashWorker(QObject):
    """Worker thread for flashing operations"""
    progress = pyqtSignal(int)
    status = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, image_path: str, device_path: str):
        super().__init__()
        self.image_path = image_path
        self.device_path = device_path
        self.cancelled = False
    
    def run(self):
        """Execute flash operation"""
        try:
            self.status.emit(f"Starting flash of {os.path.basename(self.image_path)}")
            self.status.emit(f"Target device: {self.device_path}")
            
            # Verify image exists
            if not os.path.exists(self.image_path):
                self.finished.emit(False, f"Image file not found: {self.image_path}")
                return
            
            # Unmount all partitions
            self.status.emit("Unmounting all partitions...")
            DiskUtils.unmount_all_partitions(self.device_path)
            self.progress.emit(5)
            
            if self.cancelled:
                self.finished.emit(False, "Flash cancelled by user")
                return
            
            # Detect compression and flash accordingly
            image_ext = Path(self.image_path).suffix.lower()
            
            if image_ext == '.xz':
                self.status.emit("Detected XZ compressed image")
                success = self._flash_compressed_xz()
            elif image_ext == '.gz':
                self.status.emit("Detected GZ compressed image")
                success = self._flash_compressed_gz()
            elif image_ext == '.zip':
                self.status.emit("Detected ZIP compressed image")
                success = self._flash_zip()
            else:
                self.status.emit("Detected uncompressed image")
                success = self._flash_uncompressed()
            
            if not success or self.cancelled:
                self.finished.emit(False, "Flash operation failed or was cancelled")
                return
            
            # Critical: Sync and force kernel to recognize new partitions
            self.status.emit("Syncing and waiting for device to be ready...")
            self.progress.emit(95)
            self._sync_and_wait()
            
            self.progress.emit(100)
            self.status.emit("Flash completed successfully")
            self.finished.emit(True, "Flash completed successfully")
            
        except Exception as e:
            import traceback
            error_msg = f"Flash failed: {str(e)}\n{traceback.format_exc()}"
            self.status.emit(error_msg)
            self.finished.emit(False, error_msg)
    
    def _flash_compressed_xz(self) -> bool:
        """Flash XZ compressed image using pipe"""
        try:
            self.status.emit("Decompressing and flashing XZ image...")
            
            # Use xzcat (or unxz) piped to dd
            # This is much more efficient than decompressing to temp file
            cmd = f"xzcat '{self.image_path}' | sudo dd of={self.device_path} bs=4M status=progress"
            
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            # Monitor progress
            last_progress = 10
            while True:
                if self.cancelled:
                    process.terminate()
                    process.wait(timeout=5)
                    return False
                
                # Check if process is still running
                if process.poll() is not None:
                    break
                
                # Read stderr for dd progress output
                line = process.stderr.readline()
                if line:
                    self.status.emit(line.strip())
                    # Try to parse progress from dd output
                    if 'bytes' in line or 'MB' in line or 'GB' in line:
                        # Increment progress gradually
                        if last_progress < 90:
                            last_progress += 2
                            self.progress.emit(last_progress)
                
                time.sleep(0.5)
            
            return_code = process.wait()
            
            if return_code != 0:
                stderr = process.stderr.read()
                self.status.emit(f"Flash error: {stderr}")
                return False
            
            self.status.emit("XZ decompression and flash completed")
            return True
            
        except Exception as e:
            self.status.emit(f"XZ flash error: {e}")
            return False
    
    def _flash_compressed_gz(self) -> bool:
        """Flash GZ compressed image using pipe"""
        try:
            self.status.emit("Decompressing and flashing GZ image...")
            
            cmd = f"zcat '{self.image_path}' | sudo dd of={self.device_path} bs=4M status=progress"
            
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            last_progress = 10
            while True:
                if self.cancelled:
                    process.terminate()
                    process.wait(timeout=5)
                    return False
                
                if process.poll() is not None:
                    break
                
                line = process.stderr.readline()
                if line:
                    self.status.emit(line.strip())
                    if 'bytes' in line or 'MB' in line:
                        if last_progress < 90:
                            last_progress += 2
                            self.progress.emit(last_progress)
                
                time.sleep(0.5)
            
            return_code = process.wait()
            return return_code == 0
            
        except Exception as e:
            self.status.emit(f"GZ flash error: {e}")
            return False
    
    def _flash_zip(self) -> bool:
        """Flash ZIP compressed image"""
        try:
            self.status.emit("Extracting ZIP archive...")
            
            # ZIP files need to be extracted first
            # Find the .img file inside
            with zipfile.ZipFile(self.image_path, 'r') as zf:
                img_files = [f for f in zf.namelist() if f.endswith('.img')]
                
                if not img_files:
                    self.status.emit("No .img file found in ZIP archive")
                    return False
                
                img_file = img_files[0]
                self.status.emit(f"Found image: {img_file}")
                
                # Extract to temp location
                temp_img = f"/tmp/rpi_flash_{os.getpid()}.img"
                
                with zf.open(img_file) as source, open(temp_img, 'wb') as target:
                    total_size = zf.getinfo(img_file).file_size
                    extracted = 0
                    chunk_size = 4 * 1024 * 1024
                    
                    while True:
                        chunk = source.read(chunk_size)
                        if not chunk:
                            break
                        target.write(chunk)
                        extracted += len(chunk)
                        
                        progress = 10 + int((extracted / total_size) * 40)
                        self.progress.emit(progress)
                
                # Now flash the extracted image
                self.status.emit("Flashing extracted image...")
                success = self._flash_file(temp_img, start_progress=50)
                
                # Cleanup
                try:
                    os.remove(temp_img)
                except:
                    pass
                
                return success
                
        except Exception as e:
            self.status.emit(f"ZIP flash error: {e}")
            return False
    
    def _flash_uncompressed(self) -> bool:
        """Flash uncompressed image"""
        try:
            image_size = os.path.getsize(self.image_path)
            return self._flash_file(self.image_path, image_size, start_progress=10)
        except Exception as e:
            self.status.emit(f"Flash error: {e}")
            return False
    
    def _flash_file(self, file_path: str, file_size: int = None, start_progress: int = 10) -> bool:
        """Flash a file to device with progress tracking"""
        try:
            if file_size is None:
                file_size = os.path.getsize(file_path)
            
            image_size_mb = file_size // (1024 * 1024)
            self.status.emit(f"Image size: {image_size_mb} MB")
            
            block_size = 4 * 1024 * 1024  # 4MB blocks
            
            with open(file_path, 'rb') as src:
                dd_process = subprocess.Popen(
                    ['sudo', 'dd', f'of={self.device_path}', 'bs=4M', 
                     'conv=fsync', 'oflag=direct,sync', 'status=none'],
                    stdin=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                
                bytes_written = 0
                last_log_mb = 0
                
                while True:
                    if self.cancelled:
                        dd_process.terminate()
                        dd_process.wait(timeout=5)
                        return False
                    
                    chunk = src.read(block_size)
                    if not chunk:
                        break
                    
                    dd_process.stdin.write(chunk)
                    bytes_written += len(chunk)
                    
                    # Update progress
                    progress = start_progress + int((bytes_written / file_size) * (90 - start_progress))
                    self.progress.emit(progress)
                    
                    mb_written = bytes_written // (1024 * 1024)
                    
                    # Log every 100MB to reduce noise
                    if mb_written >= last_log_mb + 100 or mb_written == image_size_mb:
                        self.status.emit(f"Written {mb_written} MB / {image_size_mb} MB")
                        last_log_mb = mb_written
                
                # Close stdin and wait for dd to complete
                dd_process.stdin.close()
                return_code = dd_process.wait(timeout=120)
                
                if return_code != 0:
                    stderr = dd_process.stderr.read().decode()
                    self.status.emit(f"DD error: {stderr}")
                    return False
                
                return True
                
        except Exception as e:
            self.status.emit(f"Flash error: {e}")
            return False
    
    def _sync_and_wait(self):
        """
        CRITICAL: Properly sync and force kernel to recognize new partition table
        This is the most important part for making the device bootable
        """
        try:
            self.status.emit("Flushing buffers (this may take a while)...")
            # Multiple sync calls to ensure everything is written
            for i in range(3):
                subprocess.run(['sync'], check=True, timeout=60)
                time.sleep(2)
            
            # Ensure the device is fully flushed
            subprocess.run(
                ['sudo', 'blockdev', '--flushbufs', self.device_path],
                check=True,
                timeout=30
            )
            
            self.status.emit("Forcing kernel to re-read partition table...")
            
            # Multiple methods to force partition table reload
            methods = [
                ['sudo', 'blockdev', '--rereadpt', self.device_path],
                ['sudo', 'partprobe', self.device_path],
                ['sudo', 'partx', '-u', self.device_path],
            ]
            
            for method in methods:
                try:
                    subprocess.run(method, timeout=10, check=False, 
                                 stderr=subprocess.DEVNULL)
                    time.sleep(2)
                except:
                    pass
            
            # Let udev settle
            self.status.emit("Waiting for udev to settle...")
            subprocess.run(['sudo', 'udevadm', 'settle'], timeout=30, check=False)
            time.sleep(3)
            
            # Verify partitions are visible
            self.status.emit("Verifying partitions...")
            for attempt in range(10):
                result = subprocess.run(
                    ['lsblk', '-ln', self.device_path],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                
                lines = [l for l in result.stdout.strip().split('\n') if l.strip()]
                partition_count = len(lines) - 1  # Subtract main device
                
                if partition_count >= 2:
                    self.status.emit(f"✓ Detected {partition_count} partitions")
                    # Show partition details
                    self.status.emit("Partition layout:")
                    for line in lines[1:]:  # Skip main device
                        self.status.emit(f"  {line}")
                    return
                
                if attempt < 9:
                    self.status.emit(f"Waiting for partitions... (attempt {attempt + 1}/10)")
                    subprocess.run(
                        ['sudo', 'partprobe', self.device_path],
                        stderr=subprocess.DEVNULL,
                        timeout=10
                    )
                    time.sleep(3)
            
            self.status.emit("⚠ Warning: Partitions may not be fully recognized yet")
            self.status.emit("  You may need to unplug and replug the device")
            
        except Exception as e:
            self.status.emit(f"Warning during sync: {e}")
    
    def cancel(self):
        """Cancel the flash operation"""
        self.cancelled = True