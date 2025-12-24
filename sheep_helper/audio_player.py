"""
Audio Player Module
Plays audio alerts from case folders with random selection
Cross-platform support for Windows, macOS, and Linux
"""

import os
import random
import threading
import subprocess
import sys
from pathlib import Path


class AudioPlayer:
    # Supported audio formats
    SUPPORTED_FORMATS = {'.wav', '.mp3', '.mp4', '.ogg', '.flac'}

    def __init__(self, case1_folder="case1", case2_folder="case2", base_path=None):
        """
        Initialize audio player

        Args:
            case1_folder: Folder name for healthy (light) alerts
            case2_folder: Folder name for alert (dark) sounds
            base_path: Base path for folders (defaults to script directory)
        """
        if base_path is None:
            base_path = Path(__file__).parent

        self.case1_path = Path(base_path) / case1_folder
        self.case2_path = Path(base_path) / case2_folder

        # Track currently playing to avoid overlap
        self.is_playing = False
        self.play_lock = threading.Lock()

        # Cooldown to prevent audio spam
        self.last_play_time = 0
        self.cooldown_seconds = 0.5  # Minimum seconds between plays (reduced for responsiveness)

        # Cache available audio files
        self.case1_files = self._scan_audio_files(self.case1_path)
        self.case2_files = self._scan_audio_files(self.case2_path)

        print(f"Audio Player initialized:")
        print(f"  Case 1 (healthy) files: {len(self.case1_files)}")
        print(f"  Case 2 (alert) files: {len(self.case2_files)}")

    def _scan_audio_files(self, folder_path):
        """
        Scan folder for audio files

        Args:
            folder_path: Path to scan

        Returns:
            List of audio file paths
        """
        files = []
        if folder_path.exists():
            for f in folder_path.iterdir():
                if f.suffix.lower() in self.SUPPORTED_FORMATS:
                    files.append(f)
        return files

    def refresh_files(self):
        """
        Rescan folders for new audio files
        """
        self.case1_files = self._scan_audio_files(self.case1_path)
        self.case2_files = self._scan_audio_files(self.case2_path)

    def _play_audio_thread(self, filepath):
        """
        Play audio in a background thread

        Args:
            filepath: Path to audio file
        """
        try:
            filepath = str(filepath)

            if sys.platform == 'win32':
                # Windows - use built-in media player
                # Try Windows Media Player via PowerShell for MP3/WAV
                if filepath.lower().endswith('.wav'):
                    # Use winsound for WAV files (simpler)
                    try:
                        import winsound
                        winsound.PlaySound(filepath, winsound.SND_FILENAME)
                    except:
                        # Fallback to PowerShell
                        subprocess.run(
                            ['powershell', '-c', f'(New-Object Media.SoundPlayer "{filepath}").PlaySync()'],
                            capture_output=True
                        )
                else:
                    # For MP3 and other formats, use Windows Media Player
                    subprocess.run(
                        ['powershell', '-c', f'''
                        Add-Type -AssemblyName presentationCore
                        $player = New-Object System.Windows.Media.MediaPlayer
                        $player.Open("{filepath}")
                        $player.Play()
                        Start-Sleep -Seconds 10
                        '''],
                        capture_output=True,
                        timeout=15
                    )

            elif sys.platform == 'darwin':
                # macOS - use afplay
                subprocess.run(['afplay', filepath], capture_output=True)

            else:
                # Linux - try multiple players
                players = [
                    ['aplay', filepath],  # ALSA (WAV only)
                    ['paplay', filepath],  # PulseAudio
                    ['mpv', '--no-video', filepath],  # MPV
                    ['ffplay', '-nodisp', '-autoexit', filepath],  # FFmpeg
                ]

                for player_cmd in players:
                    try:
                        result = subprocess.run(
                            player_cmd,
                            capture_output=True,
                            timeout=30
                        )
                        if result.returncode == 0:
                            break
                    except (FileNotFoundError, subprocess.TimeoutExpired):
                        continue

        except Exception as e:
            print(f"Audio playback error: {e}")

        finally:
            with self.play_lock:
                self.is_playing = False

    def play_case(self, case_number, blocking=False):
        """
        Play a random audio file from the specified case folder

        Args:
            case_number: 1 for healthy, 2 for alert
            blocking: If True, wait for playback to complete

        Returns:
            True if playback started, False otherwise
        """
        import time

        # Check cooldown
        current_time = time.time()
        if current_time - self.last_play_time < self.cooldown_seconds:
            return False

        # Check if already playing
        with self.play_lock:
            if self.is_playing:
                return False
            self.is_playing = True

        # Select file list
        if case_number == 1:
            files = self.case1_files
        elif case_number == 2:
            files = self.case2_files
        else:
            with self.play_lock:
                self.is_playing = False
            return False

        if not files:
            print(f"No audio files in case{case_number} folder")
            with self.play_lock:
                self.is_playing = False
            return False

        # Random selection
        selected_file = random.choice(files)
        print(f"Playing: {selected_file.name}")

        self.last_play_time = current_time

        if blocking:
            self._play_audio_thread(selected_file)
        else:
            # Play in background thread
            thread = threading.Thread(
                target=self._play_audio_thread,
                args=(selected_file,),
                daemon=True
            )
            thread.start()

        return True

    def play_healthy(self, blocking=False):
        """Play a random healthy (case 1) sound"""
        return self.play_case(1, blocking)

    def play_alert(self, blocking=False):
        """Play a random alert (case 2) sound"""
        return self.play_case(2, blocking)

    def stop(self):
        """
        Stop current playback (best effort)
        """
        # This is tricky cross-platform; mainly useful for cleanup
        with self.play_lock:
            self.is_playing = False


# Test audio playback when run directly
if __name__ == "__main__":
    print("Testing Audio Player...")
    print()

    player = AudioPlayer()

    print("\nCase 1 files:")
    for f in player.case1_files:
        print(f"  - {f.name}")

    print("\nCase 2 files:")
    for f in player.case2_files:
        print(f"  - {f.name}")

    print()
    if player.case1_files:
        print("Playing random Case 1 sound...")
        player.play_healthy(blocking=True)

    if player.case2_files:
        print("Playing random Case 2 sound...")
        player.play_alert(blocking=True)

    print("Done!")
