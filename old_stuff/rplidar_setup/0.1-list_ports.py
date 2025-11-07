import os
import sys
import platform
from serial.tools import list_ports

def list_serial_ports():
    system = platform.system()

    if system == 'Windows':
        print("Available COM ports:")
        ports = list(list_ports.comports())
        if not ports:
            print("  No COM ports found.")
        for port in ports:
            print(f"  {port.device} - {port.description}")

    elif system in ['Linux', 'Darwin']:  # Darwin = macOS
        print("Available USB serial ports:")
        base_paths = ['/dev/ttyUSB', '/dev/ttyACM']
        found = False
        for base in base_paths:
            for i in range(10):  # Check up to ttyUSB9/ttyACM9
                path = f"{base}{i}"
                if os.path.exists(path):
                    print(f"  {path}")
                    found = True
        if not found:
            print("  No USB serial ports found.")

    else:
        print(f"Unsupported OS: {system}")
        sys.exit(1)

if __name__ == "__main__":
    list_serial_ports()
