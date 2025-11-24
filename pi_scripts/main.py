#!/usr/bin/env python3
"""
Raspberry Pi Pre-Baker - Main Entry Point
A comprehensive tool for flashing and configuring Raspberry Pi images
"""

import sys
import os

from PyQt5.QtWidgets import QApplication
from gui.main_window import RaspberryPiPreBaker


def check_root():
    """Check if running with root privileges"""
    if os.geteuid() != 0:
        print("=" * 60)
        print("ERROR: This application requires root privileges")
        print("=" * 60)
        print("\nReason: Direct disk access requires elevated permissions")
        print("\nPlease run with: sudo python3 main.py")
        print("=" * 60)
        sys.exit(1)


def main():
    """Main entry point"""
    check_root()
    
    app = QApplication(sys.argv)
    app.setApplicationName("Raspberry Pi Pre-Baker")
    app.setOrganizationName("RPi Tools")
    
    window = RaspberryPiPreBaker()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()