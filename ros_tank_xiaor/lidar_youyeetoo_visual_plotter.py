#!/usr/bin/env python3
"""
YouYeeToo LiDAR Real-time Visualizer
Shows LiDAR scan data in real-time using matplotlib
"""

import serial
import struct
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

class LiDARVisualizer:
    HEADER = 0x54
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_connected = False
        
        # Store recent measurements
        self.measurements = deque(maxlen=2000)
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        self.scatter = None
        
    def connect(self):
        """Connect to LiDAR"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )
            self.is_connected = True
            print(f"✓ Connected to LiDAR on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def parse_packet(self, data):
        """Parse LiDAR packet"""
        if len(data) != 47 or data[0] != self.HEADER:
            return None
        
        try:
            start_angle_raw = struct.unpack('<H', data[4:6])[0]
            end_angle_raw = struct.unpack('<H', data[42:44])[0]
            
            start_angle = start_angle_raw / 100.0
            end_angle = end_angle_raw / 100.0
            
            angle_diff = end_angle - start_angle
            if angle_diff < 0:
                angle_diff += 360.0
            angle_step = angle_diff / 12.0
            
            measurements = []
            for i in range(12):
                offset = 6 + (i * 3)
                distance = struct.unpack('<H', data[offset:offset+2])[0]
                intensity = data[offset+2]
                
                angle = start_angle + (i * angle_step)
                if angle >= 360.0:
                    angle -= 360.0
                
                # Convert to radians for polar plot
                angle_rad = math.radians(angle)
                
                # Filter out zero/invalid measurements
                if distance > 0 and distance < 12000:  # Max 12m
                    measurements.append({
                        'angle': angle_rad,
                        'distance': distance / 1000.0,  # Convert to meters
                        'intensity': intensity
                    })
            
            return measurements
            
        except Exception as e:
            return None
    
    def read_packets(self):
        """Read and parse incoming packets"""
        if not self.is_connected:
            return
        
        try:
            # Read available data
            while self.serial.in_waiting > 0:
                byte = self.serial.read(1)
                if len(byte) > 0 and byte[0] == self.HEADER:
                    packet = byte + self.serial.read(46)
                    if len(packet) == 47:
                        measurements = self.parse_packet(packet)
                        if measurements:
                            self.measurements.extend(measurements)
        except Exception as e:
            print(f"Read error: {e}")
    
    def update_plot(self, frame):
        """Update the plot with new data"""
        # Read new data
        self.read_packets()
        
        if len(self.measurements) == 0:
            return self.scatter,
        
        # Extract angles, distances, and intensities
        angles = [m['angle'] for m in self.measurements]
        distances = [m['distance'] for m in self.measurements]
        intensities = [m['intensity'] for m in self.measurements]
        
        # Clear and redraw
        self.ax.clear()
        
        # Plot points with color based on intensity
        self.scatter = self.ax.scatter(
            angles,
            distances,
            c=intensities,
            cmap='hot',
            s=10,
            alpha=0.6,
            vmin=0,
            vmax=255
        )
        
        # Configure plot
        self.ax.set_ylim(0, 12)  # 12 meter max range
        self.ax.set_theta_zero_location('N')  # 0° at top
        self.ax.set_theta_direction(-1)  # Clockwise
        self.ax.set_title(f'LiDAR Scan - {len(self.measurements)} points', 
                         pad=20, fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        
        # Add distance rings labels
        self.ax.set_rticks([2, 4, 6, 8, 10, 12])
        self.ax.set_yticklabels(['2m', '4m', '6m', '8m', '10m', '12m'])
        
        return self.scatter,
    
    def run(self):
        """Start the visualization"""
        print("=" * 50)
        print("LiDAR Real-time Visualizer")
        print("=" * 50)
        print("Press Ctrl+C to stop")
        print("Close the plot window to exit")
        print("=" * 50)
        
        if not self.connect():
            return
        
        try:
            # Create animation
            ani = FuncAnimation(
                self.fig,
                self.update_plot,
                interval=50,  # Update every 50ms
                blit=False,
                cache_frame_data=False
            )
            
            plt.tight_layout()
            plt.show()
            
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()
            print("Disconnected")

def main():
    import sys
    
    port = '/dev/ttyUSB0'
    baudrate = 230400
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])
    
    print(f"Starting visualizer on {port} @ {baudrate} baud")
    
    viz = LiDARVisualizer(port=port, baudrate=baudrate)
    viz.run()

if __name__ == "__main__":
    main()