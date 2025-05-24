#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo"
  exit 1
fi

echo "Running as root. Proceeding..."

sudo apt -fy install ros-melodic-cartographer-rviz