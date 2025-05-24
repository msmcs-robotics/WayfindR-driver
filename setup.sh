#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo"
  exit 1
fi

echo "Running as root. Proceeding..."

# Define your functions
function option_one() {
    # setup SLAM Mapping & Waypoints
    echo "Setting up SLAM Mapping & Waypoints..."
}

function option_two() {
    # setup navigation script
    echo "Setting up Navigation & Movement..."
    wget https://raw.githubusercontent.com/msmcs-robotics/WayfindR-driver/refs/heads/main/3-navigation_driving.py -O /opt/3-navigation_driving.py
    chmod -R 777 /opt/
    chmod 777 /opt/3-navigation_driving.py
    chmod +x /opt/3-navigation_driving.py
    echo -e "
[Unit]
Description=Raspberry Pi Navigation Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/navigation/navigation.py
WorkingDirectory=/home/pi/navigation
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
    " > /etc/systemd/system/wayfinr-navigation.service

    systemctl daemon-reload
    systemctl enable wayfinr-navigation.service
    systemctl start navigation.service
}

function option_three() {
    echo "You selected Option 3 (All Options)"
    option_one
    option_two
}


# Display menu
echo "Select an option:"
echo "1) Setup SLAM Mapping & Waypoints"
echo "2) Setup Navigation & Movement"
echo "3) Setup Both"

read -p "Enter your choice [1-4]: " choice

# Execute based on input
case "$choice" in
    1) option_one ;;
    2) option_two ;;
    3) option_three ;;
    *) echo "Invalid option. Please enter 1, 2, or 3" ;;
esac


# Reboot if not connected to the internet
touch /etc/cron.d/ping_check
echo "* * * * * root ping -c 1 -W 5 google.com > /dev/null || /sbin/reboot" > /etc/cron.d/ping_check
chmod 644 /etc/cron.d/ping_check
chown root:root /etc/cron.d/ping_check

# Install system packages
apt-get update
apt-get install -y ufw python3 python3-pip python3-venv git libblas-dev liblapack-dev libatlas-base-dev gfortran

# Install WayfindR dependencies
wget https://raw.githubusercontent.com/msmcs-robotics/WayfindR-driver/refs/heads/main/requirements.txt -O /tmp/requirements.txt
pip3 install --upgrade pip
pip3 install -r /tmp/requirements.txt

# setup firewall rules

ufw allow 22/tcp
ufw allow 80/tcp
ufw allow 443/tcp
ufw allow 5000/tcp
ufw enable

# ufw deny 80/tcp
# ufw deny 443/tcp