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
    sudo apt install -fy ros-melodic-cartographer-ros ros-melodic-rplidar-ros
    git clone https://github.com/Andrew-rw/gbot_core.git


    echo -e "Run the following on your laptop/desktop:\n\nsudo apt install ros-melodic-cartographer-rviz\n\nPress 'c' to continue."

    while true; do
        read -n1 -s key  # read one character silently
        if [[ $key == "c" ]]; then
            break
        fi
    done

    echo "Continuing..."

}

function option_two() {
    # setup navigation script
    echo "Setting up Navigation & Movement..."
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
