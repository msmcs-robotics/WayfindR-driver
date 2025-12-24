#!/bin/bash
# Start all SLAM components in background terminals
# Use this for quick setup - runs everything at once

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  Starting WayfindR SLAM System"
echo "=========================================="
echo ""
echo "This will start:"
echo "  1. Static transforms (TF)"
echo "  2. RPLidar node"
echo "  3. SLAM Toolbox"
echo "  4. RViz2"
echo ""
echo "Opening in new terminal windows..."
echo "(If no GUI, use individual scripts in tmux)"
echo ""

# Check if we have a display
if [ -z "$DISPLAY" ]; then
    echo "WARNING: No display detected!"
    echo ""
    echo "Use tmux instead:"
    echo "  tmux new-session -d -s slam"
    echo "  tmux send-keys -t slam 'cd $SCRIPT_DIR && ./start_tf.sh' Enter"
    echo "  tmux split-window -h -t slam"
    echo "  tmux send-keys -t slam 'cd $SCRIPT_DIR && sleep 2 && ./start_lidar.sh' Enter"
    echo "  tmux split-window -v -t slam"
    echo "  tmux send-keys -t slam 'cd $SCRIPT_DIR && sleep 4 && ./start_slam.sh' Enter"
    echo "  tmux attach -t slam"
    exit 1
fi

# Start in separate terminals (gnome-terminal or xterm)
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal --title="TF" -- bash -c "cd $SCRIPT_DIR && ./start_tf.sh; exec bash"
    sleep 1
    gnome-terminal --title="LiDAR" -- bash -c "cd $SCRIPT_DIR && ./start_lidar.sh; exec bash"
    sleep 2
    gnome-terminal --title="SLAM" -- bash -c "cd $SCRIPT_DIR && ./start_slam.sh; exec bash"
    sleep 2
    gnome-terminal --title="RViz" -- bash -c "cd $SCRIPT_DIR && ./start_rviz.sh; exec bash"
elif command -v xterm &> /dev/null; then
    xterm -title "TF" -e "cd $SCRIPT_DIR && ./start_tf.sh; exec bash" &
    sleep 1
    xterm -title "LiDAR" -e "cd $SCRIPT_DIR && ./start_lidar.sh; exec bash" &
    sleep 2
    xterm -title "SLAM" -e "cd $SCRIPT_DIR && ./start_slam.sh; exec bash" &
    sleep 2
    xterm -title "RViz" -e "cd $SCRIPT_DIR && ./start_rviz.sh; exec bash" &
else
    echo "No terminal emulator found (gnome-terminal or xterm)"
    echo "Please run the scripts manually in separate terminals"
    exit 1
fi

echo ""
echo "All components starting!"
echo ""
echo "In RViz:"
echo "  - Set Fixed Frame to 'map'"
echo "  - Add /scan topic (LaserScan)"
echo "  - Add /map topic (Map)"
echo ""
echo "To save the map later:"
echo "  ./save_map.sh my_map_name"
echo ""
