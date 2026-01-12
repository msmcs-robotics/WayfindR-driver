#!/bin/bash
# Map Server and AMCL Testing Scripts
# Generated: 2026-01-11
# Location: /home/devel/Desktop/WayfindR-driver/findings/

# ==============================================================================
# SCRIPT 1: Basic Map Server Test
# ==============================================================================
test_map_server_basic() {
    echo "=== Testing Map Server Basic Launch ==="
    source /opt/ros/humble/setup.bash
    
    ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml &
    
    MAP_PID=$!
    sleep 5
    
    echo "Map server running with PID: $MAP_PID"
    kill $MAP_PID 2>/dev/null
    wait $MAP_PID 2>/dev/null
}

# ==============================================================================
# SCRIPT 2: Map Server with Lifecycle Management
# ==============================================================================
test_map_server_lifecycle() {
    echo "=== Testing Map Server with Lifecycle ==="
    source /opt/ros/humble/setup.bash
    
    # Start map server
    ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml &
    MAP_PID=$!
    sleep 5
    
    # Configure
    echo "Configuring..."
    ros2 lifecycle set /map_server configure
    sleep 1
    
    # Activate
    echo "Activating..."
    ros2 lifecycle set /map_server activate
    sleep 1
    
    # Check state
    echo "Current state:"
    ros2 lifecycle get /map_server
    
    # Test map topic
    echo "Testing map publication:"
    timeout 3 ros2 topic echo /map --once | head -30
    
    # Cleanup
    kill $MAP_PID 2>/dev/null
    wait $MAP_PID 2>/dev/null
}

# ==============================================================================
# SCRIPT 3: AMCL Basic Test
# ==============================================================================
test_amcl_basic() {
    echo "=== Testing AMCL Basic Launch ==="
    source /opt/ros/humble/setup.bash
    
    ros2 run nav2_amcl amcl --ros-args \
        --params-file /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml &
    
    AMCL_PID=$!
    sleep 5
    
    echo "AMCL Parameters:"
    ros2 param list /amcl | head -20
    
    echo "AMCL Topics:"
    ros2 topic list | grep -E "amcl|particle"
    
    kill $AMCL_PID 2>/dev/null
    wait $AMCL_PID 2>/dev/null
}

# ==============================================================================
# SCRIPT 4: Full Localization Stack Test
# ==============================================================================
test_full_localization() {
    echo "=== Testing Full Localization Stack ==="
    source /opt/ros/humble/setup.bash
    
    # Start map server
    echo "Starting map server..."
    ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps/first_map.yaml &
    MAP_PID=$!
    sleep 3
    
    # Start AMCL
    echo "Starting AMCL..."
    ros2 run nav2_amcl amcl --ros-args \
        --params-file /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml &
    AMCL_PID=$!
    sleep 3
    
    # Configure and activate map server
    echo "Configuring map server..."
    ros2 lifecycle set /map_server configure
    sleep 1
    ros2 lifecycle set /map_server activate
    sleep 1
    
    # Configure and activate AMCL
    echo "Configuring AMCL..."
    ros2 lifecycle set /amcl configure
    sleep 1
    ros2 lifecycle set /amcl activate
    sleep 2
    
    # Check system state
    echo ""
    echo "Active Nodes:"
    ros2 node list | grep -E "map_server|amcl"
    
    echo ""
    echo "Map Server State:"
    ros2 lifecycle get /map_server
    
    echo ""
    echo "AMCL State:"
    ros2 lifecycle get /amcl
    
    echo ""
    echo "Localization Topics:"
    ros2 topic list | grep -E "map|amcl|particle|pose"
    
    echo ""
    echo "Topic Info - Particle Cloud:"
    ros2 topic info /particle_cloud
    
    echo ""
    echo "Topic Info - AMCL Pose:"
    ros2 topic info /amcl_pose
    
    # Cleanup
    echo ""
    echo "Cleaning up..."
    kill $AMCL_PID $MAP_PID 2>/dev/null
    wait $AMCL_PID $MAP_PID 2>/dev/null
    echo "Done"
}

# ==============================================================================
# SCRIPT 5: Quick Map Verification
# ==============================================================================
verify_map() {
    echo "=== Verifying Map Files ==="
    
    MAP_DIR="/home/devel/Desktop/WayfindR-driver/ros2_cartography_attempt/maps"
    
    echo "Map files in $MAP_DIR:"
    ls -lh "$MAP_DIR"
    
    echo ""
    echo "Map configuration (first_map.yaml):"
    cat "$MAP_DIR/first_map.yaml"
    
    echo ""
    echo "Map image info:"
    file "$MAP_DIR/first_map.pgm"
    
    echo ""
    echo "Waypoints (first 20 lines):"
    head -20 "$MAP_DIR/first_map_waypoints.yaml"
}

# ==============================================================================
# SCRIPT 6: Check AMCL Configuration
# ==============================================================================
check_amcl_config() {
    echo "=== Checking AMCL Configurations ==="
    
    echo "Configuration 1: ros2_localization_attempt"
    echo "File: /home/devel/Desktop/WayfindR-driver/ros2_localization_attempt/config/amcl_params.yaml"
    echo "Key parameters:"
    grep -E "set_initial_pose|max_particles|laser_max_range" \
        /home/devel/Desktop/WayfindR-driver/ros2_localization_attempt/config/amcl_params.yaml
    
    echo ""
    echo "Configuration 2: ros2_comprehensive_attempt"
    echo "File: /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml"
    echo "Key parameters:"
    grep -E "set_initial_pose|max_particles|laser_max_range" \
        /home/devel/Desktop/WayfindR-driver/ros2_comprehensive_attempt/config/amcl_params.yaml
}

# ==============================================================================
# Main Menu
# ==============================================================================
show_menu() {
    echo ""
    echo "=============================================="
    echo "Map Server and AMCL Testing Menu"
    echo "=============================================="
    echo "1. Test Map Server (Basic)"
    echo "2. Test Map Server (Lifecycle)"
    echo "3. Test AMCL (Basic)"
    echo "4. Test Full Localization Stack"
    echo "5. Verify Map Files"
    echo "6. Check AMCL Configurations"
    echo "7. Run All Tests"
    echo "0. Exit"
    echo "=============================================="
    echo -n "Select option: "
}

# Main execution
if [ "$1" == "" ]; then
    # Interactive mode
    while true; do
        show_menu
        read -r choice
        case $choice in
            1) test_map_server_basic ;;
            2) test_map_server_lifecycle ;;
            3) test_amcl_basic ;;
            4) test_full_localization ;;
            5) verify_map ;;
            6) check_amcl_config ;;
            7) 
                verify_map
                check_amcl_config
                test_map_server_lifecycle
                test_amcl_basic
                test_full_localization
                ;;
            0) echo "Exiting..."; exit 0 ;;
            *) echo "Invalid option" ;;
        esac
        echo ""
        echo "Press Enter to continue..."
        read -r
    done
else
    # Command line mode
    case $1 in
        map-basic) test_map_server_basic ;;
        map-lifecycle) test_map_server_lifecycle ;;
        amcl-basic) test_amcl_basic ;;
        full) test_full_localization ;;
        verify) verify_map ;;
        config) check_amcl_config ;;
        all)
            verify_map
            check_amcl_config
            test_map_server_lifecycle
            test_amcl_basic
            test_full_localization
            ;;
        *)
            echo "Usage: $0 [map-basic|map-lifecycle|amcl-basic|full|verify|config|all]"
            echo "Or run without arguments for interactive menu"
            exit 1
            ;;
    esac
fi
