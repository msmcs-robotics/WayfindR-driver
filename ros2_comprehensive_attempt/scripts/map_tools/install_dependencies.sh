#!/bin/bash

# Install dependencies for ROS2 Map Tools
# Installs required Python packages

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}ROS2 Map Tools - Dependency Installation${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# Check if running in ROS2 environment
if [ ! -z "$ROS_DISTRO" ]; then
    echo -e "${GREEN}ROS2 environment detected: $ROS_DISTRO${NC}"
    echo ""
    echo "Installing dependencies via apt..."
    echo ""

    # Install via apt (preferred for ROS2)
    sudo apt update
    sudo apt install -y \
        python3-pil \
        python3-numpy \
        python3-yaml

    echo ""
    echo -e "${GREEN}Dependencies installed successfully via apt${NC}"
else
    echo -e "${YELLOW}No ROS2 environment detected${NC}"
    echo ""
    echo "Installing dependencies via pip..."
    echo ""

    # Check if pip is available
    if ! command -v pip3 &> /dev/null; then
        echo "Error: pip3 not found. Please install python3-pip first:"
        echo "  sudo apt install python3-pip"
        exit 1
    fi

    # Install via pip
    pip3 install pillow numpy pyyaml

    echo ""
    echo -e "${GREEN}Dependencies installed successfully via pip${NC}"
fi

echo ""
echo -e "${BLUE}Verifying installation...${NC}"
echo ""

# Verify installations
python3 -c "import PIL; print('  PIL (Pillow): OK')"
python3 -c "import numpy; print('  NumPy: OK')"
python3 -c "import yaml; print('  PyYAML: OK')"

echo ""
echo -e "${GREEN}All dependencies verified!${NC}"
echo ""
echo "You can now use the map tools:"
echo "  cd $SCRIPT_DIR"
echo "  ./validate_map.py --help"
echo "  ./map_info.py --help"
echo "  ./map_converter.py --help"
echo "  ./compare_maps.py --help"
echo ""
echo "Or run the test suite:"
echo "  ./test_tools.sh"
echo ""
