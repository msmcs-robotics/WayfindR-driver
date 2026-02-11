#!/bin/bash
# =============================================================================
# Ambot - Master Deployment Script
# =============================================================================
# Deploys ambot components to target devices (Raspberry Pi, Jetson).
# This script is IDEMPOTENT - safe to run multiple times.
#
# Usage:
#   ./deploy.sh                        # Deploy all to default targets
#   ./deploy.sh rpi                    # Deploy all to Raspberry Pi
#   ./deploy.sh rpi pathfinder         # Deploy only pathfinder to RPi
#   ./deploy.sh rpi --test=all         # Deploy + run ALL tests
#   ./deploy.sh rpi --test=quick       # Deploy + quick verification
#   ./deploy.sh rpi --test=hardware    # Deploy + test connected hardware
#   ./deploy.sh rpi --test=lidar       # Deploy + run specific LiDAR test
#   ./deploy.sh rpi --test=check       # Deploy + check installed packages
#   ./deploy.sh rpi --bootstrap        # Deploy + run bootstrap scripts
#   ./deploy.sh --status               # Check connection status only
#
# Components:
#   all           - All components (default)
#   bootylicious  - LLM + RAG system
#   locomotion    - Motor control
#   pathfinder    - LiDAR + obstacle avoidance
#   tests         - Test scripts
#   scripts       - Bootstrap scripts
#   docs          - Documentation
#
# Targets (from connections.md):
#   Raspberry Pi: pi@10.33.224.1
#   Jetson:       georgejetson@10.33.255.82
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[====]${NC} $1"; }

# =============================================================================
# Target Configuration
# =============================================================================

# Raspberry Pi
RPI_USER="pi"
RPI_HOST="10.33.224.1"
RPI_TARGET="$RPI_USER@$RPI_HOST"
RPI_DEST="~/ambot"

# Jetson Orin Nano
JETSON_USER="georgejetson"
JETSON_HOST="10.33.255.82"
JETSON_TARGET="$JETSON_USER@$JETSON_HOST"
JETSON_DEST="~/ambot"

# SSH timeout (seconds) - increased for slow networks
SSH_TIMEOUT=15

# Rsync base options
RSYNC_BASE="-avz --progress"
RSYNC_EXCLUDE="--exclude='*.pyc' --exclude='__pycache__' --exclude='.git' --exclude='*.egg-info' --exclude='.pytest_cache' --exclude='venv' --exclude='.venv'"

# =============================================================================
# Helper Functions
# =============================================================================

check_ssh() {
    local target="$1"
    # Use SSH to check connectivity (not ping - some networks block ICMP)
    ssh -o ConnectTimeout=$SSH_TIMEOUT -o BatchMode=yes -o StrictHostKeyChecking=no "$target" "echo ok" &>/dev/null
    return $?
}

print_banner() {
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                   AMBOT DEPLOYMENT                         ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# =============================================================================
# Status Check
# =============================================================================

check_status() {
    log_step "Checking connection status (via SSH, timeout=${SSH_TIMEOUT}s)..."
    echo ""

    # Raspberry Pi
    echo -n "  Raspberry Pi ($RPI_TARGET): "
    if check_ssh "$RPI_TARGET"; then
        echo -e "${GREEN}ONLINE${NC}"
        # Show system info
        ssh -o ConnectTimeout=$SSH_TIMEOUT "$RPI_TARGET" "uname -n && cat /proc/device-tree/model 2>/dev/null | tr -d '\0' || echo ''" 2>/dev/null | head -2 | sed 's/^/    /'
    else
        echo -e "${RED}OFFLINE${NC}"
    fi
    echo ""

    # Jetson
    if [ -n "$JETSON_HOST" ]; then
        echo -n "  Jetson ($JETSON_TARGET): "
        if check_ssh "$JETSON_TARGET"; then
            echo -e "${GREEN}ONLINE${NC}"
            ssh -o ConnectTimeout=$SSH_TIMEOUT "$JETSON_TARGET" "uname -n" 2>/dev/null | sed 's/^/    /'
        else
            echo -e "${RED}OFFLINE${NC}"
        fi
    else
        echo -e "  Jetson: ${YELLOW}NOT CONFIGURED${NC}"
        echo "    Set JETSON_HOST in this script when available"
    fi

    echo ""
}

# =============================================================================
# Sync Component
# =============================================================================

sync_component() {
    local target="$1"
    local dest="$2"
    local component="$3"

    case "$component" in
        all)
            log_info "Syncing all components..."
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                --exclude='tests/results/*.jpg' \
                --exclude='tests/results/*.png' \
                "$SCRIPT_DIR/" "$target:$dest/"
            ;;
        bootylicious)
            log_info "Syncing bootylicious (LLM + RAG)..."
            ssh "$target" "mkdir -p $dest/bootylicious"
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                "$SCRIPT_DIR/bootylicious/" "$target:$dest/bootylicious/"
            ;;
        locomotion)
            log_info "Syncing locomotion (motors)..."
            ssh "$target" "mkdir -p $dest/locomotion"
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                "$SCRIPT_DIR/locomotion/" "$target:$dest/locomotion/"
            ;;
        pathfinder)
            log_info "Syncing pathfinder (LiDAR)..."
            ssh "$target" "mkdir -p $dest/pathfinder"
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                "$SCRIPT_DIR/pathfinder/" "$target:$dest/pathfinder/"
            ;;
        tests)
            log_info "Syncing tests..."
            ssh "$target" "mkdir -p $dest/tests"
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                --exclude='results/*.jpg' \
                --exclude='results/*.png' \
                "$SCRIPT_DIR/tests/" "$target:$dest/tests/"
            ;;
        scripts)
            log_info "Syncing scripts..."
            ssh "$target" "mkdir -p $dest/scripts"
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                "$SCRIPT_DIR/scripts/" "$target:$dest/scripts/"
            ssh "$target" "chmod +x $dest/scripts/*.sh"
            ;;
        docs)
            log_info "Syncing docs..."
            ssh "$target" "mkdir -p $dest/docs"
            eval rsync $RSYNC_BASE $RSYNC_EXCLUDE \
                "$SCRIPT_DIR/docs/" "$target:$dest/docs/"
            ;;
        *)
            log_error "Unknown component: $component"
            log_info "Valid components: all, bootylicious, locomotion, pathfinder, tests, scripts, docs"
            return 1
            ;;
    esac

    log_info "Synced $component to $target:$dest"
}

# =============================================================================
# Run Tests
# =============================================================================

run_tests() {
    local target="$1"
    local dest="$2"
    local test_type="${3:-all}"

    log_step "Running test '$test_type' on $target..."

    # Venv activation prefix for individual commands
    local venv_act="source $dest/venv/bin/activate 2>/dev/null;"

    case "$test_type" in
        # --- Test suites (run via run_tests.sh, activates venv itself) ---
        all)
            ssh "$target" "cd $dest && chmod +x run_tests.sh && ./run_tests.sh --all"
            ;;
        quick)
            ssh "$target" "cd $dest && chmod +x run_tests.sh && ./run_tests.sh --quick"
            ;;
        hardware|hw)
            ssh "$target" "cd $dest && chmod +x run_tests.sh && ./run_tests.sh --hardware"
            ;;
        integration)
            ssh "$target" "cd $dest && chmod +x run_tests.sh && ./run_tests.sh --integration"
            ;;
        verify)
            ssh "$target" "cd $dest && $venv_act python3 tests/verify_all_imports.py"
            ;;
        # --- Individual tests ---
        gpio)
            ssh "$target" "cd $dest/tests && $venv_act python3 test_gpio.py"
            ;;
        camera)
            ssh "$target" "cd $dest/tests && $venv_act python3 test_usb_camera.py"
            ;;
        camera-basic)
            ssh "$target" "cd $dest && $venv_act python3 tests/gui_camera.py --headless --captures 1 --no-faces"
            ;;
        camera-faces)
            ssh "$target" "cd $dest && $venv_act python3 tests/gui_camera.py --headless --captures 1 --faces"
            ;;
        lidar|ld19)
            ssh "$target" "cd $dest/tests && $venv_act python3 test_ld19_lidar.py"
            ;;
        motors)
            ssh "$target" "cd $dest && $venv_act python3 tests/test_motors.py --check"
            ;;
        motors-basic)
            ssh "$target" "cd $dest && $venv_act python3 tests/test_motors.py --basic"
            ;;
        motors-individual)
            ssh "$target" "cd $dest && $venv_act python3 tests/test_motors.py --individual"
            ;;
        motors-pinout)
            ssh "$target" "cd $dest && $venv_act python3 tests/test_motors.py --pinout"
            ;;
        wandering-viz)
            ssh "$target" "cd $dest && $venv_act python3 tests/gui_wandering.py --headless --scans 3"
            ;;
        imu)
            ssh "$target" "cd $dest && $venv_act python3 tests/test_imu.py"
            ;;
        # --- Diagnostics ---
        check)
            ssh "$target" "cd $dest && sudo ./install.sh --check"
            ;;
        env|env-diag)
            ssh "$target" "cd $dest && $venv_act python3 scripts/env_diagnostic.py"
            ;;
        env-fix)
            log_info "Setting up venv and installing packages..."
            ssh "$target" "cd $dest && sudo ./install.sh --gui --pathfinder"
            ;;
        *)
            log_warn "Unknown test type: $test_type"
            log_info "Test suites:    all, quick, hardware, integration, verify"
            log_info "Individual:     gpio, camera, camera-basic, camera-faces, lidar, motors, motors-basic, motors-individual, motors-pinout"
            log_info "Diagnostics:    check, env, env-fix"
            ;;
    esac
}

# =============================================================================
# Deploy Functions
# =============================================================================

deploy_rpi() {
    local component="${1:-all}"
    local bootstrap="${2:-false}"
    local run_test="${3:-false}"
    local test_type="${4:-}"

    log_step "Deploying to Raspberry Pi ($RPI_TARGET)..."

    # Check connection via SSH (not ping)
    log_info "Checking SSH connection (timeout=${SSH_TIMEOUT}s)..."
    if ! check_ssh "$RPI_TARGET"; then
        log_error "Cannot connect to Raspberry Pi at $RPI_TARGET"
        log_info "Troubleshooting:"
        log_info "  1. Check if RPi is powered on"
        log_info "  2. Verify IP address: $RPI_HOST"
        log_info "  3. Check SSH key: ssh-copy-id $RPI_TARGET"
        return 1
    fi

    log_info "Connected to Raspberry Pi"

    # Create base directory
    ssh "$RPI_TARGET" "mkdir -p $RPI_DEST"

    # Sync component(s)
    sync_component "$RPI_TARGET" "$RPI_DEST" "$component"

    # Run bootstrap if requested
    if [ "$bootstrap" = "true" ]; then
        log_step "Running bootstrap scripts..."
        ssh "$RPI_TARGET" "chmod +x $RPI_DEST/scripts/*.sh && $RPI_DEST/scripts/rpi-bootstrap.sh --auto"
    fi

    # Run tests if requested
    if [ "$run_test" = "true" ] && [ -n "$test_type" ]; then
        run_tests "$RPI_TARGET" "$RPI_DEST" "$test_type"
    fi

    echo ""
    log_info "Raspberry Pi deployment complete!"
}

deploy_jetson() {
    local component="${1:-all}"
    local bootstrap="${2:-false}"

    if [ -z "$JETSON_HOST" ]; then
        log_warn "Jetson host not configured."
        log_info "Set JETSON_HOST variable in this script when Jetson is available."
        return 0
    fi

    log_step "Deploying to Jetson ($JETSON_TARGET)..."

    if ! check_ssh "$JETSON_TARGET"; then
        log_error "Cannot connect to Jetson at $JETSON_TARGET"
        return 1
    fi

    log_info "Connected to Jetson"

    # Create base directory
    ssh "$JETSON_TARGET" "mkdir -p $JETSON_DEST"

    # Sync component(s)
    sync_component "$JETSON_TARGET" "$JETSON_DEST" "$component"

    # Run setup if bootstrap requested
    if [ "$bootstrap" = "true" ]; then
        log_step "Running Jetson setup scripts..."
        ssh "$JETSON_TARGET" "chmod +x $JETSON_DEST/bootylicious/scripts/*.sh"
        log_info "Run manually: $JETSON_DEST/bootylicious/scripts/setup-all.sh"
    fi

    echo ""
    log_info "Jetson deployment complete!"
}

# =============================================================================
# Print Help
# =============================================================================

print_help() {
    echo "Usage: $0 [TARGET] [COMPONENT] [OPTIONS]"
    echo ""
    echo "Targets:"
    echo "  rpi, raspberry, pi    Deploy to Raspberry Pi ($RPI_TARGET)"
    echo "  jetson, orin          Deploy to Jetson Orin Nano"
    echo "  (none)                Deploy to all available targets"
    echo ""
    echo "Components:"
    echo "  all                   All components (default)"
    echo "  bootylicious          LLM + RAG system"
    echo "  locomotion            Motor control"
    echo "  pathfinder            LiDAR + obstacle avoidance"
    echo "  tests                 Test scripts"
    echo "  scripts               Bootstrap scripts"
    echo "  docs                  Documentation"
    echo ""
    echo "Options:"
    echo "  --bootstrap, -b       Run bootstrap scripts after deploy"
    echo "  --test=SUITE          Run tests after deploy (see test types below)"
    echo "  --status, -s          Check connection status only"
    echo "  --help, -h            Show this help"
    echo ""
    echo "Test Types (--test=TYPE):"
    echo "  Suites:      all, quick, hardware, integration, verify"
    echo "  Individual:  gpio, camera, camera-basic, camera-faces, lidar, imu, motors, motors-basic, motors-individual, motors-pinout"
    echo "  Diagnostics: check, env (environment diagnostic), env-fix (install system-wide)"
    echo ""
    echo "Examples:"
    echo "  $0 rpi                        # Deploy all to RPi"
    echo "  $0 rpi pathfinder             # Deploy only pathfinder"
    echo "  $0 rpi --test=all             # Deploy + run ALL tests"
    echo "  $0 rpi --test=quick           # Deploy + quick verification"
    echo "  $0 rpi --test=hardware        # Deploy + test hardware"
    echo "  $0 rpi --test=lidar           # Deploy + single LiDAR test"
    echo "  $0 rpi --test=check           # Deploy + check packages"
    echo "  $0 rpi --bootstrap            # Deploy all + run bootstrap"
    echo "  $0 --status                   # Check all connections"
}

# =============================================================================
# Print Summary
# =============================================================================

print_summary() {
    echo ""
    echo "═══════════════════════════════════════════════════════════════"
    echo "Deployment Summary"
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
    echo "Quick commands:"
    echo "  SSH to RPi:        ssh $RPI_TARGET"
    echo "  Run all tests:     $0 rpi --test=all"
    echo "  Quick verify:      $0 rpi --test=quick"
    echo "  Hardware tests:    $0 rpi --test=hardware"
    echo "  Single test:       $0 rpi --test=lidar"
    echo "  Check packages:    $0 rpi --test=check"
    echo "  Env diagnostic:    $0 rpi --test=env"
    echo "  Fix env issues:    $0 rpi --test=env-fix"
    echo ""
    echo "Deploy components:"
    echo "  $0 rpi pathfinder  # LiDAR system"
    echo "  $0 rpi locomotion  # Motor control"
    echo "  $0 rpi tests       # Test scripts only"
    echo ""
    if [ -n "$JETSON_HOST" ]; then
        echo "Jetson:"
        echo "  SSH: ssh $JETSON_TARGET"
        echo "  Setup: $0 jetson --bootstrap"
    fi
    echo ""
}

# =============================================================================
# Main
# =============================================================================

main() {
    # Parse arguments
    TARGET=""
    COMPONENT="all"
    BOOTSTRAP=false
    STATUS_ONLY=false
    RUN_TESTS=false
    TEST_TYPE=""

    while [[ $# -gt 0 ]]; do
        case $1 in
            rpi|raspberry|pi)
                TARGET="rpi"
                shift
                ;;
            jetson|orin)
                TARGET="jetson"
                shift
                ;;
            bootylicious|locomotion|pathfinder|tests|scripts|docs|all)
                COMPONENT="$1"
                shift
                ;;
            --bootstrap|-b)
                BOOTSTRAP=true
                shift
                ;;
            --status|-s)
                STATUS_ONLY=true
                shift
                ;;
            --test=*)
                RUN_TESTS=true
                TEST_TYPE="${1#*=}"
                shift
                ;;
            # Backwards compatibility
            --verify|-v)
                RUN_TESTS=true
                TEST_TYPE="verify"
                shift
                ;;
            --full-test)
                RUN_TESTS=true
                TEST_TYPE="all"
                shift
                ;;
            --run|-r)
                RUN_TESTS=true
                TEST_TYPE="all"
                shift
                ;;
            --help|-h)
                print_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done

    print_banner

    # Status check only
    if [ "$STATUS_ONLY" = true ]; then
        check_status
        exit 0
    fi

    # Deploy based on target
    if [ -z "$TARGET" ]; then
        # Deploy to all available
        log_info "Deploying to all available targets..."
        echo ""

        if check_ssh "$RPI_TARGET"; then
            deploy_rpi "$COMPONENT" "$BOOTSTRAP" "$RUN_TESTS" "$TEST_TYPE"
        else
            log_warn "Raspberry Pi not available, skipping..."
        fi

        if [ -n "$JETSON_HOST" ] && check_ssh "$JETSON_TARGET"; then
            deploy_jetson "$COMPONENT" "$BOOTSTRAP"
        else
            log_warn "Jetson not available or not configured, skipping..."
        fi
    else
        # Deploy to specific target
        case "$TARGET" in
            rpi)
                deploy_rpi "$COMPONENT" "$BOOTSTRAP" "$RUN_TESTS" "$TEST_TYPE"
                ;;
            jetson)
                deploy_jetson "$COMPONENT" "$BOOTSTRAP"
                ;;
        esac
    fi

    print_summary
}

main "$@"
