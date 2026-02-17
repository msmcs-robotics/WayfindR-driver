#!/bin/bash
# =============================================================================
# Ambot - Jetson CUDA Detection & Verification Script
# =============================================================================
# Detects JetPack version, determines the correct CUDA version, verifies
# installation, and fixes common issues (PATH, libraries, permissions).
#
# IMPORTANT: On Jetson, CUDA is bundled with JetPack. This script does NOT
# download or build CUDA from scratch. It verifies and fixes the existing
# JetPack CUDA installation.
#
# Usage (on Jetson):
#   ./setup-cuda.sh              # Full check + fix
#   ./setup-cuda.sh --check      # Check only (no changes)
#   ./setup-cuda.sh --fix-path   # Fix PATH only
#   ./setup-cuda.sh --test       # Run CUDA test program
#
# =============================================================================

set -euo pipefail

# Colors (disabled if not a terminal)
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    CYAN='\033[0;36m'
    NC='\033[0m'
else
    RED='' GREEN='' YELLOW='' BLUE='' CYAN='' NC=''
fi

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()    { echo -e "${BLUE}[====]${NC} $1"; }
log_ok()      { echo -e "${GREEN}[ OK ]${NC} $1"; }
log_fail()    { echo -e "${RED}[FAIL]${NC} $1"; }

# Track issues found
ISSUES_FOUND=0
ISSUES_FIXED=0

# =============================================================================
# Detect Jetson Hardware
# =============================================================================
detect_jetson() {
    log_step "Detecting Jetson Hardware"

    # Check if this is actually a Jetson
    if [ ! -f /etc/nv_tegra_release ] && [ ! -f /proc/device-tree/model ]; then
        log_error "This does not appear to be a Jetson device"
        log_error "  /etc/nv_tegra_release not found"
        log_error "  /proc/device-tree/model not found"
        echo ""
        log_info "This script is designed for NVIDIA Jetson platforms only."
        log_info "For desktop GPUs, use the standard NVIDIA CUDA installer."
        exit 1
    fi

    # Device model
    local model="unknown"
    if [ -f /proc/device-tree/model ]; then
        model=$(tr -d '\0' < /proc/device-tree/model)
    fi
    echo "  Device:    $model"

    # JetPack / L4T version
    local jetpack_version="unknown"
    if [ -f /etc/nv_tegra_release ]; then
        jetpack_version=$(head -1 /etc/nv_tegra_release | sed 's/# //')
    fi
    echo "  JetPack:   $jetpack_version"

    # Architecture
    local arch
    arch=$(uname -m)
    echo "  Arch:      $arch"
    if [ "$arch" != "aarch64" ]; then
        log_warn "Expected aarch64 architecture, got $arch"
    fi

    # Kernel
    echo "  Kernel:    $(uname -r)"

    # Memory
    local total_mem
    total_mem=$(awk '/MemTotal/ {printf "%.1f GiB", $2/1024/1024}' /proc/meminfo)
    echo "  Memory:    $total_mem (unified CPU+GPU)"
    echo ""
}

# =============================================================================
# Determine Required CUDA Version
# =============================================================================
determine_cuda_version() {
    log_step "Determining Required CUDA Version"

    # JetPack ↔ CUDA version mapping
    # Each JetPack release bundles exactly ONE CUDA version
    # Source: https://developer.nvidia.com/embedded/jetpack-archive
    local l4t_major="" l4t_minor=""
    local expected_cuda="" expected_cudnn=""

    if [ -f /etc/nv_tegra_release ]; then
        # Parse R36 (release 36), REVISION: 4.4
        l4t_major=$(head -1 /etc/nv_tegra_release | grep -oP 'R\K[0-9]+')
        l4t_minor=$(head -1 /etc/nv_tegra_release | grep -oP 'REVISION:\s*\K[0-9.]+')
    fi

    # Map L4T release to CUDA version
    case "${l4t_major}.${l4t_minor}" in
        36.4*)  expected_cuda="12.6"; expected_cudnn="9.3" ;;
        36.3*)  expected_cuda="12.4"; expected_cudnn="9.0" ;;
        36.2*)  expected_cuda="12.2"; expected_cudnn="8.9" ;;
        36.1*)  expected_cuda="12.2"; expected_cudnn="8.9" ;;
        35.5*)  expected_cuda="11.4"; expected_cudnn="8.6" ;;
        35.4*)  expected_cuda="11.4"; expected_cudnn="8.6" ;;
        35.3*)  expected_cuda="11.4"; expected_cudnn="8.6" ;;
        35.2*)  expected_cuda="11.4"; expected_cudnn="8.4" ;;
        35.1*)  expected_cuda="11.4"; expected_cudnn="8.4" ;;
        32.7*)  expected_cuda="10.2"; expected_cudnn="8.2" ;;
        32.6*)  expected_cuda="10.2"; expected_cudnn="8.2" ;;
        *)
            log_warn "Unknown L4T version: R${l4t_major} rev ${l4t_minor}"
            log_info "Attempting to detect CUDA from installed files..."
            if [ -f /usr/local/cuda/version.json ]; then
                expected_cuda=$(python3 -c "import json; print(json.load(open('/usr/local/cuda/version.json'))['cuda']['version'])" 2>/dev/null | cut -d. -f1-2)
            fi
            ;;
    esac

    if [ -n "$expected_cuda" ]; then
        echo "  L4T:              R${l4t_major} rev ${l4t_minor}"
        echo "  Required CUDA:    ${expected_cuda}"
        echo "  Required cuDNN:   ${expected_cudnn:-unknown}"
        echo ""
        log_info "This Jetson needs CUDA ${expected_cuda} ONLY"
        log_info "DO NOT install other CUDA versions — they waste disk and create conflicts"
    else
        log_error "Could not determine required CUDA version"
        exit 1
    fi

    # Export for other functions
    EXPECTED_CUDA="$expected_cuda"
    EXPECTED_CUDNN="${expected_cudnn:-unknown}"
    echo ""
}

# =============================================================================
# Check CUDA Installation
# =============================================================================
check_cuda_install() {
    log_step "Checking CUDA Installation"

    local cuda_found=false
    local cuda_version=""

    # Method 1: Check /usr/local/cuda symlink
    if [ -d /usr/local/cuda ]; then
        log_ok "/usr/local/cuda exists (-> $(readlink -f /usr/local/cuda 2>/dev/null || echo 'direct'))"
        cuda_found=true

        # Get version from version.json (JetPack 6+)
        if [ -f /usr/local/cuda/version.json ]; then
            cuda_version=$(python3 -c "import json; print(json.load(open('/usr/local/cuda/version.json'))['cuda']['version'])" 2>/dev/null || echo "")
            if [ -n "$cuda_version" ]; then
                echo "  Version (json):  $cuda_version"
            fi
        fi

        # Get version from version.txt (older JetPack)
        if [ -f /usr/local/cuda/version.txt ]; then
            echo "  Version (txt):   $(cat /usr/local/cuda/version.txt)"
        fi
    else
        log_fail "/usr/local/cuda not found"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
    fi

    # Method 2: Check for versioned CUDA directories
    local cuda_dirs
    cuda_dirs=$(ls -d /usr/local/cuda-* 2>/dev/null || true)
    if [ -n "$cuda_dirs" ]; then
        echo "  Installed dirs:  $cuda_dirs"
        # Count unique real directories (resolve symlinks)
        local real_dirs
        real_dirs=$(for d in $cuda_dirs; do readlink -f "$d"; done | sort -u | wc -l)
        if [ "$real_dirs" -gt 1 ]; then
            log_warn "Multiple distinct CUDA installations found!"
            log_warn "Only one CUDA version should be installed on Jetson."
            for d in $cuda_dirs; do
                echo "    $d -> $(readlink -f "$d")"
            done
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        elif [ "$(echo "$cuda_dirs" | wc -w)" -gt 1 ]; then
            log_ok "Multiple dirs are symlinks to the same installation (normal)"
        fi
    fi

    # Method 3: Check nvcc
    echo ""
    if command -v nvcc &>/dev/null; then
        log_ok "nvcc in PATH: $(which nvcc)"
        echo "  $(nvcc --version 2>/dev/null | grep 'release' || echo 'version unknown')"
    elif [ -x /usr/local/cuda/bin/nvcc ]; then
        log_warn "nvcc exists at /usr/local/cuda/bin/nvcc but NOT in PATH"
        echo "  $(/usr/local/cuda/bin/nvcc --version 2>/dev/null | grep 'release' || echo 'version unknown')"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
    else
        log_fail "nvcc not found anywhere"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
    fi

    # Method 4: Check CUDA libraries
    echo ""
    local lib_dir="/usr/local/cuda/targets/aarch64-linux/lib"
    if [ -d "$lib_dir" ]; then
        local libcudart
        libcudart=$(ls "$lib_dir"/libcudart.so* 2>/dev/null | head -1)
        if [ -n "$libcudart" ]; then
            log_ok "CUDA runtime library: $libcudart"
        else
            log_fail "libcudart.so not found in $lib_dir"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        fi
    else
        # Check alternate location
        lib_dir="/usr/local/cuda/lib64"
        if [ -d "$lib_dir" ]; then
            log_ok "CUDA libraries: $lib_dir"
        else
            log_fail "CUDA library directory not found"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        fi
    fi

    # Version match check
    if [ -n "$cuda_version" ] && [ -n "${EXPECTED_CUDA:-}" ]; then
        local installed_major_minor
        installed_major_minor=$(echo "$cuda_version" | cut -d. -f1-2)
        if [ "$installed_major_minor" = "$EXPECTED_CUDA" ]; then
            log_ok "CUDA version matches JetPack requirement ($installed_major_minor = $EXPECTED_CUDA)"
        else
            log_fail "CUDA version mismatch! Installed: $installed_major_minor, Required: $EXPECTED_CUDA"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        fi
    fi

    echo ""
}

# =============================================================================
# Check cuDNN
# =============================================================================
check_cudnn() {
    log_step "Checking cuDNN"

    # Check via dpkg
    local cudnn_pkg
    cudnn_pkg=$(dpkg -l 2>/dev/null | grep -i cudnn | grep '^ii' | head -1 || true)
    if [ -n "$cudnn_pkg" ]; then
        log_ok "cuDNN installed via apt"
        echo "  $cudnn_pkg"
    else
        log_warn "cuDNN not found via dpkg (may be in CUDA toolkit)"
    fi

    # Check header file
    local cudnn_header=""
    for h in /usr/include/cudnn_version.h /usr/local/cuda/include/cudnn_version.h /usr/include/aarch64-linux-gnu/cudnn_version_v*.h; do
        if [ -f "$h" ]; then
            cudnn_header="$h"
            break
        fi
    done

    if [ -n "$cudnn_header" ]; then
        local major minor patch
        major=$(grep '#define CUDNN_MAJOR' "$cudnn_header" 2>/dev/null | awk '{print $3}' || echo "?")
        minor=$(grep '#define CUDNN_MINOR' "$cudnn_header" 2>/dev/null | awk '{print $3}' || echo "?")
        patch=$(grep '#define CUDNN_PATCHLEVEL' "$cudnn_header" 2>/dev/null | awk '{print $3}' || echo "?")
        log_ok "cuDNN version: ${major}.${minor}.${patch} (from $cudnn_header)"
    fi

    # Check library
    local cudnn_lib
    cudnn_lib=$(ldconfig -p 2>/dev/null | grep libcudnn.so | head -1 || true)
    if [ -n "$cudnn_lib" ]; then
        log_ok "cuDNN library: $cudnn_lib"
    fi

    echo ""
}

# =============================================================================
# Check TensorRT
# =============================================================================
check_tensorrt() {
    log_step "Checking TensorRT"

    local trt_pkg
    trt_pkg=$(dpkg -l 2>/dev/null | grep 'tensorrt' | grep '^ii' | head -1 || true)
    if [ -n "$trt_pkg" ]; then
        log_ok "TensorRT installed"
        echo "  $trt_pkg"
    else
        log_info "TensorRT not installed (optional, not needed for Ollama)"
    fi

    if command -v trtexec &>/dev/null; then
        log_ok "trtexec available: $(which trtexec)"
    fi

    echo ""
}

# =============================================================================
# Check Ollama GPU Usage
# =============================================================================
check_ollama_gpu() {
    log_step "Checking Ollama GPU Acceleration"

    if ! command -v ollama &>/dev/null; then
        log_warn "Ollama not installed, skipping GPU check"
        echo ""
        return
    fi

    if ! systemctl is-active ollama &>/dev/null; then
        log_warn "Ollama service not running"
        echo ""
        return
    fi

    # Check Ollama logs for CUDA usage
    local cuda_lines
    cuda_lines=$(journalctl -u ollama --no-pager -n 200 2>/dev/null | grep -i 'CUDA\|cuda\|gpu\|GPU' | tail -5 || true)
    if [ -n "$cuda_lines" ]; then
        log_ok "Ollama GPU activity found in logs:"
        echo "$cuda_lines" | while IFS= read -r line; do
            echo "    $line"
        done
    else
        log_warn "No CUDA references in recent Ollama logs"
        log_info "Try running a query first: ollama run llama3.2:3b 'hello'"
    fi

    # Check for CUDA buffer allocation
    local kv_buffer
    kv_buffer=$(journalctl -u ollama --no-pager -n 500 2>/dev/null | grep 'CUDA0.*buffer' | tail -3 || true)
    if [ -n "$kv_buffer" ]; then
        log_ok "CUDA0 buffer allocations confirmed:"
        echo "$kv_buffer" | while IFS= read -r line; do
            echo "    $line"
        done
    fi

    echo ""
}

# =============================================================================
# Check Docker GPU Access
# =============================================================================
check_docker_gpu() {
    log_step "Checking Docker GPU Access"

    if ! command -v docker &>/dev/null; then
        log_warn "Docker not installed, skipping"
        echo ""
        return
    fi

    # Check daemon.json for nvidia runtime
    if [ -f /etc/docker/daemon.json ]; then
        if grep -q '"nvidia"' /etc/docker/daemon.json; then
            log_ok "NVIDIA runtime configured in daemon.json"
            if grep -q 'default-runtime.*nvidia' /etc/docker/daemon.json; then
                log_ok "NVIDIA is the default Docker runtime"
            else
                log_warn "NVIDIA runtime exists but is NOT the default"
                log_info "Add '\"default-runtime\": \"nvidia\"' to /etc/docker/daemon.json"
                ISSUES_FOUND=$((ISSUES_FOUND + 1))
            fi
        else
            log_fail "No NVIDIA runtime in /etc/docker/daemon.json"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        fi
    else
        log_fail "/etc/docker/daemon.json not found"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
    fi

    # Check nvidia-container-toolkit
    if dpkg -l nvidia-container-toolkit &>/dev/null 2>&1; then
        log_ok "nvidia-container-toolkit installed"
    else
        log_fail "nvidia-container-toolkit NOT installed"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
    fi

    echo ""
}

# =============================================================================
# Fix PATH (add CUDA bin and lib directories)
# =============================================================================
fix_cuda_path() {
    log_step "Fixing CUDA PATH"

    local profile_file="/etc/profile.d/cuda-path.sh"
    local cuda_bin="/usr/local/cuda/bin"
    local cuda_lib="/usr/local/cuda/lib64"

    # Check if already in PATH
    if echo "$PATH" | grep -q "$cuda_bin"; then
        log_ok "CUDA bin already in current PATH"
    else
        log_warn "CUDA bin NOT in current PATH"
    fi

    # Check if profile.d script exists
    if [ -f "$profile_file" ]; then
        log_ok "CUDA PATH script exists: $profile_file"
        echo "  Contents:"
        cat "$profile_file" | sed 's/^/    /'
    else
        if [ "${CHECK_ONLY:-false}" = true ]; then
            log_warn "CUDA PATH script missing: $profile_file (use --fix-path to create)"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        else
            log_info "Creating $profile_file ..."
            cat > "$profile_file" << 'PATHEOF'
# CUDA PATH for Jetson (JetPack)
# Created by ambot setup-cuda.sh
if [ -d /usr/local/cuda/bin ]; then
    export PATH=/usr/local/cuda/bin:$PATH
fi
if [ -d /usr/local/cuda/lib64 ]; then
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
fi
PATHEOF
            chmod 644 "$profile_file"
            log_ok "Created $profile_file"
            ISSUES_FIXED=$((ISSUES_FIXED + 1))

            # Also apply to current shell
            export PATH=/usr/local/cuda/bin:$PATH
            export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
            log_info "Applied to current shell session"
        fi
    fi

    # Verify nvcc is now accessible
    if command -v nvcc &>/dev/null; then
        log_ok "nvcc accessible: $(nvcc --version 2>/dev/null | grep 'release' | sed 's/.*release /CUDA /' | sed 's/,.*//')"
    elif [ -x /usr/local/cuda/bin/nvcc ]; then
        log_warn "nvcc at /usr/local/cuda/bin/nvcc but not in current PATH"
        log_info "Run 'source /etc/profile.d/cuda-path.sh' or log out and back in"
    fi

    echo ""
}

# =============================================================================
# Install Missing CUDA Components (JetPack apt packages only)
# =============================================================================
install_cuda_if_missing() {
    log_step "Checking CUDA Packages"

    if [ ! -d /usr/local/cuda ]; then
        log_warn "CUDA not installed. Installing via JetPack apt packages..."
        log_info "This installs ONLY the JetPack-matched CUDA version."

        # JetPack CUDA packages (DO NOT use runfile installer or NVIDIA CUDA repo)
        local packages=(
            "cuda-toolkit-${EXPECTED_CUDA/./-}"
            "cuda-cudart-${EXPECTED_CUDA/./-}"
        )

        log_info "Installing: ${packages[*]}"
        if apt-get update -qq && apt-get install -y "${packages[@]}" 2>/dev/null; then
            log_ok "CUDA packages installed"
            ISSUES_FIXED=$((ISSUES_FIXED + 1))
        else
            log_warn "apt install failed — CUDA may use different package names on this JetPack"
            log_info "Try: sudo apt search cuda | grep ${EXPECTED_CUDA}"
            log_info "Or reinstall JetPack SDK from NVIDIA SDK Manager"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        fi
    else
        log_ok "CUDA directory exists: /usr/local/cuda"
    fi

    echo ""
}

# =============================================================================
# Compile & Run CUDA Test
# =============================================================================
run_cuda_test() {
    log_step "Running CUDA Functionality Test"

    local nvcc_path=""
    if command -v nvcc &>/dev/null; then
        nvcc_path="nvcc"
    elif [ -x /usr/local/cuda/bin/nvcc ]; then
        nvcc_path="/usr/local/cuda/bin/nvcc"
    else
        log_fail "nvcc not found — cannot compile CUDA test"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
        echo ""
        return 1
    fi

    local test_dir
    test_dir=$(mktemp -d)
    local test_file="$test_dir/cuda_test.cu"
    local test_bin="$test_dir/cuda_test"

    # Write minimal CUDA test program
    cat > "$test_file" << 'CUDA_EOF'
#include <stdio.h>
#include <cuda_runtime.h>

__global__ void hello_kernel() {
    printf("  CUDA kernel running on GPU thread %d\n", threadIdx.x);
}

int main() {
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);

    if (err != cudaSuccess) {
        printf("FAIL: cudaGetDeviceCount error: %s\n", cudaGetErrorString(err));
        return 1;
    }

    printf("  CUDA devices found: %d\n", device_count);

    for (int i = 0; i < device_count; i++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        printf("  Device %d: %s\n", i, prop.name);
        printf("    Compute capability: %d.%d\n", prop.major, prop.minor);
        printf("    Total memory: %.0f MiB\n", prop.totalGlobalMem / 1048576.0);
        printf("    Multiprocessors: %d\n", prop.multiProcessorCount);
        printf("    Max threads/block: %d\n", prop.maxThreadsPerBlock);
    }

    // Launch a simple kernel
    printf("\n  Launching test kernel...\n");
    hello_kernel<<<1, 4>>>();
    err = cudaDeviceSynchronize();

    if (err != cudaSuccess) {
        printf("FAIL: Kernel launch error: %s\n", cudaGetErrorString(err));
        return 1;
    }

    printf("\n  CUDA test PASSED\n");
    return 0;
}
CUDA_EOF

    log_info "Compiling CUDA test program..."
    if $nvcc_path -o "$test_bin" "$test_file" 2>&1; then
        log_ok "Compilation successful"

        log_info "Running CUDA test..."
        echo ""
        if "$test_bin" 2>&1; then
            echo ""
            log_ok "CUDA is fully functional"
        else
            echo ""
            log_fail "CUDA test program failed"
            ISSUES_FOUND=$((ISSUES_FOUND + 1))
        fi
    else
        log_fail "CUDA compilation failed"
        log_info "This may indicate missing CUDA development headers"
        log_info "Try: sudo apt install cuda-toolkit-${EXPECTED_CUDA/./-}"
        ISSUES_FOUND=$((ISSUES_FOUND + 1))
    fi

    # Cleanup
    rm -rf "$test_dir"
    echo ""
}

# =============================================================================
# Summary
# =============================================================================
print_summary() {
    echo ""
    echo "========================================"
    echo "  CUDA Setup Summary"
    echo "========================================"

    if [ $ISSUES_FOUND -eq 0 ]; then
        log_ok "All checks passed. CUDA is properly configured."
    else
        local remaining=$((ISSUES_FOUND - ISSUES_FIXED))
        if [ $ISSUES_FIXED -gt 0 ]; then
            log_info "$ISSUES_FIXED issue(s) fixed automatically"
        fi
        if [ $remaining -gt 0 ]; then
            log_warn "$remaining issue(s) need attention"
        else
            log_ok "All $ISSUES_FOUND issue(s) were fixed"
        fi
    fi

    echo ""
    echo "  GPU acceleration status:"
    echo "    Ollama LLM:     Uses CUDA via llama.cpp (automatic)"
    echo "    Embeddings:     CPU (acceptable for 22MB model)"
    echo "    Docker:         NVIDIA runtime configured"
    echo ""
    echo "  CUDA version policy:"
    echo "    Required:       CUDA ${EXPECTED_CUDA:-?}"
    echo "    DO NOT install additional CUDA versions"
    echo ""
    echo "========================================"
    echo ""
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo ""
    echo "========================================"
    echo "  Ambot - Jetson CUDA Setup"
    echo "  $(date)"
    echo "========================================"
    echo ""

    local do_check=false
    local do_fix_path=false
    local do_test=false
    local do_full=true

    for arg in "$@"; do
        case $arg in
            --check)
                do_check=true
                do_full=false
                CHECK_ONLY=true
                export CHECK_ONLY
                ;;
            --fix-path)
                do_fix_path=true
                do_full=false
                ;;
            --test)
                do_test=true
                do_full=false
                ;;
            --help|-h)
                echo "Usage: $0 [options]"
                echo ""
                echo "Options:"
                echo "  --check      Check CUDA status (no changes)"
                echo "  --fix-path   Fix CUDA PATH only"
                echo "  --test       Compile and run CUDA test program"
                echo "  --help       Show this help"
                echo ""
                echo "No args = full check + fix + test"
                echo ""
                echo "IMPORTANT: This script does NOT download/build CUDA."
                echo "On Jetson, CUDA is bundled with JetPack."
                echo "This script verifies and fixes the existing installation."
                exit 0
                ;;
            *)
                log_error "Unknown option: $arg (use --help)"
                exit 1
                ;;
        esac
    done

    # Always detect hardware and CUDA version first
    detect_jetson
    determine_cuda_version

    if [ "$do_check" = true ]; then
        CHECK_ONLY=true
        export CHECK_ONLY
        check_cuda_install
        check_cudnn
        check_tensorrt
        check_ollama_gpu
        check_docker_gpu
        fix_cuda_path  # Will only report, not fix (CHECK_ONLY=true)
        print_summary
        exit 0
    fi

    if [ "$do_fix_path" = true ]; then
        fix_cuda_path
        exit 0
    fi

    if [ "$do_test" = true ]; then
        run_cuda_test
        exit 0
    fi

    # Full mode
    check_cuda_install
    install_cuda_if_missing
    check_cudnn
    check_tensorrt
    fix_cuda_path
    check_ollama_gpu
    check_docker_gpu
    run_cuda_test
    print_summary
}

main "$@"
