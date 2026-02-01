#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Cleanup function
cleanup() {
    if [ -d "${TEMP_DIR:-}" ]; then
        log_info "Cleaning up temporary directory..."
        rm -rf "$TEMP_DIR"
    fi
}
trap cleanup EXIT

# Create a temporary directory for downloads and builds
TEMP_DIR=$(mktemp -d)
log_info "Using temporary directory: $TEMP_DIR"

install_linux_deps() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        DISTRO=$ID
    else
        log_error "Cannot identify Linux distribution."
        exit 1
    fi

    log_info "Detected Linux distribution: $DISTRO"

    if [[ "$DISTRO" == "ubuntu" || "$DISTRO" == "debian" || "$DISTRO" == "raspbian" || "$DISTRO" == "linuxmint" || "$DISTRO" == "pop" ]]; then
        # Check for sudo
        if [ "$EUID" -ne 0 ]; then
            log_warn "This script requires superuser privileges for package installation."
            sudo -v
        fi

        log_info "Updating package list..."
        sudo apt-get update

        log_info "Installing system build dependencies and libraries..."
        sudo apt-get install -y cmake build-essential unzip git wget pkg-config \
            libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
            libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran \
            libopencv-dev libeigen3-dev

        log_info "Dependencies installed via apt."
        
        # Check if running on Raspberry Pi
        if grep -q "Raspberry Pi" /sys/firmware/devicetree/base/model 2>/dev/null; then
            log_info "Raspberry Pi detected. Installing GStreamer libcamera plugin..."
            sudo apt-get install -y gstreamer1.0-libcamera gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
        fi

    elif [[ "$DISTRO" == "fedora" ]]; then
        log_info "Detected Fedora. Installing via dnf..."
        sudo dnf install -y cmake opencv-devel eigen3-devel gcc-c++ make

    elif [[ "$DISTRO" == "arch" || "$DISTRO" == "manjaro" ]]; then
        log_info "Detected Arch/Manjaro. Installing via pacman..."
        sudo pacman -S --noconfirm cmake opencv eigen base-devel

    else
        log_error "Unsupported Linux distribution: $DISTRO"
        log_info "Please manually install CMake, OpenCV, and Eigen."
        exit 1
    fi
}

install_mac_deps() {
    log_info "Detected macOS."
    
    if ! command_exists brew; then
        log_error "Homebrew not found. Please install Homebrew first: https://brew.sh/"
        exit 1
    fi

    log_info "Updating Homebrew..."
    brew update

    log_info "Installing dependencies via Homebrew..."
    brew install cmake opencv eigen

    log_info "macOS dependencies installed."
}

# Main Execution
echo "------------------------------------------------"
log_info "Starting Dependency Installation"
echo "------------------------------------------------"

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    install_linux_deps
elif [[ "$OSTYPE" == "darwin"* ]]; then
    install_mac_deps
else
    log_error "Unsupported Operating System: $OSTYPE"
    exit 1
fi

echo "------------------------------------------------"
log_info "Installation Complete!"
echo "------------------------------------------------"
