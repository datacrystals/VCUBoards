#!/usr/bin/env bash
# =============================================================================
# flash.sh — Auto-detect, Build, and Flash CHAdeMO firmware
# =============================================================================

set -euo pipefail

# ---- Colors for output ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ---- Configuration ----
BUILD_DIR="${1:-build}"
ROLE="${2:-VEHICLE}" # Default to VEHICLE if not specified

RP2040_LABEL="RPI-RP2"
RP2350_LABEL="RP2350"

OS=$(uname -s)

# =============================================================================
# Helper functions
# =============================================================================

info() { echo -e "${BLUE}[INFO]${NC}  $1" >&2; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $1" >&2; }
error() { echo -e "${RED}[ERR ]${NC}  $1" >&2; }
success() { echo -e "${GREEN}[OK  ]${NC}  $1" >&2; }
die() { error "$1"; exit 1; }

# =============================================================================
# Detect RP2040/RP2350 device
# =============================================================================
detect_device() {
    local label="$1"
    case "$OS" in
    Linux)
        local dev_info
        dev_info=$(lsblk -o NAME,LABEL,MOUNTPOINT,TYPE,SIZE -nr 2>/dev/null | grep -i "$label" | head -1 || true)
        if [[ -z "$dev_info" ]]; then return 1; fi
        local part_name=$(echo "$dev_info" | awk '{print $1}')
        local mount_point=$(lsblk -o NAME,MOUNTPOINT -nr "/dev/$part_name" 2>/dev/null | awk '{$1=""; print substr($0,2)}' || true)
        echo "/dev/$part_name|$mount_point"
        return 0
        ;;
    Darwin)
        local vol_path="/Volumes/$label"
        if [[ -d "$vol_path" ]]; then
            echo "osx|$vol_path"
            return 0
        fi
        return 1
        ;;
    *) die "Unsupported OS: $OS" ;;
    esac
}

ensure_mounted() {
    local dev="$1"
    local current_mount="$2"
    local label="$3"

    if [[ -n "$current_mount" && "$current_mount" != " " && -d "$current_mount" ]]; then
        echo "$current_mount"
        return 0
    fi

    info "Device $dev not mounted. Attempting to mount..."
    if command -v udisksctl &>/dev/null; then
        local mount_output
        if mount_output=$(udisksctl mount -b "$dev" 2>&1); then
            local mount_point=$(echo "$mount_output" | grep -oP 'Mounted .* at \K.*' || true)
            if [[ -n "$mount_point" && -d "$mount_point" ]]; then
                echo "$mount_point"
                return 0
            fi
        fi
    fi

    local fallback_mount="/tmp/pico-flash-$label"
    mkdir -p "$fallback_mount"
    if sudo mount "$dev" "$fallback_mount" 2>/dev/null; then
        echo "$fallback_mount"
        return 0
    fi
    die "Failed to mount $dev."
}

wait_for_device() {
    local timeout_sec=30
    local waited=0

    info "Looking for RP2040/RP2350 in BOOTSEL mode..."
    while (( waited < timeout_sec )); do
        if dev_info=$(detect_device "$RP2040_LABEL" 2>/dev/null); then
            echo "RP2040|$dev_info"
            return 0
        fi
        if dev_info=$(detect_device "$RP2350_LABEL" 2>/dev/null); then
            echo "RP2350|$dev_info"
            return 0
        fi

        printf "\r${CYAN}[WAIT]${NC}  Searching for device... %2ds / ${timeout_sec}s" "$waited" >&2
        sleep 1
        waited=$((waited + 1))
    done
    printf "\n" >&2
    die "No Pico device detected. Ensure it's in BOOTSEL mode."
}

verify_flash() {
    local mount_point="$1"
    local dev="$2"
    info "Waiting for device to reboot..."
    local waited=0
    local max_wait=15

    while (( waited < max_wait )); do
        sleep 1
        waited=$((waited + 1))
        if [[ "$OS" == "Darwin" ]]; then
            if [[ ! -d "$mount_point" ]]; then
                success "Device rebooted successfully."
                return 0
            fi
        else
            if [[ ! -b "$dev" ]] || ! mountpoint -q "$mount_point" 2>/dev/null; then
                success "Device rebooted successfully."
                return 0
            fi
        fi
        printf "\r${CYAN}[WAIT]${NC}  Verifying reboot... %2ds / ${max_wait}s" "$waited" >&2
    done
    printf "\n" >&2
    warn "Device did not disappear within ${max_wait}s."
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║     CHAdeMO Controller — Auto-Build & Flash Utility          ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"

    # 1. Detect device FIRST
    DETECTION=$(wait_for_device)
    PICO_MODEL=$(echo "$DETECTION" | cut -d'|' -f1)
    DEVICE=$(echo "$DETECTION" | cut -d'|' -f2)
    MOUNT=$(echo "$DETECTION" | cut -d'|' -f3)
    success "Found $PICO_MODEL at $DEVICE"

    # 2. Map model to platform flag
    local PLATFORM_FLAG="rp2040"
    if [[ "$PICO_MODEL" == "RP2350" ]]; then
        PLATFORM_FLAG="rp2350"
    fi

    # 3. Configure and Build
    info "Configuring CMake for $PICO_MODEL ($ROLE)..."
    cmake -B "$BUILD_DIR" -DCHADEMO_ROLE="$ROLE" -DPICO_PLATFORM="$PLATFORM_FLAG" >/dev/null

    info "Compiling firmware..."
    make -C "$BUILD_DIR" -j4 >/dev/null || die "Compilation failed!"
    success "Build complete."

    # 4. Find the UF2
    local uf2_pattern="chademo_controller_${ROLE,,}*.uf2"
    UF2_FILE=$(find "$BUILD_DIR" -maxdepth 1 -name "$uf2_pattern" -print -quit 2>/dev/null || true)
    [[ -n "$UF2_FILE" ]] || die "UF2 file not found after build."

    # 5. Mount and Flash
    if [[ "$OS" == "Darwin" ]]; then
        MOUNT_POINT="$MOUNT"
    else
        local label=$(lsblk -no LABEL "$DEVICE" 2>/dev/null || echo "")
        [[ -z "$label" ]] && label="$RP2040_LABEL"
        MOUNT_POINT=$(ensure_mounted "$DEVICE" "$MOUNT" "$label")
    fi

    info "Copying $(basename "$UF2_FILE") to device..."
    if cp "$UF2_FILE" "$MOUNT_POINT/"; then
        success "UF2 copied successfully."
        sync
    else
        die "Failed to copy UF2."
    fi

    if [[ "$OS" != "Darwin" ]]; then
        verify_flash "$MOUNT_POINT" "$DEVICE"
    else
        sleep 3
    fi

    echo -e "\n${GREEN}══════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  Flash complete! Target: $PICO_MODEL | Role: $ROLE${NC}"
    echo -e "${GREEN}══════════════════════════════════════════════════════════════${NC}"
}

main "$@"
