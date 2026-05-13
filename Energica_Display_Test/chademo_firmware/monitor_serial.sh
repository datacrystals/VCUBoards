#!/usr/bin/env bash
# =============================================================================
# monitor_serial.sh — Read and log USB serial output from RP2040/RP2350
# =============================================================================
#
# Usage:
#   ./monitor_serial.sh              # Auto-detect port, log to timestamped file
#   ./monitor_serial.sh /dev/ttyACM0 # Use specific port
#
# The Pico outputs debug info over USB CDC (virtual serial). This script
# connects to it, prints to the terminal, and saves everything to:
#   logs/serial_YYYY-MM-DD_HH-MM-SS.log
#
# Press Ctrl+C to stop.
# =============================================================================

set -euo pipefail

# ---- Colors ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${BLUE}[INFO]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
error() { echo -e "${RED}[ERR ]${NC}  $1"; }
ok()    { echo -e "${GREEN}[OK  ]${NC}  $1"; }

# ---- Settings ----
BAUD=115200
LOG_DIR="logs"
PORT="${1:-}"

# =============================================================================
# Detect Pico serial port
# =============================================================================
detect_port() {
    local candidates=(/dev/ttyACM* /dev/ttyUSB*)
    local found=""

    for dev in "${candidates[@]}"; do
        if [[ -e "$dev" ]]; then
            found="$dev"
            break
        fi
    done

    if [[ -z "$found" ]]; then
        return 1
    fi

    echo "$found"
}

wait_for_port() {
    local timeout_sec=60
    local waited=0

    info "Looking for Pico USB serial device..."
    while (( waited < timeout_sec )); do
        local dev
        if dev=$(detect_port 2>/dev/null); then
            echo "$dev"
            return 0
        fi

        printf "\r${CYAN}[WAIT]${NC}  Waiting for /dev/ttyACM* or /dev/ttyUSB* ... %2ds / ${timeout_sec}s" "$waited"
        sleep 1
        waited=$((waited + 1))
    done
    printf "\n"
    error "No serial device found within ${timeout_sec}s."
    error "Make sure the Pico is plugged in and has USB stdio enabled."
    exit 1
}

# =============================================================================
# Main
# =============================================================================
main() {
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║     Energica Display Probe — Serial Monitor & Logger         ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # Determine serial port
    if [[ -n "$PORT" ]]; then
        if [[ ! -e "$PORT" ]]; then
            error "Specified port $PORT does not exist."
            exit 1
        fi
        ok "Using specified port: $PORT"
    else
        PORT=$(wait_for_port)
        ok "Found serial device: $PORT"
    fi

    # Create log directory
    mkdir -p "$LOG_DIR"
    local timestamp
    timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
    local logfile="${LOG_DIR}/serial_${timestamp}.log"

    info "Log file: $logfile"
    info "Baud rate: $BAUD"
    echo ""
    echo -e "${GREEN}══════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  Starting capture — Press Ctrl+C to stop${NC}"
    echo -e "${GREEN}══════════════════════════════════════════════════════════════${NC}"
    echo ""

    # Configure terminal: raw mode, no echo, specified baud
    # USB CDC doesn't strictly need baud rate, but set it anyway for compatibility
    stty -F "$PORT" raw "$BAUD" -echo cs8 -cstopb -parenb 2>/dev/null || {
        error "Failed to configure $PORT. Try running with sudo?"
        exit 1
    }

    # Flush any buffered data
    sleep 0.2
    dd if="$PORT" of=/dev/null bs=4096 iflag=nonblock 2>/dev/null || true

    # Start reading and logging
    # We use 'cat' piped to 'tee' so output goes to both terminal and file.
    # Wrap in a subshell so we can print a friendly exit message on Ctrl+C.
    (
        cat "$PORT" | tee "$logfile"
    ) &
    local pid=$!

    # Handle Ctrl+C gracefully
    trap 'echo ""; info "Stopping capture..."; kill $pid 2>/dev/null; wait $pid 2>/dev/null; ok "Log saved to: $logfile"; exit 0' INT TERM

    wait $pid || true

    echo ""
    ok "Capture ended. Log saved to: $logfile"
}

main "$@"
