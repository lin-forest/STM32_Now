#!/bin/bash
# flash_all.sh — Build & flash all firmware nodes in the correct order
set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

flash_node() {
    local name="$1"
    local path="$2"
    echo ""
    echo "=== [${name}] Building & Flashing ==="
    cmake --build "${ROOT_DIR}/${path}/build/Debug" --target flash
}

flash_node "3_MCLM_t2"         "3_MotorControl_LowMachine/lin_cmake/3_MCLM_t2"
flash_node "ChassisController"  "5_Tec_USART/5_ChassisController_t1"

echo ""
echo "=== All nodes flashed successfully ==="
