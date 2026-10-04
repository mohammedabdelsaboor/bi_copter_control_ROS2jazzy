#!/bin/bash
# ==============================================================================
# BI-COPTER PX4 AUTOPILOT SITL INSTALLATION & LINKING SCRIPT
# ==============================================================================
#
# Hey there! This script bridges our custom bi-copter drone into the official
# PX4-Autopilot flight stack for Software-In-The-Loop (SITL) simulation in Gazebo.
#
# Reference Guide:
# Modeled after the official Gazebo-PX4 setup architecture:
# https://github.com/MrStealYoCurls/Gazebo-PX4-Setup-Guide
#
# What this script does automatically:
# 1. Checks if PX4-Autopilot exists locally (or clones it if missing).
# 2. Registers our custom bi-copter airframe (4005_bicopter) in PX4's ROMFS.
# 3. Copies the bi_copter Gazebo model into PX4's model library.
# 4. Installs the bicopter_world arena into PX4's Gazebo worlds directory.
# 5. Configures all Gazebo resource paths so PX4 can spawn the model seamlessly.
#
# Usage:
#   ./setup_px4_sitl.sh [path/to/PX4-Autopilot]
#
# If no path is provided, it defaults to: $HOME/PX4-Autopilot
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_SRC=$(cd "${PKG_DIR}/.." && pwd)

# Target PX4 directory (defaults to $HOME/PX4-Autopilot)
PX4_DIR="${1:-$HOME/PX4-Autopilot}"

echo "======================================================================"
echo "    PX4 AUTOPILOT SITL SETUP FOR DUAL-TILT BI-COPTER"
echo "======================================================================"
echo "Package Directory : ${PKG_DIR}"
echo "PX4 Target Path   : ${PX4_DIR}"
echo "======================================================================"

# 1. Clone PX4-Autopilot if not already present
if [ ! -d "${PX4_DIR}" ]; then
    echo ""
    echo "[1/5] Cloning PX4-Autopilot repository..."
    echo "Cloning into: ${PX4_DIR}"
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
    echo ""
    echo "[1/5] Found existing PX4-Autopilot at: ${PX4_DIR}"
fi

# 2. Register Bi-copter Airframe (4005_bicopter) in PX4 ROMFS
echo ""
echo "[2/5] Registering bi_copter airframe in PX4 ROMFS..."
AIRFRAME_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"
mkdir -p "${AIRFRAME_DIR}"
cp -v "${SCRIPT_DIR}/4005_bicopter" "${AIRFRAME_DIR}/4005_bicopter"
chmod +x "${AIRFRAME_DIR}/4005_bicopter"

# Also add to CMakeLists.txt if airframe list is manually tracked in that version
AIRFRAME_CMAKE="${AIRFRAME_DIR}/CMakeLists.txt"
if [ -f "${AIRFRAME_CMAKE}" ]; then
    if ! grep -q "4005_bicopter" "${AIRFRAME_CMAKE}"; then
        echo "Adding 4005_bicopter to ${AIRFRAME_CMAKE}..."
        sed -i '/px4_add_airframe/a \    4005_bicopter' "${AIRFRAME_CMAKE}" 2>/dev/null || true
    fi
fi

# 3. Install Bi-Copter Model in PX4 Gazebo Models Directory
echo ""
echo "[3/5] Installing bi_copter model into PX4 Gazebo directory..."
PX4_GZ_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
mkdir -p "${PX4_GZ_MODELS}/bi_copter"
cp -r "${PKG_DIR}/models/bi_copter/"* "${PX4_GZ_MODELS}/bi_copter/"

# 4. Install Bi-Copter Arena World in PX4 Gazebo Worlds Directory
echo ""
echo "[4/5] Installing bicopter_world.sdf into PX4 Gazebo worlds directory..."
PX4_GZ_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
mkdir -p "${PX4_GZ_WORLDS}"
cp -v "${PKG_DIR}/worlds/bicopter_world.sdf" "${PX4_GZ_WORLDS}/bicopter_world.sdf"

# 5. Verification & Instructions
echo ""
echo "[5/5] Setup completed successfully!"
echo "======================================================================"
echo "HOW TO RUN PX4 SITL SIMULATION:"
echo "======================================================================"
echo "1. Run the simulation launcher:"
echo "   cd ${SCRIPT_DIR}"
echo "   ./run_px4_sitl.sh ${PX4_DIR}"
echo ""
echo "   Or run directly inside PX4-Autopilot:"
echo "   cd ${PX4_DIR}"
echo "   export GZ_SIM_RESOURCE_PATH=\"${WS_SRC}:${PKG_DIR}:${PX4_DIR}/Tools/simulation/gz/models\""
echo "   export PX4_GZ_WORLD=bicopter_world"
echo "   make px4_sitl gz_bi_copter"
echo ""
echo "2. Connect QGroundControl:"
echo "   Open QGroundControl -> Automatically connects via UDP 14550."
echo "   View live Artificial Horizon, arm, tune PIDs, and fly with joysticks!"
echo "======================================================================"
