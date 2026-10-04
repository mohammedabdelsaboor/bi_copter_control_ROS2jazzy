#!/bin/bash
# ==============================================================================
# BI-COPTER PX4 AUTOPILOT SITL LAUNCHER
# ==============================================================================
#
# Welcome! This script starts the full PX4-Autopilot Software-In-The-Loop (SITL)
# simulation with our custom bi-copter drone in Gazebo Harmonic.
#
# What this script does:
# 1. Finds your PX4-Autopilot repository (defaults to $HOME/PX4-Autopilot).
# 2. Configures environment variables (GZ_SIM_RESOURCE_PATH, PX4_GZ_WORLD, etc.).
# 3. Ensures the airframe definition (4005_bicopter) and world are in place.
# 4. Compiles and boots PX4 SITL: `make px4_sitl gz_bi_copter`
# 5. Automatically begins broadcasting MAVLink on UDP 14550 so QGroundControl
#    can connect instantly!
#
# Usage:
#   ./run_px4_sitl.sh [path/to/PX4-Autopilot]
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_SRC=$(cd "${PKG_DIR}/.." && pwd)

# Search for PX4 repository if not provided
PX4_DIR="${1:-$HOME/PX4-Autopilot}"

if [ ! -d "${PX4_DIR}" ]; then
    echo "ERROR: PX4-Autopilot directory not found at '${PX4_DIR}'."
    echo "Please specify your PX4-Autopilot path:"
    echo "  ./run_px4_sitl.sh /path/to/PX4-Autopilot"
    exit 1
fi

echo "Found PX4-Autopilot at: ${PX4_DIR}"

# 1. Export Gazebo resource paths to include bi_copter model and meshes
export GZ_SIM_RESOURCE_PATH="${WS_SRC}:${PKG_DIR}/models:${PKG_DIR}:${PX4_DIR}/Tools/simulation/gz/models"
export GZ_IP="127.0.0.1"

# 2. Tell PX4 which world and model to simulate
export PX4_GZ_WORLD="bicopter_world"
export PX4_GZ_MODEL_NAME="bi_copter"

# 3. Copy/Link the airframe file into PX4 ROMFS if not already present
AIRFRAME_DEST="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4005_bicopter"
if [ ! -f "${AIRFRAME_DEST}" ]; then
    echo "Registering airframe 4005_bicopter into PX4 ROMFS..."
    cp "${SCRIPT_DIR}/4005_bicopter" "${AIRFRAME_DEST}"
fi

# 4. Copy/Link world into PX4 worlds folder
WORLD_DEST="${PX4_DIR}/Tools/simulation/gz/worlds/bicopter_world.sdf"
if [ ! -f "${WORLD_DEST}" ]; then
    echo "Linking bicopter_world.sdf into PX4 worlds directory..."
    cp "${PKG_DIR}/worlds/bicopter_world.sdf" "${WORLD_DEST}"
fi

# 5. Launch PX4 SITL with Gazebo Harmonic
cd "${PX4_DIR}"
echo "Starting PX4 SITL with bi_copter..."
make px4_sitl gz_bi_copter
