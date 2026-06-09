#!/usr/bin/env bash
set -e

# ROS2 + Gazebo startup for LLM robot agent
#
# Usage:
#   bash run_gazebo.sh          # headless (no GUI, most stable)
#   bash run_gazebo.sh --gui    # with Gazebo GUI (requires GPU)
#
# Prerequisites:
#   export ARK_API_KEY=your_key_here
#   (or set in ~/.bashrc)

MODE="${1:-headless}"
WORLD="${2:-office}"  # office or empty

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== ROS2+Gazebo Startup ==="
echo "ROS distro: kilted"
echo "Workspace: $WS_DIR"
echo "World: $WORLD"

# Check API key
if [ -z "$ARK_API_KEY" ] && [ -z "$(grep ARK_API_KEY ~/.bashrc 2>/dev/null)" ]; then
    echo "WARNING: ARK_API_KEY not set. LLM will fail."
    echo "  export ARK_API_KEY=your_key_here"
fi

# Source ROS2 and workspace overlay
source /opt/ros/kilted/setup.bash
source "$WS_DIR/install/setup.bash"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down..."
    pkill -9 -f "gz sim" 2>/dev/null || true
    kill -9 $BRIDGE_PID 2>/dev/null || true
    kill -9 $LAUNCH_PID 2>/dev/null || true
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

# Kill leftover processes from previous runs
lsof -ti:9090 2>/dev/null | xargs kill -9 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 2

# Select world file and set model path
case "$WORLD" in
    office)
        WORLD_FILE="$WS_DIR/install/robot_sim_gz/share/robot_sim_gz/worlds/office_simple.sdf"
        # Add building_L1 model path
        OFFICE_MODELS="$WS_DIR/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models"
        export GZ_SIM_RESOURCE_PATH="$OFFICE_MODELS:$GZ_SIM_RESOURCE_PATH"
        echo "  World: office_simple (building_L1 + doors)"
        ;;
    *)
        WORLD_FILE="$WS_DIR/install/robot_sim_gz/share/robot_sim_gz/worlds/empty_5ms.sdf"
        echo "  World: empty_5ms (flat ground)"
        ;;
esac

# 1) Start Gazebo
echo ""
echo "[1/4] Starting Gazebo..."
case "$MODE" in
    --gui|gui)
        echo "  Mode: GUI (server + GUI)"
        gz sim -r -v 1 "$WORLD_FILE" &
        ;;
    *)
        echo "  Mode: headless (server only)"
        gz sim -r -s --headless-rendering -v 1 "$WORLD_FILE" &
        ;;
esac
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"
sleep 4

# 2) Spawn robots + ROS-Gazebo bridges
echo ""
echo "[2/4] Spawning robots and starting ROS-Gazebo bridges..."
ros2 launch robot_sim_gz two_robot_sim.launch.py &
LAUNCH_PID=$!
sleep 4

# 3) Start WebSocket bridge
echo ""
echo "[3/4] Starting WebSocket bridge (ws://localhost:9090)..."
source /opt/ros/kilted/setup.bash
source "$WS_DIR/install/setup.bash"
ros2 run robot_sim_bridge bridge_node &
BRIDGE_PID=$!
echo "  Bridge PID: $BRIDGE_PID"
sleep 2

# 4) Start Go agent
echo ""
echo "[4/4] Starting Go agent (--simulator=ros2)..."
echo ""

cd "$SCRIPT_DIR"
GOTOOLCHAIN=local GOPROXY=https://goproxy.cn,direct ~/go1.23/bin/go run . --simulator=ros2 --ws=ws://localhost:9090
