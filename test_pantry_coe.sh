#!/usr/bin/env bash
set -e

# Test: "一号去pantry，同时送货到coe"
# Records video with timestamps for later annotation

MODE="${1:---gui}"  # --gui or --headless

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
VIDEO_FILE="$SCRIPT_DIR/test_${TIMESTAMP}.mp4"

# Check API key
if [ -z "$ARK_API_KEY" ]; then
    echo "ERROR: ARK_API_KEY not set"
    exit 1
fi

echo "=== Test: 一号去pantry，同时送货到coe ==="
echo "Mode: $MODE"
echo "Video: $VIDEO_FILE"

# Source ROS2
source /opt/ros/kilted/setup.bash
source "$WS_DIR/install/setup.bash"

# Cleanup leftover processes
lsof -ti:9090 2>/dev/null | xargs kill -9 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "two_robot_sim" 2>/dev/null || true
pkill -9 -f "bridge_node" 2>/dev/null || true
sleep 2

# Start Gazebo
echo ""
echo "[1] Starting Gazebo ($MODE)..."
OFFICE_MODELS="$WS_DIR/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models"
export GZ_SIM_RESOURCE_PATH="$OFFICE_MODELS:$GZ_SIM_RESOURCE_PATH"
WORLD_FILE="$WS_DIR/install/robot_sim_gz/share/robot_sim_gz/worlds/office_simple.sdf"

if [ "$MODE" = "--gui" ]; then
    gz sim -r -v 1 "$WORLD_FILE" &
else
    gz sim -r -s --headless-rendering -v 1 "$WORLD_FILE" &
fi
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"
sleep 5

# Start ROS2 launch
echo ""
echo "[2] Spawning robots..."
ros2 launch robot_sim_gz two_robot_sim.launch.py &
LAUNCH_PID=$!
sleep 5

# Start WebSocket bridge
echo ""
echo "[3] Starting WebSocket bridge..."
ros2 run robot_sim_bridge bridge_node &
BRIDGE_PID=$!
sleep 3

# Start recording with ffmpeg (if available, otherwise skip)
RECORD_PID=""
if command -v ffmpeg &>/dev/null; then
    echo ""
    echo "[Recording] Starting video capture..."
    # Use x11grab for GUI window capture
    if [ "$MODE" = "--gui" ]; then
        # Find Gazebo window
        sleep 2
        ffmpeg -f x11grab -video_size 1280x720 -i :0 -c:v libx264 -preset ultrafast -qp 0 -y "$VIDEO_FILE" &
        RECORD_PID=$!
        echo "  Recording PID: $RECORD_PID (video: $VIDEO_FILE)"
    else
        echo "  (skipping: headless mode)"
    fi
else
    echo "  (ffmpeg not found, skipping recording)"
fi

# Wait a bit for everything to stabilize
echo ""
echo "[4] Starting agent (auto-piped commands)..."
echo ""
sleep 2

# Pipe commands: status → task → confirm → status → quit
printf '/status\n一号去pantry，同时送货到coe\n1\n/status\n/quit\n' | \
    GOTOOLCHAIN=local GOPROXY=https://goproxy.cn,direct ~/go1.23/bin/go run . \
    --simulator=ros2 --ws=ws://localhost:9090

EXIT_CODE=$?

# Stop recording
if [ -n "$RECORD_PID" ]; then
    echo ""
    echo "[Cleanup] Stopping recording..."
    kill $RECORD_PID 2>/dev/null || true
    wait $RECORD_PID 2>/dev/null || true
    echo "  Video saved: $VIDEO_FILE"
fi

# Stop processes
echo ""
echo "[Cleanup] Shutting down..."
kill $BRIDGE_PID 2>/dev/null || true
kill $LAUNCH_PID 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
wait 2>/dev/null || true

echo ""
echo "=== Test complete (exit code: $EXIT_CODE) ==="
exit $EXIT_CODE
