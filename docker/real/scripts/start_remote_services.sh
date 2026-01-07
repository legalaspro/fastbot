#!/bin/bash
# =============================================================================
# Startup script for Fastbot Remote Development Container
# Launches: ROSBridge, Web Video Server, TF2 Web Republisher, HTTP Server
#
# This runs on the dev machine and connects to Pi robot via CycloneDDS
# Note: ROS2 is sourced by ros_entrypoint.sh
# =============================================================================

set -e

echo "=============================================="
echo "Starting FastBot Remote Services"
echo "=============================================="
echo "CycloneDDS URI: ${CYCLONEDDS_URI:-not set}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "=============================================="

# Wait a moment for CycloneDDS to discover peers
echo "Waiting for CycloneDDS peer discovery..."
sleep 3

# Check if we can see topics from Pi
echo "Checking ROS2 topics from Pi..."
ros2 topic list || echo "Warning: No topics found yet (Pi might not be running)"

echo ""

# Start ROSBridge WebSocket Server (port 9090)
echo "[1/4] Starting ROSBridge WebSocket Server on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 &
sleep 2

# Start Web Video Server (port 11315)
echo "[2/4] Starting Web Video Server on port 11315..."
ros2 run web_video_server web_video_server --ros-args -p port:=11315 -p address:="0.0.0.0" &
sleep 1

# Start TF2 Web Republisher
echo "[3/4] Starting TF2 Web Republisher..."
ros2 run tf2_web_republisher_py tf2_web_republisher &
sleep 1

# Start Web App HTTP Server (port 8000)
echo "[4/4] Starting Web App Server on port 8000..."
cd /ros2_ws/src/fastbot_webapp && python3 -m http.server 8000 --bind 0.0.0.0 &

echo ""
echo "=============================================="
echo "All services started!"
echo "=============================================="
echo "  - ROSBridge WebSocket: ws://localhost:9090"
echo "  - Web Video Server:    http://localhost:11315"
echo "  - Web App:             http://localhost:8000"
echo ""
echo "To run RViz (in another terminal):"
echo "  docker exec -it fastbot-remote rviz2"
echo "=============================================="
echo ""

# Keep container running
wait

