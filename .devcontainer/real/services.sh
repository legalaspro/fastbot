#!/bin/bash
# =============================================================================
# ROS2 Web Services Manager for Fastbot Real Robot Devcontainer
# 
# Usage:
#   ./services.sh start   - Start all services in background
#   ./services.sh stop    - Stop all services
#   ./services.sh restart - Restart all services
#   ./services.sh status  - Show status of all services
#   ./services.sh logs    - Tail logs from all services
#   ./services.sh save-map [name] - Save map from cartographer (optional name)
# =============================================================================

set -e

# Configuration
PID_DIR="/tmp/fastbot-services"
LOG_DIR="/tmp/fastbot-services/logs"
SERVICES=("rosbridge" "webvideo" "tf2web" "httpserver")

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Ensure ROS2 is sourced
source_ros() {
    source /opt/ros/humble/setup.bash
    if [ -f /ros2_ws/install/setup.bash ]; then
        source /ros2_ws/install/setup.bash
    fi
}

# Create directories
init_dirs() {
    mkdir -p "$PID_DIR" "$LOG_DIR"
}

# Start a single service
start_service() {
    local name=$1
    local cmd=$2
    local pid_file="$PID_DIR/${name}.pid"
    local log_file="$LOG_DIR/${name}.log"
    
    if [ -f "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
        echo -e "${YELLOW}[$name]${NC} Already running (PID: $(cat "$pid_file"))"
        return 0
    fi
    
    echo -e "${GREEN}[$name]${NC} Starting..."
    nohup bash -c "$cmd" > "$log_file" 2>&1 &
    local pid=$!
    echo $pid > "$pid_file"
    sleep 1
    
    if kill -0 $pid 2>/dev/null; then
        echo -e "${GREEN}[$name]${NC} Started (PID: $pid)"
    else
        echo -e "${RED}[$name]${NC} Failed to start! Check: $log_file"
        return 1
    fi
}

# Stop a single service
stop_service() {
    local name=$1
    local pid_file="$PID_DIR/${name}.pid"
    
    if [ ! -f "$pid_file" ]; then
        echo -e "${YELLOW}[$name]${NC} Not running (no PID file)"
        return 0
    fi
    
    local pid=$(cat "$pid_file")
    if kill -0 "$pid" 2>/dev/null; then
        echo -e "${YELLOW}[$name]${NC} Stopping (PID: $pid)..."
        kill "$pid" 2>/dev/null || true
        sleep 1
        # Force kill if still running
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
        echo -e "${GREEN}[$name]${NC} Stopped"
    else
        echo -e "${YELLOW}[$name]${NC} Already stopped"
    fi
    rm -f "$pid_file"
}

# Check status of a service
check_service() {
    local name=$1
    local pid_file="$PID_DIR/${name}.pid"
    
    if [ -f "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
        echo -e "${GREEN}[$name]${NC} Running (PID: $(cat "$pid_file"))"
    else
        echo -e "${RED}[$name]${NC} Stopped"
        [ -f "$pid_file" ] && rm -f "$pid_file"
    fi
}

# Start all services
cmd_start() {
    echo "=============================================="
    echo "Starting FastBot Web Services"
    echo "=============================================="
    init_dirs
    source_ros
    
    # ROSBridge WebSocket (port 9090)
    start_service "rosbridge" \
        "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0"
    
    sleep 2
    
    # Web Video Server (port 11315)
    start_service "webvideo" \
        "source /opt/ros/humble/setup.bash && ros2 run web_video_server web_video_server --ros-args -p port:=11315 -p address:='0.0.0.0'"
    
    # TF2 Web Republisher
    start_service "tf2web" \
        "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; ros2 run tf2_web_republisher_py tf2_web_republisher"
    
    # HTTP Server for WebApp (port 8000)
    start_service "httpserver" \
        "cd /ros2_ws/src/fastbot_webapp && python3 -m http.server 8000 --bind 0.0.0.0"
    
    echo ""
    echo "=============================================="
    echo "Services started! Ports:"
    echo "  - ROSBridge:   ws://localhost:9090"
    echo "  - Web Video:   http://localhost:11315"
    echo "  - WebApp:      http://localhost:8000"
    echo "  - noVNC:       http://localhost:6080"
    echo ""
    echo "Logs: $LOG_DIR/"
    echo "=============================================="
}

# Stop all services
cmd_stop() {
    echo "Stopping all services..."
    for svc in "${SERVICES[@]}"; do
        stop_service "$svc"
    done
    # Also kill any orphaned processes
    pkill -f "rosbridge_websocket" 2>/dev/null || true
    pkill -f "web_video_server" 2>/dev/null || true
    pkill -f "tf2_web_republisher" 2>/dev/null || true
    pkill -f "http.server 8000" 2>/dev/null || true
    echo "All services stopped."
}

# Show status
cmd_status() {
    echo "Service Status:"
    echo "---------------"
    for svc in "${SERVICES[@]}"; do
        check_service "$svc"
    done
}

# Tail logs
cmd_logs() {
    if [ ! -d "$LOG_DIR" ]; then
        echo "No logs found. Start services first."
        exit 1
    fi
    echo "Tailing logs (Ctrl+C to exit)..."
    tail -f "$LOG_DIR"/*.log
}

# Save map (stops services, saves, restarts)
cmd_save_map() {
    local map_name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
    local map_dir="/ros2_ws/maps"  # External maps dir (not in package, no rebuild needed)

    source_ros

    echo "=============================================="
    echo "Saving map: $map_name"
    echo "=============================================="

    # Check if map topic exists
    if ! ros2 topic list | grep -q "/map"; then
        echo -e "${RED}Error: /map topic not found!${NC}"
        echo "Is cartographer running on the robot?"
        exit 1
    fi

    # Stop services to free up resources
    echo "Stopping services..."
    cmd_stop
    sleep 2

    # Save the map
    echo "Saving map to: $map_dir/$map_name"
    mkdir -p "$map_dir"
    cd "$map_dir"

    if ros2 run nav2_map_server map_saver_cli -f "$map_name"; then
        echo -e "${GREEN}Map saved successfully!${NC}"
        echo "Files created:"
        ls -la "$map_dir/$map_name"*
    else
        echo -e "${RED}Failed to save map!${NC}"
    fi

    # Restart services
    echo ""
    echo "Restarting services..."
    cmd_start
}

# Main
case "${1:-}" in
    start)
        cmd_start
        ;;
    stop)
        cmd_stop
        ;;
    restart)
        cmd_stop
        sleep 2
        cmd_start
        ;;
    status)
        cmd_status
        ;;
    logs)
        cmd_logs
        ;;
    save-map)
        cmd_save_map "$2"
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|logs|save-map [name]}"
        exit 1
        ;;
esac

