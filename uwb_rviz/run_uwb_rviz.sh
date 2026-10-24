#!/bin/bash
# Show the Go2 UWB remote in RViz
# Usage: bash run_uwb_rviz.sh [--no-rviz] [node ROS args, e.g. -p bearing_field:=yaw_est -p bearing_sign:=-1.0]
#   --no-rviz : only run the converter node (e.g. view from another machine on the same DDS network)
# RViz opens on $DISPLAY, or the Jetson's local screen (:0) if DISPLAY is not set.

DIR=$(cd "$(dirname "$0")" && pwd)
RVIZ=1
if [ "$1" = "--no-rviz" ]; then RVIZ=0; shift; fi

source ~/unitree_ros2/setup.sh > /dev/null

if [ $RVIZ = 0 ]; then
    exec python3 "$DIR/uwb_viz_node.py" --ros-args "$@"
fi

python3 "$DIR/uwb_viz_node.py" --ros-args "$@" &
NODE_PID=$!
trap 'kill -INT $NODE_PID 2>/dev/null; wait $NODE_PID 2>/dev/null' EXIT

DISPLAY=${DISPLAY:-:0} rviz2 -d "$DIR/uwb.rviz"
