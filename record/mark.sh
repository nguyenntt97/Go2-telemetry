#!/bin/bash
# Label an event in the running recording session
# Usage: bash record/mark.sh "turn 90 left"
# Publishes the text on /annotations (recorded in the bag) and appends it with a
# Jetson timestamp to meta/annotations.csv of the newest session in logs/.

TEXT=${1:?Please give a label, e.g. bash record/mark.sh "start circle"}
LOGS=$(cd "$(dirname "$0")/.." && pwd)/logs

SESSION=$(ls -1dt "$LOGS"/*/ 2>/dev/null | head -1)
if [ -n "$SESSION" ]; then
    printf '%s,%s\n' "$(date +%s.%N)" "$(printf '%s' "$TEXT" | sed 's/"/""/g; s/.*/"&"/')" >> "${SESSION}meta/annotations.csv"
fi

source ~/unitree_ros2/setup.sh > /dev/null
# JSON is valid YAML, so json.dumps escapes any quotes in the label
MSG=$(python3 -c 'import json,sys; print(json.dumps({"data": sys.argv[1]}))' "$TEXT")
# -w 1: wait until the recorder has subscribed, so the message is not lost
timeout 10 ros2 topic pub -1 -w 1 /annotations std_msgs/msg/String "$MSG" > /dev/null \
    && echo "marked: $TEXT" || echo "marked in annotations.csv only (recorder did not subscribe to /annotations)"
