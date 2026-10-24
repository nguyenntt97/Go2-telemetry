#!/bin/bash
# Record Go2 telemetry into logs/<name>_<time>/
# Usage: bash record/go2_record.sh <name> [--profile lite|standard|full] [--seconds N] [--no-audio] [--note "text"]
#   --profile   lite: no clouds/video, standard (default): + clouds/video, full: + deskewed cloud
#   --seconds   stop after N seconds (default: run until Ctrl+C)
#   --no-audio  do not record /audiosender
#   --note      free text stored in meta/session.yaml
# Output: logs/<name>_<time>/{go2/ (rosbag2), meta/, record.log}
# Profiles and exclusions are in record/profiles.yaml. See README.md for details.

set -m   # background jobs get their own process group, so Ctrl+C only reaches this script
DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$DIR")
IFACE=enP8p1s0
ROBOT_IP=192.168.123.161

NAME=${1:?Please give a name, e.g. bash record/go2_record.sh uwb_calibration --seconds 300}
shift
PROFILE=standard; SEC=""; AUDIO=1; NOTE=""
while [ $# -gt 0 ]; do
    case $1 in
        --profile) PROFILE=$2; shift 2 ;;
        --seconds) SEC=$2; shift 2 ;;
        --no-audio) AUDIO=0; shift ;;
        --note) NOTE=$2; shift 2 ;;
        *) echo "unknown option: $1"; exit 1 ;;
    esac
done

# Exclude regex and expected rate from profiles.yaml
read -r EXCLUDE RATE < <(python3 - "$DIR/profiles.yaml" "$PROFILE" "$AUDIO" <<'EOF'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1])); prof, audio = sys.argv[2], sys.argv[3] == "1"
if prof not in cfg["profiles"]:
    sys.exit(f"unknown profile '{prof}', choose from {list(cfg['profiles'])}")
topics = cfg["base_exclude"] + cfg["profiles"][prof] + ([] if audio else cfg["audio"])
print("^(" + "|".join(topics) + ")$", cfg["rate_mb_s"][prof])
EOF
)
[ -z "$EXCLUDE" ] && exit 1

STAMP=$(date +%Y%m%d_%H%M%S)
SESSION=$ROOT/logs/${NAME}_${STAMP}
mkdir -p "$SESSION/meta"
echo "jetson_time,text" > "$SESSION/meta/annotations.csv"
source ~/unitree_ros2/setup.sh > /dev/null

# 1. Pre-flight checks
echo "== Pre-flight"
if ! ip link show "$IFACE" | grep -q 'state UP'; then
    echo "$IFACE is not up"; exit 1
fi
if ! ping -c1 -W1 $ROBOT_IP > /dev/null; then
    echo "Go2 main board $ROBOT_IP does not answer ping"; exit 1
fi
# Free space must cover twice the expected size (1 h if no --seconds)
NEED_MB=$(python3 -c "print(int(2 * $RATE * ${SEC:-3600}))")
FREE_MB=$(df -m --output=avail "$ROOT/logs" | tail -1)
if [ "$FREE_MB" -lt "$NEED_MB" ]; then
    echo "only ${FREE_MB} MB free, need ${NEED_MB} MB"; exit 1
fi
python3 "$DIR/check_topics.py" 3 > "$SESSION/meta/preflight.txt" 2>/dev/null \
    || echo "WARNING: some key topics are silent, see meta/preflight.txt"
SLAM=$(awk '/^slam_active/ {print $2}' "$SESSION/meta/preflight.txt")
sed 's/^/   /' "$SESSION/meta/preflight.txt"
timeout 15 ros2 topic list -t > "$SESSION/meta/topics_snapshot.txt" 2>/dev/null

cat > "$SESSION/meta/session.yaml" <<EOF
name: $NAME
profile: $PROFILE
audio: $([ $AUDIO = 1 ] && echo true || echo false)
seconds: ${SEC:-null}
note: "$(printf '%s' "$NOTE" | sed 's/"/\\"/g')"
exclude_regex: '$EXCLUDE'
slam_active_at_start: ${SLAM:-unknown}
start: $(date --iso-8601=seconds)
host: $(hostname)
ros_distro: $ROS_DISTRO
l4t: "$(head -1 /etc/nv_tegra_release 2>/dev/null | cut -d, -f1-2)"
EOF

# 2. Jetson health + bag recording in the background
# (< /dev/null: ros2 bag reads the keyboard; without this it gets stopped in the background)
tegrastats --interval 1000 --logfile "$SESSION/meta/tegrastats.log" &
TEGRA_PID=$!
disown $TEGRA_PID   # no "Terminated" job notice when it is stopped
ros2 bag record -a -x "$EXCLUDE" -o "$SESSION/go2" \
    --max-bag-duration 300 --storage-preset-profile resilient \
    < /dev/null > "$SESSION/record.log" 2>&1 &
BAG_PID=$!
# Jetson CPU/RAM/GPU/temps/power/disk/network + recorder usage, 1 Hz
python3 "$DIR/hwmon.py" "$SESSION/meta/hardware.csv" --pid $BAG_PID --disk-path "$ROOT/logs" &
HW_PID=$!
disown $HW_PID

# On exit (time up or Ctrl+C), close the bag cleanly and summarize
stop_all() {
    trap - EXIT INT TERM
    kill -INT $BAG_PID 2>/dev/null
    wait $BAG_PID 2>/dev/null
    kill $HW_PID 2>/dev/null
    tegrastats --stop > /dev/null 2>&1; kill $TEGRA_PID 2>/dev/null
    echo "end: $(date --iso-8601=seconds)" >> "$SESSION/meta/session.yaml"
    echo "== Summary"
    python3 "$DIR/summarize.py" "$SESSION"
    echo "== Reading checks (full report: meta/verify.txt)"
    python3 "$DIR/verify.py" "$SESSION" | grep -E '^== |WARN|warning'
    if grep -q -i 'failure in topics discovery' "$SESSION/record.log"; then
        echo "WARNING: topic discovery failures in record.log; a message type may be missing from base_exclude"
    fi
    echo "Session: $SESSION"
}
trap stop_all EXIT
trap 'exit 130' INT TERM

sleep 4
if [ ! -d "$SESSION/go2" ]; then
    echo "bag failed to start, see $SESSION/record.log"; exit 1
fi
echo "== Recording profile '$PROFILE' -> $SESSION"
echo "   Label events with: bash $DIR/mark.sh \"text\""
if [ -n "$SEC" ]; then
    echo "   Stopping in $SEC s (Ctrl+C to stop early)"
    sleep "$SEC" & wait $!
else
    echo "   Press Ctrl+C to stop"
    wait $BAG_PID
fi
