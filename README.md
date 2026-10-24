# Go2 EDU telemetry recording

Tools that run on the Jetson payload (`192.168.123.18`, interface `enP8p1s0`) to record and look at telemetry from a Unitree Go2 EDU. The Go2 main board is at `192.168.123.161`. Everything uses ROS 2 Humble with CycloneDDS from `~/unitree_ros2`.

| Path | What it is | Status |
|---|---|---|
| `uwb_rviz/` | Live RViz view of the UWB remote | Working |
| `record/` | Recorder for robot telemetry, saved to `logs/` | Recording works; `export.py` not built yet |
| `logs/` | Recording sessions | Created by the recorder |

---

## UWB remote in RViz (`uwb_rviz/`)

```bash
bash uwb_rviz/run_uwb_rviz.sh              # node + RViz (on $DISPLAY, or :0 if not set)
bash uwb_rviz/run_uwb_rviz.sh --no-rviz    # node only
```

The node turns `/uwbstate` into `/uwb/markers`, `/uwb/tag_pose` and a TF frame `base_link -> uwb_tag`.

Unitree does not document the angle fields in `UwbState`, so the mapping from fields to position is set with parameters:

```bash
bash uwb_rviz/run_uwb_rviz.sh -p bearing_field:=yaw_est -p bearing_sign:=-1.0 -p elevation_field:=none
```

| Parameter | Default | Meaning |
|---|---|---|
| `bearing_field` | `orientation_est` | Field used as the horizontal angle to the remote |
| `bearing_sign`, `bearing_offset` | `1.0`, `0.0` | Sign flip and offset (rad) for the bearing |
| `elevation_field` | `pitch_est` | Field used as the up/down angle (`none` puts the remote on the ground) |
| `frame_id` | `base_link` | Frame the markers are drawn in |
| `trail_length` | `150` | Number of past positions in the trail |
| `stale_timeout` | `1.0` | Seconds without data before the remote is greyed out |

The defaults are a guess and have not been checked against real movement. See [UWB calibration](#uwb-calibration-session).

### What the UWB data contains

`/uwbstate` (`unitree_go/msg/UwbState`) arrives at about 5–6 Hz, with gaps of up to 0.7 s.

- Header: `version`, `channel`, `joy_mode`
- Remote position relative to the robot: `distance_est` (looks like metres), `orientation_est`, `pitch_est`, `yaw_est`
- Remote attitude: `tag_roll`, `tag_pitch`, `tag_yaw`
- Robot attitude: `base_roll`, `base_pitch`, `base_yaw`
- Remote inputs: `joystick[2]`, `buttons`
- Status: `error_state`, `enabled_from_app`

There is **no gyro or accelerometer data from the remote** on any topic, only the filtered attitude angles above. Angular rate can only be estimated by differencing `tag_*` over time, which is limited to slow motion (below about 3 Hz) by the 6 Hz message rate.

---

## Telemetry recorder (`record/`)

```bash
bash record/go2_record.sh uwb_calibration                    # standard profile, until Ctrl+C
bash record/go2_record.sh walk_test --seconds 300 --note "hallway, 2 people"
bash record/go2_record.sh mapping --profile full --no-audio
bash record/mark.sh "start circle"                           # label an event in the running session
python3 record/summarize.py logs/<session>                   # re-run the per-topic summary
python3 record/verify.py logs/<session>                      # re-run the reading checks
```

Each run checks the network, disk space and key topics, then records until the time is up or Ctrl+C. While recording it logs Jetson hardware once per second to `meta/hardware.csv`. At the end it prints:

- a per-topic summary (count, rate, largest gap, size), saved to `meta/summary.csv`
- reading checks with OK / WARN per item, saved to `meta/verify.txt`. These cover body state, IMU, battery and cells, motor temperatures, lidar health, UWB, video stream structure and Jetson load.

`hardware.csv` columns: CPU load (total and per core), CPU frequency, load average, RAM and swap, GPU load, temperatures (cpu, gpu, soc0–2, tj), power rails (VDD_IN, VDD_CPU_GPU_CV, VDD_SOC), fan, disk free and read/write rate, network rx/tx for `enP8p1s0` and Wi-Fi, and CPU and memory of the recorder process.

## Implementation plan

Goal: record the Go2's telemetry into `logs/`, in a form that is easy to analyse later. Steps 1–4 below are built and tested; step 5 (`export.py`) is not built yet.

Scope decisions:

- **Detailed motor data is left out.** `/lowstate` and `/lowcmd` (500 Hz, about 1 MB/s together) are not recorded. `/lf/lowstate` (20 Hz) is kept, because it is cheap and also carries the robot IMU, foot force and battery.
- **The D435i is deferred.** The recorder covers the Go2 only; the camera can be added later.
- **SLAM is captured only when it is already running.** The SDK has no API to start or control SLAM, so the recorder does not try. If SLAM is started from the Unitree app, its topics (`/uslam/*`, `/lio_sam_ros2/*`, `/slam_info`, `/slam_key_info`) are recorded automatically.

### Measured topic inventory

Measured on 2026-10-23 with 10 s captures, robot standing, SLAM not running.

| Group | Topics | Rate | Bandwidth | Recorded |
|---|---|---|---|---|
| Body state (pose, velocity, gait) | `/sportmodestate` | 300 Hz | 151 KB/s | yes |
| Low-rate state (IMU, foot force, battery, motors at 20 Hz) | `/lf/lowstate`, `/lf/sportmodestate` | 20 Hz | 33 KB/s | yes |
| Lidar IMU and odometry | `/utlidar/imu`, `/utlidar/robot_odom`, `/utlidar/robot_pose`, `/utlidar/lidar_state` | 250 / 150 / 19 / 5 Hz | 188 KB/s | yes |
| UWB | `/uwbstate` | 5–6 Hz | 0.3 KB/s | yes |
| Lidar point cloud | `/utlidar/cloud` | 15 Hz | 664 KB/s | `standard`, `full` |
| Lidar point cloud, robot frame | `/utlidar/cloud_base` | 15 Hz | 116 KB/s | `standard`, `full` |
| Front camera (compressed video) | `/frontvideostream` | 29 Hz | 229 KB/s | `standard`, `full` |
| Deskewed point cloud | `/utlidar/cloud_deskewed` | 15 Hz | 4.7 MB/s | `full` only |
| Battery cells | `/lf/battery_alarm` (per-cell voltages, JSON) | 1 Hz | small | yes |
| Commands and status | `/api/*/request` and `/response` (22 services, e.g. `sport`, `obstacles_avoid`, `uwbswitch`, `vui`), `/multiplestate`, `/audiohub/player/state`, `/rtc/state`, `/public_network_status`, `/gpt_state` | event-driven | small | yes |
| SLAM and mapping | `/uslam/*`, `/lio_sam_ros2/mapping/odometry`, `/slam_info`, `/slam_key_info`, `/utlidar/height_map*`, `grid_map`, `range_map`, `voxel_map`, `range_info` | silent unless SLAM runs | unknown | yes, when publishing |
| Controller | `/wirelesscontroller`, `/wirelesscontroller_unprocessed` | silent unless used | small | yes |
| Detailed motor data | `/lowstate`, `/lowcmd` | 500 Hz | 970 KB/s | **no** |
| Microphone (probably) | `/audiosender` | 50 Hz | 9 KB/s | yes |

### Recording approach: everything except an exclude list

`ros2 bag record -a` on its own does not work. Seven message types published by the robot are not built on this Jetson, and when `-a` reaches them its topic discovery fails and it silently skips other topics, including `/lowstate`, `/uwbstate` and all of `/utlidar/*`.

Recording all topics with those types excluded works. Tested on 2026-10-23: no discovery failures, all 92 recordable topics subscribed, data on the 27 that were active.

```bash
ros2 bag record -a -x '/(arm_Command|arm_Feedback|qt_.*|query_result_.*|pctoimage_local|config_change_status|utlidar/voxel_map_compressed)$'
```

Because the recorder subscribes to everything, topics that only appear when a feature is used (SLAM, controller, voice, `/api/*` calls) are captured without listing them. Each profile is that base exclude list plus a few more exclusions:

| Profile | Also excludes | Approx. size (SLAM off) |
|---|---|---|
| `lite` | `/lowstate`, `/lowcmd`, `/utlidar/cloud`, `/utlidar/cloud_base`, `/utlidar/cloud_deskewed`, `/frontvideostream` | 0.4 MB/s, 1.4 GB/h |
| `standard` (default) | `/lowstate`, `/lowcmd`, `/utlidar/cloud_deskewed` | 1.4 MB/s, 5 GB/h |
| `full` | `/lowstate`, `/lowcmd` | 6 MB/s, 22 GB/h |

`/audiosender` (probably the robot's microphone) is recorded in every profile; `--no-audio` excludes it. SLAM point clouds add an unmeasured amount on top when SLAM runs.

### Session layout

```
logs/<name>_<YYYYmmdd_HHMMSS>/
  go2/          rosbag2 recording, split every 5 min
  meta/         session.yaml (name, profile, start/end, notes, versions)
                topics_snapshot.txt (ros2 topic list -t at start)
                preflight.txt (key/SLAM topic message counts before start)
                hardware.csv (Jetson hardware, 1 Hz; see above)
                tegrastats.log (raw tegrastats output, 1 Hz)
                annotations.csv (mark.sh labels with Jetson time)
                summary.csv (per-topic count, rate, max gap, size)
                verify.txt (reading checks)
  record.log
```

### Implementation steps

1. **`record/profiles.yaml`**: the base exclude list and the extra exclusions for each profile, kept in one place. **`record/check_topics.py`** listens for 3 s to report which key topics and SLAM topics are publishing (used by the pre-flight check).
2. **`record/go2_record.sh <name> [--profile lite|standard|full] [--seconds N] [--no-audio]`**: the main recorder.
   - Checks before starting: `enP8p1s0` is up, `192.168.123.161` answers a ping, free disk space is more than twice the expected size, and the key topics (`/sportmodestate`, `/lf/lowstate`, `/uwbstate`, `/utlidar/robot_odom`) have a publisher (a warning, not an abort). Notes in `session.yaml` whether SLAM topics are publishing.
   - Writes the metadata snapshot into `meta/`.
   - Starts `ros2 bag record -a -x <profile regex>` with `--max-bag-duration 300` and `--storage-preset-profile resilient`, so a crash loses at most the current chunk.
   - Starts **`record/hwmon.py`** (writes `meta/hardware.csv`) and `tegrastats` (writes `meta/tegrastats.log`), to tell whether dropouts came from the Jetson being overloaded.
   - On exit (time up or Ctrl+C), stops everything cleanly and runs the summary and the reading checks.
3. **`record/mark.sh "text"`**: publishes a `std_msgs/String` on `/annotations` so events can be labelled during a session (for example "start circle", "turn 90 left").
4. **`record/summarize.py <session>`**: writes per-topic count, rate, largest gap and size to `meta/summary.csv`, and flags dropouts by comparing against the measured rates above. **`record/verify.py <session>`** decodes the key topics and checks the values are plausible (`meta/verify.txt`).
5. **`record/export.py <session>`**: flattens a session into one Parquet or CSV file per topic, timestamped by when the Jetson received each message:
   - `sportmodestate`: position, velocity, yaw rate, gait
   - `lf/lowstate`: IMU, foot force, battery (motor fields skipped)
   - `uwbstate`, lidar and SLAM odometry, controller input, annotations, battery cells, `/api/*` calls
   - Point clouds (`cloud_base`, SLAM maps) optionally to `.npz` per frame
   - **Video needs a custom parser.** The local `Go2FrontVideoData` definition does not match what the robot sends (decoding fails with "invalid data size"), but the raw bytes are recorded intact. Layout of each message: 4-byte CDR header, `uint64` time_frame, `uint32` resolution (720 or 360), `uint32` length, then that many bytes of H.264 Annex-B. Each stream (720p and 360p) arrives at about 14 Hz with a keyframe roughly every 2 s. A first test on 2026-10-23 decoded 702 of 707 frames per stream with OpenCV, at 1280×720 and 640×360; the 5 lost frames were before the first keyframe. ffmpeg is not installed; OpenCV's built-in FFmpeg can write the `.mp4`.
6. **Time alignment**: all topics share the Jetson's receive clock, so they line up without extra work.
7. **Optional, later**: get the missing `.msg` files from a newer `unitree_ros2` release, rebuild `cyclonedds_ws`, and drop `/utlidar/voxel_map_compressed` and `/config_change_status` from the base exclude list. The `unitree_interfaces` and `unitree_arm` types are probably not public and are not needed for analysis.

### Not covered

- **Raw IMU of the UWB remote and the main board's internal logs**: these stay on the main board at `192.168.123.161`, which the Jetson cannot reach.
- **Starting or controlling SLAM**: no SDK API. Start it from the app if a session needs it.
- **D435i**: deferred.

### Validation

- Run a 10-minute `standard` session and a 2-minute `full` session.
- Check that the rates in `summary.csv` match the inventory table, that `/lowstate` and `/lowcmd` are absent, and that `record.log` has no "Failure in topics discovery" lines.
- Check Jetson load in `meta/hardware.csv` (summarized at the end of `meta/verify.txt`).
- If possible, run one session with SLAM started from the app, to measure its topics and bandwidth.

### UWB calibration session

The first real session, used to work out what the UWB angle fields mean. Robot standing still, recorder running, `mark.sh` labelling each step:

1. Walk a circle around the robot at about 2 m, facing it. The field that sweeps 360° is the bearing; its direction gives the sign.
2. Stand in front of the robot and turn your body 90° left, then right, without moving your feet. A field that changes while the bearing stays fixed is the relative heading.
3. Stand still for 60 s. Slow change in that field is drift.
4. Optionally, rotate the robot in place from the app, to see whether `base_yaw` and `tag_yaw` share a reference.

The results set the defaults for `uwb_rviz/`.

### Decisions

- `/audiosender` is recorded by default (2026-10-23). Note that it may capture people's speech.
- `standard` is the default profile (2026-10-23).
# Go2-telemetry
