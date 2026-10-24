# Go2 EDU telemetry recording

Tools that run on the Jetson payload (`192.168.123.18`, interface `enP8p1s0`) to record and look at telemetry from a Unitree Go2 EDU. The Go2 main board is at `192.168.123.161`. Everything uses ROS 2 Humble with CycloneDDS from `~/unitree_ros2`.

| Path | What it is | Status |
|---|---|---|
| `uwb_rviz/` | Live RViz view of the UWB remote | Working |
| `record/view.py` | Rerun server for a recorded session, for a remote viewer | Working |
| `record/` | Recorder for robot telemetry, saved to `logs/` | Working |
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
python3 record/export.py logs/<session>                      # flatten to one table per topic
python3 record/export.py logs/<session> --list               # what is in the session
python3 record/view.py                                       # pick a session, view it in Rerun
```

Each run checks the network, disk space and key topics, then records until the time is up or Ctrl+C. While recording it logs Jetson hardware once per second to `meta/hardware.csv`. At the end it prints:

- a per-topic summary (count, rate, largest gap, size), saved to `meta/summary.csv`
- reading checks with OK / WARN per item, saved to `meta/verify.txt`. These cover body state, IMU, battery and cells, motor temperatures, lidar health, UWB, video stream structure and Jetson load.

`hardware.csv` columns: CPU load (total and per core), CPU frequency, load average, RAM and swap, GPU load, temperatures (cpu, gpu, soc0–2, tj), power rails (VDD_IN, VDD_CPU_GPU_CV, VDD_SOC), fan, disk free and read/write rate, network rx/tx for `enP8p1s0` and Wi-Fi, and CPU and memory of the recorder process.

## Exporting a session (`record/export.py`)

Turns a recorded session into tables that pandas, DuckDB or a spreadsheet can open, one file per topic, plus the two payloads that do not belong in a table.

```bash
python3 record/export.py logs/<session>                     # Parquet if pyarrow is installed, else CSV
python3 record/export.py logs/<session> --list              # topics and message counts, export nothing
python3 record/export.py logs/<session> --format csv --clouds --video mp4
python3 record/export.py logs/<session> --topics '^/(sportmodestate|uwbstate)$'
```

Output goes to `logs/<session>/export/` (`--out` to change it):

```
export/
  sportmodestate.parquet        one file per topic, `/` becomes `__`
  lf__lowstate.parquet
  uwbstate.parquet
  utlidar__cloud_base.parquet   cloud metadata; the points go to clouds/
  annotations.parquet
  video/  frontvideostream_720p.h264        recorded bytes, untouched
          frontvideostream_720p_index.*     frame index: time, offset, length, keyframe
          frontvideostream_360p.h264
  clouds/<topic>/<nnnnnn>_<t_ns>.npz        only with --clouds
  export_report.json            per topic: rows, columns, decoder used, decode failures
```

Every table starts with `t_ns` (the Jetson receive timestamp), `t_rel` (seconds from the start of the session) and `t_iso`. Because all topics share the receive clock, joining two tables on `t_ns` needs no correction.

Message fields are flattened by rule rather than by a list of wanted fields per topic, so a topic that gains a field still exports it: nested messages become `imu_state.gyroscope.0`, arrays up to `--max-array` (32) become indexed columns, longer ones and byte blobs become a `.len` column. Motor arrays are dropped by default (`--no-default-skip` keeps them), since 20 motors × 7 fields are most of `/lf/lowstate`'s columns and are not what the 20 Hz topic is kept for.

### How it decodes without depending on ROS

The bag is read straight out of its sqlite3 files with the standard library, and blobs are decoded one of two ways (`--decoder`):

- **`ros`** — the installed typesupport, authoritative when the types are built.
- **`msgdef`** — the `.msg` files are parsed and the CDR laid out from them. Needs no ROS, so a session can be exported off the Jetson.
- **`auto`** (default) — `ros` when it is importable, otherwise `msgdef`.

`msgdef` reads the definitions from `~/unitree_ros2` and `/opt/ros/humble/share` (`--msg-path` to add more), never from field orders written into the exporter: a wrong field order would give plausible but wrong numbers. Because the recorder subscribes to everything, a type that fails to decode is counted in `export_report.json` and the rest of the session still exports.

### Video

The local `Go2FrontVideoData` definition does not match what the robot sends (decoding fails with "invalid data size"), but the raw bytes are recorded intact, so this topic is parsed from the bytes instead of through its message type. Layout of each message: 4-byte CDR header, `uint64` time_frame, `uint32` resolution (720 or 360), `uint32` length, then that many bytes of H.264 Annex-B.

The two resolutions are interleaved on the one topic and are split into a stream each, at about 14 Hz with a keyframe roughly every 2 s. Frames before a stream's first keyframe cannot be decoded, so they are dropped and counted (`dropped_before_keyframe` in the report) — the 5 lost frames per stream in the 2026-10-23 test were these.

`--video raw` (default) writes the recorded bytes as `.h264`, which is lossless and what any decoder should be pointed at. `--video mp4` additionally decodes and re-encodes with OpenCV, which is lossy; ffmpeg is not installed on the Jetson, but OpenCV carries its own.

### Dependencies

The default run needs only the standard library. pyarrow enables Parquet (CSV is written instead when it is missing), numpy enables `--clouds`, and OpenCV enables `--video mp4`; each is reported as unavailable rather than failing the export.

### Tests

```bash
python3 record/tests/test_export.py
```

The tests build synthetic rosbag2 sqlite3 bags with the same schema and CDR encoding as a real session, so they need no robot, no ROS and no recording. They cover CDR alignment and both endiannesses, `.msg` parsing, round-tripping each recorded message type, the flattening rules, split sessions, the video layout and keyframe handling, and point cloud conversion. The `.msg` files under `record/tests/msgs/` are fixtures that mirror the real definitions; they are not authoritative, and the exporter does not use them at run time.

What the tests cannot cover is the one thing that needs the robot: whether the local `.msg` definitions agree with what it actually sends. Run a real session through `--list` and then `export.py`, and check `export_report.json` for decode failures.

## Viewing a session in Rerun (`record/view.py`)

Serves a recorded session to a Rerun viewer, which can be on another machine. Needs `pip install rerun-sdk` (tested against 0.40). Reading is done by `go2export`, so no ROS is needed and this runs wherever the logs are mounted.

```bash
python3 record/view.py                                  # pick a session, pick telemetry
python3 record/view.py walk_test_20261023_143002        # by name, under the logs root
python3 record/view.py <session> --telemetry pose,uwb,video
python3 record/view.py <session> --serve web            # remote client just opens a browser
python3 record/view.py <session> --list-telemetry       # what the session has, then stop
```

With no session argument it lists what is on the drive, newest first, with size, profile and note, and asks. With no `--telemetry` it lists the telemetry the session actually contains and asks. `--yes` takes the defaults (newest session, all telemetry) for running unattended.

The logs root defaults to `/media/nguyen/project_data/go2/logs`; `--logs-root` or `$GO2_LOGS` override it.

### Connecting from another machine

The server holds the recording in memory and waits. It listens on every interface, but Rerun reports `127.0.0.1`, so the printed URL is rewritten with this machine's LAN address (`--advertise-host` to set it by hand, e.g. a hostname or a tunnel endpoint).

| `--serve` | What happens |
|---|---|
| `grpc` (default) | Waits for a viewer. Connect with `rerun --connect rerun+http://<host>:9876/proxy`, or paste that into the viewer's "Add connection". |
| `web` | Also hosts the viewer itself, so a remote client only needs a browser at `http://<host>:9090`. |
| `spawn` | Opens a viewer on this machine and exits once the data is sent. |
| `save` | Writes an `.rrd` file instead of serving, to hand around or open later. |

`--memory-limit` (default 2 GiB) is how much the server keeps before dropping the oldest data; a long session with point clouds will need more, or `--cloud-stride`.

### What gets shown

| Group | Topics | In the viewer |
|---|---|---|
| `pose` | `/sportmodestate` | Robot transform and body box, a trail, velocity / yaw rate / gait plots |
| `imu` | `/lf/lowstate`, `/utlidar/imu` | Gyro, accelerometer, roll-pitch-yaw |
| `feet` | `/lf/lowstate`, `/sportmodestate` | Foot forces, and the four feet in the robot's frame |
| `battery` | `/lf/lowstate`, `/lf/battery_alarm` | State of charge, current, per-cell voltages |
| `uwb` | `/uwbstate` | The remote as a point and a ray from the robot, plus every angle field |
| `odom` | `/utlidar/robot_odom`, `/utlidar/robot_pose`, SLAM odometry | Pose and twist per source |
| `clouds` | `/utlidar/cloud*` | Point clouds, coloured by intensity |
| `video` | `/frontvideostream` | The 720p and 360p H.264 streams, on the same timeline |
| `events` | `/annotations`, `/api/*`, `/multiplestate` | A text log, so `mark.sh` labels appear against the data |
| `jetson` | `meta/hardware.csv` | Jetson load, so a dropout can be put next to what the Jetson was doing |

Groups are what a person picks from, not one per topic, so `/lf/lowstate` feeds imu, feet and battery. A topic several selected groups want is read and decoded once.

Two timelines are set on everything: `receive` (the Jetson's wall clock) and `session` (seconds from the start). Because all topics share the receive clock, they line up without correction — including the video, so a frame can be read against the pose at that instant.

The front camera is parsed from the bytes, not through `Go2FrontVideoData`, and logged as a `VideoStream` so Rerun decodes it; the two resolutions become separate streams and frames before the first keyframe are dropped.

The UWB mapping takes the same parameters as `uwb_rviz/` (`--bearing-field`, `--bearing-sign`, `--bearing-offset`, `--elevation-field`), so whatever the calibration session settles on transfers between the two tools:

```bash
python3 record/view.py <session> --telemetry uwb \
  --bearing-field yaw_est --bearing-sign -1.0 --elevation-field none
```

Since every angle field is plotted alongside the derived bearing, a recorded calibration session can be read off the plots directly rather than guessed at live.

### Options worth knowing

| Option | Default | Meaning |
|---|---|---|
| `--start`, `--duration` | `0`, `0` | Only a window of the session, in seconds from its start |
| `--cloud-stride` | `1` | Keep every Nth cloud frame |
| `--max-cloud-points` | `0` (all) | Subsample clouds larger than this |
| `--trail-length`, `--trail-every` | `600`, `25` | Length of the pose trail, and how often it is redrawn |
| `--quaternion-order` | `wxyz` | Unitree sends `w,x,y,z`; Rerun wants `x,y,z,w` |
| `--decoder`, `--msg-path` | `auto` | As for `export.py` |

### Tests

```bash
python3 record/tests/test_view.py
```

The handlers are tested by standing in for the `rerun` module and recording what they log, so the assertions are about entity paths and geometry rather than Rerun's own behaviour. The UWB mapping and the quaternion reordering are pure functions and are tested directly. Also covered: session discovery and selection, the telemetry menu, per-topic dispatch (including that a shared topic is decoded once and that the video topic is never decoded through a message type), the time window, `hardware.csv` parsing, and that one failing handler does not stop the rest.

Not covered by the tests, because it needs a viewer: the picture itself. Both serve modes were checked by hand — the gRPC port accepts connections on the non-loopback address, and `--serve web` returns the viewer page — but whether the robot looks right on screen has to be judged with a real session.

## Implementation plan

Goal: record the Go2's telemetry into `logs/`, in a form that is easy to analyse later. Steps 1–5 and 8 below are built and tested; step 7 is still optional work.

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
5. **`record/export.py <session>`**: flattens a session into one Parquet or CSV file per topic, timestamped by when the Jetson received each message. See [Exporting a session](#exporting-a-session-recordexportpy) below.
6. **Time alignment**: all topics share the Jetson's receive clock, so they line up without extra work.
7. **Optional, later**: get the missing `.msg` files from a newer `unitree_ros2` release, rebuild `cyclonedds_ws`, and drop `/utlidar/voxel_map_compressed` and `/config_change_status` from the base exclude list. The `unitree_interfaces` and `unitree_arm` types are probably not public and are not needed for analysis.
8. **`record/view.py <session>`**: serves a session to a Rerun viewer, which can be on another machine, with the session and the telemetry chosen interactively. See [Viewing a session in Rerun](#viewing-a-session-in-rerun-recordviewpy).

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
