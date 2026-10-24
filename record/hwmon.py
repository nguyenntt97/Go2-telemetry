#!/usr/bin/env python3
"""Log Jetson hardware stats to CSV once per second until stopped (Ctrl+C / SIGTERM).

Usage: python3 hwmon.py <out.csv> [--pid PID] [--interval 1.0]
  --pid  process to track (CPU %, memory), e.g. the ros2 bag recorder; children included
Columns: CPU load per core and frequency, load average, RAM/swap, GPU load, temperatures,
power rails (INA3221), fan, disk free and I/O, network throughput, tracked process usage.
"""
import argparse
import csv
import glob
import os
import shutil
import signal
import time

import psutil

GPU_LOAD = "/sys/devices/platform/bus@0/17000000.gpu/load"   # 0-1000
DISK = "nvme0n1"
NET_IFACES = ("enP8p1s0", "wlx20e15d9b3135")   # robot DDS network, Wi-Fi


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except (OSError, TypeError):   # unavailable thermal zones (cv0-2) fail with EAGAIN
        return default


def thermal_zones():
    zones = {}
    for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        name = read(f"{z}/type").replace("-thermal", "")
        if read(f"{z}/temp"):
            zones[name] = f"{z}/temp"
    return zones


def power_rails():
    """INA3221 rails as {label: (mV path, mA path)}."""
    rails = {}
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        if read(f"{h}/name") != "ina3221":
            continue
        for lab in sorted(glob.glob(f"{h}/in*_label")):
            n = os.path.basename(lab)[2:].split("_")[0]
            if read(lab).startswith("VDD"):   # skip the "sum of shunt voltages" channel
                rails[read(lab)] = (f"{h}/in{n}_input", f"{h}/curr{n}_input")
    return rails


def hwmon_path(name, file):
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        if read(f"{h}/name") == name and os.path.exists(f"{h}/{file}"):
            return f"{h}/{file}"
    return None


def proc_usage(proc):
    """CPU % (all cores, 100 = one core) and RSS MB of proc and its children."""
    try:
        procs = [proc] + proc.children(recursive=True)
    except psutil.NoSuchProcess:
        return "", ""
    cpu = rss = 0.0
    for p in procs:
        try:
            cpu += p.cpu_percent(None)
            rss += p.memory_info().rss
        except psutil.NoSuchProcess:
            pass
    return round(cpu, 1), round(rss / 2**20, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--pid", type=int)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--disk-path", default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()

    zones, rails = thermal_zones(), power_rails()
    fan_pwm, fan_rpm = hwmon_path("pwmfan", "pwm1"), hwmon_path("pwm_tach", "rpm")
    ncpu = psutil.cpu_count()
    proc = psutil.Process(a.pid) if a.pid else None

    cols = (["time", "cpu_pct"] + [f"cpu{i}_pct" for i in range(ncpu)] +
            ["cpu_mhz", "load1", "ram_used_mb", "ram_avail_mb", "swap_used_mb", "gpu_pct"] +
            [f"temp_{z}_c" for z in zones] + [f"power_{r}_mw" for r in rails] +
            ["fan_pwm", "fan_rpm", "disk_free_gb", "disk_read_mb_s", "disk_write_mb_s"] +
            [f"{i}_{d}_mb_s" for i in NET_IFACES for d in ("rx", "tx")] +
            ["rec_cpu_pct", "rec_rss_mb"])

    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))

    # Prime the counters that report rates since the previous call
    psutil.cpu_percent(percpu=True)
    if proc:
        proc_usage(proc)
    disk0 = psutil.disk_io_counters(perdisk=True).get(DISK)
    net0 = psutil.net_io_counters(pernic=True)
    t0 = time.monotonic()

    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        while not stop:
            time.sleep(a.interval)
            t1 = time.monotonic()
            dt = t1 - t0
            per = psutil.cpu_percent(percpu=True)
            mem, swap = psutil.virtual_memory(), psutil.swap_memory()
            disk1 = psutil.disk_io_counters(perdisk=True).get(DISK)
            net1 = psutil.net_io_counters(pernic=True)

            row = [f"{time.time():.3f}", round(sum(per) / len(per), 1), *per,
                   round(psutil.cpu_freq().current), round(os.getloadavg()[0], 2),
                   (mem.total - mem.available) >> 20, mem.available >> 20, swap.used >> 20,
                   int(read(GPU_LOAD, "0")) / 10]
            row += [int(read(p, "0")) / 1000 for p in zones.values()]
            row += [round(int(read(v, "0")) * int(read(i, "0")) / 1000) for v, i in rails.values()]
            row += [read(fan_pwm) if fan_pwm else "", read(fan_rpm) if fan_rpm else ""]
            row.append(round(shutil.disk_usage(a.disk_path).free / 2**30, 1))
            if disk0 and disk1:
                row += [round((disk1.read_bytes - disk0.read_bytes) / dt / 2**20, 2),
                        round((disk1.write_bytes - disk0.write_bytes) / dt / 2**20, 2)]
            else:
                row += ["", ""]
            for i in NET_IFACES:
                if i in net0 and i in net1:
                    row += [round((net1[i].bytes_recv - net0[i].bytes_recv) / dt / 2**20, 3),
                            round((net1[i].bytes_sent - net0[i].bytes_sent) / dt / 2**20, 3)]
                else:
                    row += ["", ""]
            row += list(proc_usage(proc)) if proc else ["", ""]
            w.writerow(row)
            f.flush()
            t0, disk0, net0 = t1, disk1, net1


if __name__ == "__main__":
    main()
