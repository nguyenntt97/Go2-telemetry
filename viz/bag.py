#!/usr/bin/env python3
"""Read a recording session straight from the rosbag2 SQLite files.

No ROS on this machine, so the bag is read as what it is: a `messages` table of
(timestamp, topic_id, blob) and a `topics` table naming the types. Message bodies are
handed to `cdr` for decoding.
"""
import glob
import os
import sqlite3

import numpy as np
import yaml

import cdr
import msgs


def sessions(logs_dir):
    """Recording sessions under `logs_dir`, newest first."""
    out = []
    for path in sorted(glob.glob(os.path.join(logs_dir, "*"))):
        if glob.glob(os.path.join(path, "go2", "*.db3")):
            out.append(Session(path))
    out.sort(key=lambda s: s.name, reverse=True)
    return out


class Session:
    """A session directory: the bag plus the metadata the recorder wrote."""

    def __init__(self, path):
        self.path = path.rstrip("/")
        self.name = os.path.basename(self.path)
        self.meta = self._yaml(os.path.join(self.path, "meta", "session.yaml"))
        info = self._yaml(os.path.join(self.path, "go2", "metadata.yaml")) or {}
        info = info.get("rosbag2_bagfile_information", {})
        # Counts come from metadata.yaml, so listing topics costs nothing; counting
        # rows in the bag would mean scanning several GB.
        self.counts = {t["topic_metadata"]["name"]: t["message_count"]
                       for t in info.get("topics_with_message_count", [])}
        self.types = {t["topic_metadata"]["name"]: t["topic_metadata"]["type"]
                      for t in info.get("topics_with_message_count", [])}
        self.duration = info.get("duration", {}).get("nanoseconds", 0) * 1e-9
        self.start_ns = info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)
        self.total = info.get("message_count", 0)

    @staticmethod
    def _yaml(path):
        try:
            with open(path) as f:
                return yaml.safe_load(f)
        except (OSError, yaml.YAMLError):
            return None

    @property
    def files(self):
        files = glob.glob(os.path.join(self.path, "go2", "*.db3"))
        return sorted(files, key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))

    @property
    def profile(self):
        return (self.meta or {}).get("profile", "?")

    def active(self, minimum=1):
        """Topics that actually carry data."""
        return {t: n for t, n in self.counts.items() if n >= minimum}

    # --- reading --------------------------------------------------------------

    def blobs(self, topic, every=1, limit=None):
        """(receive times in s since session start, list of message bytes).

        `every` keeps one message in n, skipping while streaming from SQLite so the
        blobs that are thrown away are never all held in memory at once.
        """
        times, out = [], []
        for path in self.files:
            db = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
            try:
                rows = db.execute(
                    "select m.timestamp, m.data from messages m "
                    "join topics t on t.id = m.topic_id where t.name = ? "
                    "order by m.timestamp", (topic,))
                for i, (ts, data) in enumerate(rows):
                    if i % every:
                        continue
                    times.append(ts)
                    out.append(data)
                    if limit and len(out) >= limit:
                        return self._rel(times), out
            finally:
                db.close()
        return self._rel(times), out

    def _rel(self, times):
        t = np.asarray(times, dtype=np.int64)
        return (t - self.start_ns) * 1e-9 if len(t) else np.zeros(0)

    def decode(self, topic, every=1, limit=None):
        """(times, structured array) for a topic with a known fixed layout."""
        schema = msgs.MESSAGES.get(self.types.get(topic))
        if schema is None:
            raise KeyError(f"no schema for {topic} ({self.types.get(topic)})")
        t, blobs = self.blobs(topic, every, limit)
        if not blobs:
            return t, None
        return t, cdr.decode_fixed(blobs, schema)

    def strings(self, topic, every=1, limit=None):
        """(times, list of str) for std_msgs/String topics."""
        t, blobs = self.blobs(topic, every, limit)
        return t, [cdr.Reader(b).string() for b in blobs]

    def clouds(self, topic, every=1, limit=None):
        """(times, list of (N,3) float32 xyz) for sensor_msgs/PointCloud2."""
        t, blobs = self.blobs(topic, every, limit)
        return t, [_cloud_xyz(b) for b in blobs]


def _cloud_xyz(payload):
    """xyz of a PointCloud2. Point clouds vary in length, so this is read field by
    field rather than through a fixed dtype."""
    r = cdr.Reader(payload)
    r.header()
    height, width = r.prim("u32"), r.prim("u32")
    fields = {}
    for _ in range(r.prim("u32")):
        name = r.string()
        fields[name] = (r.prim("u32"), r.prim("u8"), r.prim("u32"))   # offset, dtype, count
    r.prim("bool")                       # is_bigendian
    point_step, _row_step = r.prim("u32"), r.prim("u32")
    n = r.prim("u32")                    # length of the data sequence, in bytes
    raw = np.frombuffer(r.buf, dtype=np.uint8, count=n, offset=r.off)
    npts = height * width
    if not npts or point_step < 12:
        return np.zeros((0, 3), np.float32)
    pts = raw[:npts * point_step].reshape(npts, point_step)
    off = fields.get("x", (0,))[0]
    return pts[:, off:off + 12].copy().view(np.float32).reshape(-1, 3)
