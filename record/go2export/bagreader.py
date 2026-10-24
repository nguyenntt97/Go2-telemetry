"""Read a rosbag2 sqlite3 recording without ROS.

`go2_record.sh` splits every 5 minutes, so a session holds several `.db3`
files. Topic ids are per file, so each file is opened on its own and the
topics are matched up by name.
"""

from __future__ import annotations

import glob
import os
import sqlite3
from contextlib import closing
from dataclasses import dataclass, field


@dataclass
class TopicInfo:
    name: str
    type: str
    count: int = 0
    serialization_format: str = ""
    files: list = field(default_factory=list)


class BagError(RuntimeError):
    pass


def find_bag_dir(session: str) -> str:
    """Accept either a session directory or the bag directory inside it."""
    session = os.path.abspath(os.path.expanduser(session))
    if not os.path.isdir(session):
        raise BagError(f"not a directory: {session}")
    candidates = [os.path.join(session, "go2"), session]
    for candidate in candidates:
        if os.path.isdir(candidate) and glob.glob(os.path.join(candidate, "*.db3")):
            return candidate
    if glob.glob(os.path.join(session, "**", "*.mcap"), recursive=True):
        raise BagError(
            f"{session} holds an mcap bag; this exporter reads the sqlite3 bags "
            "that go2_record.sh writes"
        )
    raise BagError(f"no .db3 files under {session}")


class BagReader:
    def __init__(self, bag_dir: str) -> None:
        self.bag_dir = bag_dir
        self.files = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
        if not self.files:
            raise BagError(f"no .db3 files in {bag_dir}")
        self._topics: dict[str, TopicInfo] = {}
        self._scan()

    def _connect(self, path: str) -> sqlite3.Connection:
        return sqlite3.connect(f"file:{path}?immutable=1", uri=True)

    def _scan(self) -> None:
        for path in self.files:
            with closing(self._connect(path)) as conn:
                rows = conn.execute(
                    "SELECT t.name, t.type, t.serialization_format, COUNT(m.id) "
                    "FROM topics t LEFT JOIN messages m ON m.topic_id = t.id "
                    "GROUP BY t.id"
                ).fetchall()
            for name, type_name, fmt, count in rows:
                info = self._topics.get(name)
                if info is None:
                    info = self._topics[name] = TopicInfo(
                        name=name, type=type_name, serialization_format=fmt or ""
                    )
                info.count += count
                if count:
                    info.files.append(path)

    @property
    def topics(self) -> dict[str, TopicInfo]:
        return self._topics

    def active_topics(self) -> dict[str, TopicInfo]:
        """Only the topics that actually carry messages.

        The recorder subscribes to everything, so most of the ~92 topics in a
        session are empty.
        """
        return {name: info for name, info in self._topics.items() if info.count}

    def start_time(self) -> int | None:
        """Earliest receive timestamp in the session, in nanoseconds."""
        best = None
        for path in self.files:
            with closing(self._connect(path)) as conn:
                row = conn.execute("SELECT MIN(timestamp) FROM messages").fetchone()
            if row and row[0] is not None:
                best = row[0] if best is None else min(best, row[0])
        return best

    def messages(self, topic: str):
        """Yield ``(timestamp_ns, blob)`` for one topic, in time order.

        Each split file is read in turn; within a session the files are
        already ordered in time, so no global sort is needed.
        """
        info = self._topics.get(topic)
        if info is None:
            raise BagError(f"no topic {topic} in {self.bag_dir}")
        for path in info.files or self.files:
            with closing(self._connect(path)) as conn:
                cursor = conn.execute(
                    "SELECT m.timestamp, m.data FROM messages m "
                    "JOIN topics t ON t.id = m.topic_id "
                    "WHERE t.name = ? ORDER BY m.timestamp",
                    (topic,),
                )
                for timestamp, blob in cursor:
                    yield timestamp, bytes(blob)
