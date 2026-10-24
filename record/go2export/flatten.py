"""Turn a decoded message into one flat row of columns.

Flattening everything that is a scalar, rather than listing wanted fields per
topic, means a topic that gains a field still exports it. The parts that would
be useless as columns are handled by rule: motor arrays are skipped by
default, byte blobs become a length, and anything longer than `max_array`
becomes a length too.
"""

from __future__ import annotations

import json
import re

#: Dropped by default. `/lf/lowstate` carries 20 motors x ~10 fields, which is
#: the bulk of its columns and not what the low rate topic is kept for.
DEFAULT_SKIP = (
    r"(^|\.)motor_state(\.|$)",
    r"(^|\.)motor_cmd(\.|$)",
)


def compile_skips(patterns) -> list:
    return [re.compile(p) for p in patterns]


def _is_scalar(value) -> bool:
    return isinstance(value, (int, float, bool, str)) or value is None


def _maybe_json(value):
    """Parse a string field that holds JSON, as `/api/*` and the battery
    alarm topic do. Returns None when it is not a JSON object."""
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text.startswith("{"):
        return None
    try:
        parsed = json.loads(text)
    except (ValueError, TypeError):
        return None
    return parsed if isinstance(parsed, dict) else None


class Flattener:
    def __init__(self, max_array: int = 32, skip=None, expand_json: bool = False) -> None:
        self.max_array = max_array
        self.skips = compile_skips(DEFAULT_SKIP if skip is None else skip)
        self.expand_json = expand_json

    def _skipped(self, path: str) -> bool:
        return any(pattern.search(path) for pattern in self.skips)

    def flatten(self, message: dict) -> dict:
        row: dict = {}
        self._walk(message, "", row)
        return row

    def _put(self, path: str, value, row: dict) -> None:
        row[path] = value

    def _walk(self, value, path: str, row: dict) -> None:
        if path and self._skipped(path):
            return

        if isinstance(value, dict):
            for key, item in value.items():
                self._walk(item, f"{path}.{key}" if path else key, row)
            return

        if isinstance(value, (bytes, bytearray)):
            # Point cloud payloads and video frames live here; keep the size
            # only, the real data is exported by the dedicated writers.
            self._put(f"{path}.len" if path else "len", len(value), row)
            return

        if isinstance(value, (list, tuple)):
            items = list(value)
            if len(items) > self.max_array:
                self._put(f"{path}.len", len(items), row)
                return
            for index, item in enumerate(items):
                self._walk(item, f"{path}.{index}", row)
            return

        if _is_scalar(value):
            if self.expand_json:
                parsed = _maybe_json(value)
                if parsed is not None:
                    self._put(path, value, row)
                    for key, item in parsed.items():
                        self._walk(item, f"{path}.{key}", row)
                    return
            self._put(path, value, row)
            return

        # Anything left (an unexpected object) goes in as text rather than
        # failing the whole topic.
        self._put(path, repr(value), row)
