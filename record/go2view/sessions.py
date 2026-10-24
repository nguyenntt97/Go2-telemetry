"""Find recorded sessions and let the user pick one.

Sessions are the `<name>_<YYYYmmdd_HHMMSS>` directories the recorder makes.
The default root is the project drive; `--logs-root` or `$GO2_LOGS` override it.
"""

from __future__ import annotations

import datetime
import glob
import os
import re
from dataclasses import dataclass

DEFAULT_LOGS_ROOT = "/media/nguyen/project_data/go2/logs"

#: The recorder's `<name>_<date>_<time>` suffix.
_STAMP_RE = re.compile(r"^(?P<name>.+)_(?P<stamp>\d{8}_\d{6})$")


class SessionError(RuntimeError):
    pass


def default_logs_root() -> str:
    return os.environ.get("GO2_LOGS") or DEFAULT_LOGS_ROOT


@dataclass
class SessionInfo:
    path: str
    name: str
    started: datetime.datetime | None
    size_bytes: int
    profile: str = ""
    note: str = ""

    @property
    def label(self) -> str:
        when = self.started.strftime("%Y-%m-%d %H:%M") if self.started else "unknown time"
        parts = [f"{human_size(self.size_bytes):>8}", when]
        if self.profile:
            parts.append(self.profile)
        if self.note:
            parts.append(f'"{self.note}"')
        return "  ".join(parts)


def human_size(size: float) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if size < 1024 or unit == "TB":
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} TB"


def _read_session_yaml(path: str) -> dict:
    """Pull a few keys out of `meta/session.yaml`.

    PyYAML is used when available; the fallback handles the flat
    `key: value` file the recorder writes.
    """
    if not os.path.isfile(path):
        return {}
    try:
        import yaml

        with open(path, encoding="utf-8", errors="replace") as handle:
            loaded = yaml.safe_load(handle)
        if isinstance(loaded, dict):
            return {str(k): v for k, v in loaded.items()}
    except Exception:  # noqa: BLE001 - fall through to the flat parser
        pass

    out: dict = {}
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            for line in handle:
                if line.startswith((" ", "\t", "#")) or ":" not in line:
                    continue
                key, _, value = line.partition(":")
                out[key.strip()] = value.strip().strip("\"'")
    except OSError:
        return {}
    return out


def _session_size(path: str) -> int:
    # Only the bag files, so this stays cheap on a session holding hours of
    # point clouds.
    total = 0
    for pattern in ("go2/*.db3", "*.db3"):
        for found in glob.glob(os.path.join(path, pattern)):
            try:
                total += os.path.getsize(found)
            except OSError:
                pass
        if total:
            break
    return total


def describe_session(path: str) -> SessionInfo:
    path = os.path.abspath(os.path.expanduser(path))
    base = os.path.basename(path.rstrip(os.sep))
    match = _STAMP_RE.match(base)
    started = None
    if match:
        try:
            started = datetime.datetime.strptime(match.group("stamp"), "%Y%m%d_%H%M%S")
        except ValueError:
            started = None
    if started is None:
        try:
            started = datetime.datetime.fromtimestamp(os.path.getmtime(path))
        except OSError:
            started = None
    meta = _read_session_yaml(os.path.join(path, "meta", "session.yaml"))
    return SessionInfo(
        path=path,
        name=base,
        started=started,
        size_bytes=_session_size(path),
        profile=str(meta.get("profile", "") or ""),
        note=str(meta.get("note", "") or ""),
    )


def find_sessions(logs_root: str) -> list:
    """Every session under `logs_root`, newest first."""
    root = os.path.abspath(os.path.expanduser(logs_root))
    if not os.path.isdir(root):
        raise SessionError(f"no logs directory at {root}")
    sessions = []
    for entry in sorted(os.listdir(root)):
        path = os.path.join(root, entry)
        if not os.path.isdir(path):
            continue
        if not glob.glob(os.path.join(path, "go2", "*.db3")) and not glob.glob(
            os.path.join(path, "*.db3")
        ):
            continue
        sessions.append(describe_session(path))
    sessions.sort(key=lambda s: (s.started or datetime.datetime.min), reverse=True)
    return sessions


def resolve_session(argument: str | None, logs_root: str, interactive: bool = True) -> SessionInfo:
    """Turn a command line argument into one session.

    A path is taken as given. A bare name is looked up under `logs_root`.
    Nothing at all means ask, when there is a terminal to ask on.
    """
    if argument:
        candidate = os.path.abspath(os.path.expanduser(argument))
        if os.path.isdir(candidate):
            return describe_session(candidate)
        under_root = os.path.join(os.path.expanduser(logs_root), argument)
        if os.path.isdir(under_root):
            return describe_session(under_root)
        matches = [s for s in find_sessions(logs_root) if s.name.startswith(argument)]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            names = ", ".join(s.name for s in matches[:5])
            raise SessionError(f"{argument!r} matches more than one session: {names}")
        raise SessionError(f"no session {argument!r} in {logs_root}")

    sessions = find_sessions(logs_root)
    if not sessions:
        raise SessionError(f"no sessions in {logs_root}")
    if not interactive:
        return sessions[0]
    return choose_session(sessions)


def choose_session(sessions: list) -> SessionInfo:
    print(f"Sessions ({len(sessions)}, newest first):\n")
    width = max(len(s.name) for s in sessions)
    for index, session in enumerate(sessions, 1):
        print(f"  {index:>2}  {session.name:<{width}}  {session.label}")
    print()
    while True:
        try:
            answer = input(f"Session [1-{len(sessions)}, default 1]: ").strip()
        except EOFError:
            answer = ""
        if not answer:
            return sessions[0]
        if answer.isdigit() and 1 <= int(answer) <= len(sessions):
            return sessions[int(answer) - 1]
        named = [s for s in sessions if s.name == answer or s.name.startswith(answer)]
        if len(named) == 1:
            return named[0]
        print("  not a choice; type a number from the list")
