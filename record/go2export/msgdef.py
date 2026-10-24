"""Read `.msg` files and turn them into field layouts.

The decoder is driven by the definitions in the tree the recording was made
against (`~/unitree_ros2` on the Jetson), rather than by field orders written
out here. Getting a field order wrong would produce plausible looking but
wrong numbers, so the layout always comes from the same `.msg` files the
robot's types were built from.

A few `builtin_interfaces` and `std_msgs` types are embedded as a fallback,
because they are small, fixed, and may not be on the search path.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass

from .cdr import PRIMITIVE_FORMATS

PRIMITIVES = frozenset(PRIMITIVE_FORMATS) | {"string", "wstring"}

DEFAULT_SEARCH_PATHS = (
    "~/unitree_ros2",
    "/opt/ros/humble/share",
)

_BUILTIN_DEFS = {
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec\n",
    "builtin_interfaces/Duration": "int32 sec\nuint32 nanosec\n",
    "std_msgs/Header": "builtin_interfaces/Time stamp\nstring frame_id\n",
    "std_msgs/String": "string data\n",
    "std_msgs/Bool": "bool data\n",
    "std_msgs/Int32": "int32 data\n",
    "std_msgs/Float32": "float32 data\n",
    "std_msgs/Float64": "float64 data\n",
    "std_msgs/MultiArrayDimension": "string label\nuint32 size\nuint32 stride\n",
}

_FIELD_RE = re.compile(r"^(\S+)\s+(\S+)")


class MsgDefError(LookupError):
    """A type could not be found, or its definition could not be parsed."""


@dataclass(frozen=True)
class Field:
    name: str
    base: str
    #: ``None`` for a scalar, ``("fixed", n)`` for ``T[n]``,
    #: ``("seq", n_or_None)`` for ``T[]`` and ``T[<=n]``.
    array: tuple | None

    @property
    def is_primitive(self) -> bool:
        return self.base in PRIMITIVES


def normalize_type(name: str, default_package: str | None = None) -> str:
    """Return ``pkg/Type`` for any of the spellings a `.msg` file may use."""
    if name in PRIMITIVES:
        return name
    parts = name.split("/")
    if len(parts) == 3:  # pkg/msg/Type
        return f"{parts[0]}/{parts[2]}"
    if len(parts) == 2:
        return f"{parts[0]}/{parts[1]}"
    if default_package:
        return f"{default_package}/{name}"
    raise MsgDefError(f"cannot resolve bare type name {name!r} without a package")


def _strip_comment(line: str) -> str:
    out = []
    quote = None
    for char in line:
        if quote:
            out.append(char)
            if char == quote:
                quote = None
        elif char in "'\"":
            quote = char
            out.append(char)
        elif char == "#":
            break
        else:
            out.append(char)
    return "".join(out).strip()


def _parse_type_token(token: str) -> tuple[str, tuple | None]:
    array: tuple | None = None
    if token.endswith("]"):
        base, _, spec = token[:-1].rpartition("[")
        spec = spec.strip()
        if spec == "":
            array = ("seq", None)
        elif spec.startswith("<="):
            array = ("seq", int(spec[2:]))
        else:
            array = ("fixed", int(spec))
        token = base
    if "<=" in token:  # bounded string, e.g. string<=32
        token = token.split("<=", 1)[0]
    return token, array


def parse_definition(text: str, package: str | None = None) -> list[Field]:
    """Parse one `.msg` body into its fields, ignoring constants."""
    fields: list[Field] = []
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line:
            continue
        match = _FIELD_RE.match(line)
        if not match:
            continue
        type_token, name_token = match.group(1), match.group(2)
        # A constant is `TYPE NAME=value`; a field default is `TYPE name value`.
        if "=" in name_token:
            continue
        base, array = _parse_type_token(type_token)
        if base == "wstring":
            raise MsgDefError("wstring fields are not supported")
        if base not in PRIMITIVES:
            base = normalize_type(base, package)
        fields.append(Field(name=name_token, base=base, array=array))
    return fields


class MsgRegistry:
    """Finds and caches field layouts, by ``pkg/Type``."""

    def __init__(self, search_paths=None) -> None:
        self._index: dict[str, str] = {}
        self._cache: dict[str, list[Field]] = {}
        self.search_paths: list[str] = []
        for path in search_paths if search_paths is not None else DEFAULT_SEARCH_PATHS:
            self.add_search_path(path)

    def add_search_path(self, path: str) -> None:
        root = os.path.abspath(os.path.expanduser(path))
        if not os.path.isdir(root):
            return
        self.search_paths.append(root)
        # Both the source layout (<pkg>/msg/T.msg) and the install layout
        # (share/<pkg>/msg/T.msg) put definitions in a directory called `msg`
        # whose parent is the package.
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in (".git", "build", "install")]
            if os.path.basename(dirpath) != "msg":
                continue
            package = os.path.basename(os.path.dirname(dirpath))
            for filename in filenames:
                if filename.endswith(".msg"):
                    key = f"{package}/{filename[:-4]}"
                    self._index.setdefault(key, os.path.join(dirpath, filename))

    def known_types(self) -> list[str]:
        return sorted(set(self._index) | set(_BUILTIN_DEFS))

    def fields(self, type_name: str) -> list[Field]:
        key = normalize_type(type_name)
        if key in self._cache:
            return self._cache[key]
        package = key.split("/", 1)[0]
        path = self._index.get(key)
        if path is not None:
            with open(path, "r", encoding="utf-8", errors="replace") as handle:
                text = handle.read()
        elif key in _BUILTIN_DEFS:
            text = _BUILTIN_DEFS[key]
        else:
            raise MsgDefError(
                f"no definition for {key}; searched {', '.join(self.search_paths) or '(nothing)'}"
            )
        fields = parse_definition(text, package)
        self._cache[key] = fields
        return fields
