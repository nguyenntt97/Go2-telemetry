"""Plain CDR (XCDR1) reading and writing.

Every ROS 2 message type in Humble is a `final` type, so what a rosbag2 blob
holds is plain CDR: a four byte encapsulation header followed by the members,
each aligned on its own size relative to the start of the payload. There are
no DHEADERs to skip, which is what makes decoding without ROS practical.
"""

from __future__ import annotations

import struct

ENCAPSULATION_LE = b"\x00\x01\x00\x00"
ENCAPSULATION_BE = b"\x00\x00\x00\x00"

# primitive name -> (struct code, size in bytes, which is also its alignment)
PRIMITIVE_FORMATS = {
    "bool": ("?", 1),
    "byte": ("B", 1),
    "char": ("B", 1),
    "int8": ("b", 1),
    "uint8": ("B", 1),
    "int16": ("h", 2),
    "uint16": ("H", 2),
    "int32": ("i", 4),
    "uint32": ("I", 4),
    "int64": ("q", 8),
    "uint64": ("Q", 8),
    "float32": ("f", 4),
    "float64": ("d", 8),
}

# One byte wide, so a sequence of them can be taken as a single block.
BYTE_TYPES = frozenset({"byte", "char", "int8", "uint8"})


class CdrError(ValueError):
    """A blob is too short, or not valid CDR."""


class CdrReader:
    """Sequential reader over one serialized message."""

    def __init__(self, data: bytes, little_endian: bool | None = None) -> None:
        if len(data) < 4:
            raise CdrError(f"{len(data)} bytes is too short for a CDR header")
        self._data = data
        self._origin = 4  # alignment is relative to the payload, not the blob
        self._pos = 4
        if little_endian is None:
            # Encapsulation kind: 0x0000 is big endian, 0x0001 little endian.
            little_endian = bool(data[1] & 0x01)
        self.little_endian = little_endian
        self._prefix = "<" if little_endian else ">"

    @property
    def remaining(self) -> int:
        return len(self._data) - self._pos

    def tell(self) -> int:
        """Offset into the payload, i.e. excluding the encapsulation header."""
        return self._pos - self._origin

    def align(self, size: int) -> None:
        if size > 1:
            self._pos += (self._origin - self._pos) % size

    def _take(self, count: int) -> bytes:
        end = self._pos + count
        if count < 0 or end > len(self._data):
            raise CdrError(
                f"want {count} bytes at offset {self._pos}, "
                f"only {self.remaining} left"
            )
        chunk = self._data[self._pos : end]
        self._pos = end
        return chunk

    def primitive(self, kind: str):
        try:
            code, size = PRIMITIVE_FORMATS[kind]
        except KeyError:
            raise CdrError(f"not a CDR primitive: {kind}") from None
        self.align(size)
        return struct.unpack(self._prefix + code, self._take(size))[0]

    def uint32(self) -> int:
        return self.primitive("uint32")

    def block(self, count: int) -> bytes:
        """Read `count` bytes with no alignment, for one byte wide elements."""
        return self._take(count)

    def string(self) -> str:
        # The serialized length includes the terminating nul, which the data
        # also carries.
        raw = self._take(self.uint32())
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")

    def array(self, kind: str, count: int) -> list:
        code, size = PRIMITIVE_FORMATS[kind]
        if count == 0:
            return []
        self.align(size)
        return list(
            struct.unpack(self._prefix + code * count, self._take(size * count))
        )


class CdrWriter:
    """Mirror of :class:`CdrReader`, used to build fixtures for the tests."""

    def __init__(self, little_endian: bool = True) -> None:
        self.little_endian = little_endian
        self._prefix = "<" if little_endian else ">"
        self._buf = bytearray(ENCAPSULATION_LE if little_endian else ENCAPSULATION_BE)
        self._origin = 4

    def getvalue(self) -> bytes:
        return bytes(self._buf)

    def align(self, size: int) -> None:
        if size > 1:
            self._buf.extend(b"\x00" * ((self._origin - len(self._buf)) % size))

    def primitive(self, kind: str, value) -> None:
        code, size = PRIMITIVE_FORMATS[kind]
        self.align(size)
        if kind == "bool":
            value = bool(value)
        elif code in "bBhHiIqQ":
            value = int(value)
        else:
            value = float(value)
        self._buf.extend(struct.pack(self._prefix + code, value))

    def uint32(self, value: int) -> None:
        self.primitive("uint32", value)

    def block(self, data: bytes) -> None:
        self._buf.extend(data)

    def string(self, value: str) -> None:
        raw = value.encode("utf-8") + b"\x00"
        self.uint32(len(raw))
        self._buf.extend(raw)

    def array(self, kind: str, values) -> None:
        values = list(values)
        if not values:
            return
        code, size = PRIMITIVE_FORMATS[kind]
        self.align(size)
        if kind == "bool":
            values = [bool(v) for v in values]
        elif code in "bBhHiIqQ":
            values = [int(v) for v in values]
        else:
            values = [float(v) for v in values]
        self._buf.extend(struct.pack(self._prefix + code * len(values), *values))
