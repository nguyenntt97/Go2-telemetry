#!/usr/bin/env python3
"""Decode ROS 2 CDR messages without a ROS installation.

The Jetson has `unitree_ros2`, this machine does not, so `record/verify.py` (which
uses rclpy) cannot run here. The bags are plain SQLite, so the only missing piece is
the deserializer.

ROS 2 serializes with CDR: a 4-byte encapsulation header, then little-endian fields,
each primitive aligned to its own size counting from the end of that header. For a
message with no strings or sequences the result is byte-identical to a packed C
struct, so a whole topic can be decoded with one `np.frombuffer` instead of a Python
loop per field. Two paths:

  compile(schema, body_len, probe) -> numpy structured dtype  (fixed layout)
  Reader(body)                     -> field-by-field reader   (strings/sequences)
"""
import struct

import numpy as np

# CDR primitives: name -> (numpy format, size). Alignment equals size.
PRIM = {
    "i8": ("i1", 1), "u8": ("u1", 1), "bool": ("u1", 1), "char": ("u1", 1),
    "i16": ("i2", 2), "u16": ("u2", 2),
    "i32": ("i4", 4), "u32": ("u4", 4),
    "i64": ("i8", 8), "u64": ("u8", 8),
    "f32": ("f4", 4), "f64": ("f8", 8),
}

HEADER_LEN = 4   # encapsulation header, stripped before offsets are counted

SCHEMA_STRUCTS = {}   # nested types, filled in by msgs.py


class SchemaError(Exception):
    pass


def _align(off, a):
    return (off + a - 1) // a * a


# --- fixed-layout path: schema -> numpy dtype ---------------------------------

def _layout(fields, structs):
    """Lay out a struct's fields relative to its own start.

    Relative offsets are valid because a struct is always placed on a multiple of
    its own alignment -- the strictest of its members' -- so relative and absolute
    padding agree.

    Returns (names, formats, offsets, natural size, alignment), where the natural
    size has *no* trailing padding: CDR pads between fields, never after the last
    one, so a nested struct is followed immediately by whatever aligns next.
    """
    names, formats, offsets, off, maxal = [], [], [], 0, 1
    for name, typ in fields:
        fmt, size, al, shape = _resolve(typ, structs)
        off = _align(off, al)
        maxal = max(maxal, al)
        names.append(name)
        formats.append((fmt, shape) if shape else fmt)
        offsets.append(off)
        off += size
    return names, formats, offsets, off, maxal


def _struct(fields, structs, stride):
    """A nested struct -> (dtype, size it occupies, alignment).

    `stride=True` pads the size up to the struct's alignment, which is what an array
    element needs so that the next element's fields still land on their boundaries.
    A struct used as a single field takes its natural size instead.
    """
    names, formats, offsets, size, maxal = _layout(fields, structs)
    itemsize = _align(size, maxal) if stride else size
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": itemsize})
    return dt, itemsize, maxal


def _resolve(typ, structs):
    """A field type -> (numpy format, bytes occupied, alignment, shape)."""
    if isinstance(typ, str):
        if typ in PRIM:
            fmt, size = PRIM[typ]
            return fmt, size, size, ()
        if typ in structs:
            return _struct(structs[typ], structs, stride=False)[:3] + ((),)
        raise SchemaError(f"unknown type {typ!r}")

    kind, elem, n = typ
    if kind != "arr":
        raise SchemaError(f"{kind!r} has no fixed layout; use Reader")
    if isinstance(elem, str) and elem in structs:
        dt, stride, al = _struct(structs[elem], structs, stride=True)
        return dt, stride * n, al, (n,)
    fmt, size, al, shape = _resolve(elem, structs)
    if shape:
        raise SchemaError("nested fixed arrays are not supported")
    return fmt, size * n, al, (n,)


def compile(schema, body_len, probe):
    """Schema -> numpy dtype covering one message body (header already stripped).

    Two pseudo-types take their length from `probe`, the body of one real message:
    "str" for a string field and "hdr" for std_msgs/Header (stamp + string
    frame_id). A string only has a fixed offset if its length never changes, which is
    implied by every message on the topic having the same size -- checked by
    decode_fixed, and by the exact-size check below.
    """
    names, formats, offsets, off, maxal = [], [], [], 0, 1

    for name, typ in schema:
        if typ in ("hdr", "str"):
            off = _align(off, 4)
            maxal = max(maxal, 4)
            if typ == "hdr":
                names += [f"{name}.sec", f"{name}.nanosec"]
                formats += ["i4", "u4"]
                offsets += [off, off + 4]
                off += 8
                off = _align(off, 4)
            slen, = struct.unpack_from("<I", probe, off)
            off += 4 + slen          # the string itself is not exposed as a column
            continue
        fmt, size, al, shape = _resolve(typ, SCHEMA_STRUCTS)
        off = _align(off, al)
        maxal = max(maxal, al)
        names.append(name)
        formats.append((fmt, shape) if shape else fmt)
        offsets.append(off)
        off += size

    if off > body_len:
        raise SchemaError(f"schema needs {off} B, message body is {body_len} B")
    # Trailing slack is CDR end padding (at most 3 B). More than that means the
    # schema is missing fields, which would silently mis-plot, so refuse it.
    if body_len - off > 3:
        raise SchemaError(f"schema covers {off} of {body_len} B; "
                          f"{body_len - off} B unexplained")
    return np.dtype({"names": names, "formats": formats, "offsets": offsets,
                     "itemsize": body_len})


def decode_fixed(blobs, schema, probe=None):
    """Decode same-length messages in one pass -> structured array of len(blobs)."""
    n = len(blobs)
    if not n:
        return None
    body = len(blobs[0]) - HEADER_LEN
    dt = compile(schema, body, probe if probe is not None else blobs[0][HEADER_LEN:])
    buf = bytearray(body * n)
    for i, b in enumerate(blobs):
        if len(b) - HEADER_LEN != body:
            raise SchemaError(f"message {i} is {len(b)} B, expected {body + HEADER_LEN}")
        buf[i * body:(i + 1) * body] = b[HEADER_LEN:]
    return np.frombuffer(bytes(buf), dtype=dt, count=n)


# --- variable-layout path -----------------------------------------------------

class Reader:
    """Sequential CDR reader for messages with strings or sequences."""

    FMT = {"i8": "b", "u8": "B", "bool": "B", "i16": "h", "u16": "H",
           "i32": "i", "u32": "I", "i64": "q", "u64": "Q", "f32": "f", "f64": "d"}

    def __init__(self, payload):
        self.buf = payload[HEADER_LEN:] if payload[:2] == b"\x00\x01" else payload
        self.off = 0

    def prim(self, typ):
        size = PRIM[typ][1]
        self.off = _align(self.off, size)
        v, = struct.unpack_from("<" + self.FMT[typ], self.buf, self.off)
        self.off += size
        return v

    def string(self):
        n = self.prim("u32")
        s = self.buf[self.off:self.off + n]
        self.off += n
        return bytes(s).rstrip(b"\x00").decode("utf-8", "replace")

    def array(self, typ, n):
        fmt, size = PRIM[typ]
        self.off = _align(self.off, size)
        a = np.frombuffer(self.buf, dtype=fmt, count=n, offset=self.off)
        self.off += size * n
        return a

    def seq(self, typ):
        return self.array(typ, self.prim("u32"))

    def header(self):
        return {"sec": self.prim("i32"), "nanosec": self.prim("u32"),
                "frame_id": self.string()}
