"""Decode CDR blobs into plain dicts, driven by `.msg` definitions.

Two decoders are available. `ros` uses the installed typesupport, which is
authoritative when the types are built; `msgdef` reads the `.msg` files and
walks the layout itself, which needs no ROS at all. `auto` prefers `ros` and
falls back. Both are exact only insofar as the local definitions match what
the robot actually sends, which for `Go2FrontVideoData` they do not -- that
topic has its own parser in `video.py`.
"""

from __future__ import annotations

from .cdr import BYTE_TYPES, CdrReader, CdrWriter
from .msgdef import Field, MsgRegistry


class DecodeError(ValueError):
    """A blob could not be decoded as the topic's type."""


def _read_value(reader: CdrReader, base: str, registry: MsgRegistry):
    if base == "string":
        return reader.string()
    if base in BYTE_TYPES or base in ("bool", "int16", "uint16", "int32",
                                      "uint32", "int64", "uint64",
                                      "float32", "float64"):
        return reader.primitive(base)
    return decode_fields(reader, registry.fields(base), registry)


def _read_field(reader: CdrReader, field: Field, registry: MsgRegistry):
    if field.array is None:
        return _read_value(reader, field.base, registry)

    kind, size = field.array
    count = reader.uint32() if kind == "seq" else size

    # One byte wide elements come back as bytes; flatten.py records their
    # length rather than expanding them into thousands of columns.
    if field.base in BYTE_TYPES:
        return reader.block(count)
    if field.base == "string":
        return [reader.string() for _ in range(count)]
    if field.is_primitive:
        return reader.array(field.base, count)
    nested = registry.fields(field.base)
    return [decode_fields(reader, nested, registry) for _ in range(count)]


def decode_fields(reader: CdrReader, fields, registry: MsgRegistry) -> dict:
    return {field.name: _read_field(reader, field, registry) for field in fields}


def decode_message(blob: bytes, type_name: str, registry: MsgRegistry) -> dict:
    """Decode one blob of the named type into nested dicts."""
    try:
        reader = CdrReader(blob)
        return decode_fields(reader, registry.fields(type_name), registry)
    except DecodeError:
        raise
    except Exception as exc:  # noqa: BLE001 - reported per topic, never fatal
        raise DecodeError(f"{type_name}: {exc}") from exc


# --- encoding, for the tests -------------------------------------------------


def _write_value(writer: CdrWriter, base: str, value, registry: MsgRegistry) -> None:
    if base == "string":
        writer.string(value or "")
    elif base in BYTE_TYPES or base in ("bool", "int16", "uint16", "int32",
                                        "uint32", "int64", "uint64",
                                        "float32", "float64"):
        writer.primitive(base, value)
    else:
        encode_fields(writer, registry.fields(base), value or {}, registry)


def _write_field(writer: CdrWriter, field: Field, value, registry: MsgRegistry) -> None:
    if field.array is None:
        _write_value(writer, field.base, value, registry)
        return

    kind, size = field.array
    if field.base in BYTE_TYPES and isinstance(value, (bytes, bytearray)):
        data = bytes(value)
        if kind == "seq":
            writer.uint32(len(data))
        writer.block(data)
        return

    values = list(value or [])
    if kind == "fixed":
        if len(values) != size:
            raise ValueError(f"{field.name}: want {size} elements, got {len(values)}")
    else:
        writer.uint32(len(values))

    if field.base == "string":
        for item in values:
            writer.string(item or "")
    elif field.is_primitive:
        writer.array(field.base, values)
    else:
        nested = registry.fields(field.base)
        for item in values:
            encode_fields(writer, nested, item or {}, registry)


def encode_fields(writer: CdrWriter, fields, data: dict, registry: MsgRegistry) -> None:
    for field in fields:
        _write_field(writer, field, data.get(field.name), registry)


def encode_message(data: dict, type_name: str, registry: MsgRegistry) -> bytes:
    """Serialize a dict as the named type. Inverse of :func:`decode_message`."""
    writer = CdrWriter()
    encode_fields(writer, registry.fields(type_name), data, registry)
    return writer.getvalue()


# --- the ROS typesupport path ------------------------------------------------


class RosDecoder:
    """Decodes with the installed typesupport, when ROS is importable."""

    def __init__(self) -> None:
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import deserialize_message

        self._get_message = get_message
        self._deserialize = deserialize_message
        self._to_dict = message_to_ordereddict
        self._types: dict = {}

    @staticmethod
    def available() -> bool:
        try:
            import rclpy.serialization  # noqa: F401
            import rosidl_runtime_py.utilities  # noqa: F401
        except Exception:  # noqa: BLE001
            return False
        return True

    def decode(self, blob: bytes, type_name: str) -> dict:
        try:
            cls = self._types.get(type_name)
            if cls is None:
                cls = self._types[type_name] = self._get_message(type_name)
            return dict(self._to_dict(self._deserialize(blob, cls)))
        except Exception as exc:  # noqa: BLE001
            raise DecodeError(f"{type_name}: {exc}") from exc


class Decoder:
    """Picks a decode path per topic and remembers which one worked."""

    def __init__(self, registry: MsgRegistry, prefer: str = "auto") -> None:
        if prefer not in ("auto", "ros", "msgdef"):
            raise ValueError(f"unknown decoder {prefer!r}")
        self.registry = registry
        self.prefer = prefer
        self._ros: RosDecoder | None = None
        if prefer in ("auto", "ros") and RosDecoder.available():
            try:
                self._ros = RosDecoder()
            except Exception:  # noqa: BLE001
                self._ros = None
        if prefer == "ros" and self._ros is None:
            raise DecodeError("--decoder ros asked for, but ROS is not importable here")
        self._path: dict[str, str] = {}

    def path_for(self, type_name: str) -> str:
        return self._path.get(type_name, "")

    def decode(self, blob: bytes, type_name: str) -> dict:
        if self._ros is not None and self.prefer != "msgdef":
            try:
                result = self._ros.decode(blob, type_name)
                self._path[type_name] = "ros"
                return result
            except DecodeError:
                if self.prefer == "ros":
                    raise
        result = decode_message(blob, type_name, self.registry)
        self._path[type_name] = "msgdef"
        return result
