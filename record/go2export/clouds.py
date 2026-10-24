"""Write point cloud frames out as `.npz`, one file per frame.

A session's clouds are far too large for a table, so they are kept out of the
per-topic export unless `--clouds` is given.
"""

from __future__ import annotations

import os

#: sensor_msgs/PointField datatype -> numpy dtype string
_DATATYPES = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}


class CloudError(ValueError):
    pass


def numpy_available() -> bool:
    try:
        import numpy  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


def cloud_to_arrays(message: dict) -> dict:
    """Turn a decoded PointCloud2 into one array per field."""
    import numpy

    data = message.get("data")
    if not isinstance(data, (bytes, bytearray)):
        raise CloudError("PointCloud2 has no data bytes")
    point_step = int(message.get("point_step") or 0)
    if point_step <= 0:
        raise CloudError("PointCloud2 has no point_step")

    names, formats, offsets = [], [], []
    for pointfield in message.get("fields") or []:
        datatype = int(pointfield.get("datatype", 0))
        if datatype not in _DATATYPES:
            continue
        count = int(pointfield.get("count") or 1)
        base = _DATATYPES[datatype]
        names.append(str(pointfield.get("name")))
        formats.append(f"{count}{base}" if count > 1 else base)
        offsets.append(int(pointfield.get("offset") or 0))
    if not names:
        raise CloudError("PointCloud2 declares no usable fields")

    endian = ">" if message.get("is_bigendian") else "<"
    dtype = numpy.dtype(
        {
            "names": names,
            "formats": [endian + f for f in formats],
            "offsets": offsets,
            "itemsize": point_step,
        }
    )
    points = len(data) // point_step
    structured = numpy.frombuffer(bytes(data)[: points * point_step], dtype=dtype)
    return {name: structured[name] for name in names}


def write_frame(message: dict, out_dir: str, index: int, timestamp_ns: int) -> str:
    import numpy

    arrays = cloud_to_arrays(message)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{index:06d}_{timestamp_ns}.npz")
    numpy.savez_compressed(path, **arrays)
    return path
