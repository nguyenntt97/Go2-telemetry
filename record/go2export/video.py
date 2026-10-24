"""Extract the front camera H.264 streams.

The local `Go2FrontVideoData` definition does not match what the robot sends,
so this topic is parsed from the bytes instead of through a message type. Each
message is, after the four byte CDR header: `uint64 time_frame`,
`uint32 resolution` (720 or 360), `uint32 length`, then that many bytes of
H.264 Annex-B. The two resolutions are interleaved on the one topic at about
14 Hz each, with a keyframe roughly every 2 s.

Frames before the first keyframe of a stream cannot be decoded, so they are
dropped and counted rather than written.
"""

from __future__ import annotations

import os

from .cdr import CdrError, CdrReader

#: NAL unit types that start a decodable sequence.
_KEYFRAME_NALS = frozenset({5, 7})  # IDR slice, SPS


class VideoError(ValueError):
    pass


def parse_frame(blob: bytes) -> dict:
    """Pull one video message apart. Raises :class:`VideoError` if it does not
    match the measured layout."""
    try:
        reader = CdrReader(blob)
        time_frame = reader.primitive("uint64")
        resolution = reader.primitive("uint32")
        length = reader.primitive("uint32")
        if length > reader.remaining:
            raise VideoError(
                f"frame claims {length} bytes but only {reader.remaining} remain"
            )
        payload = reader.block(length)
    except CdrError as exc:
        raise VideoError(str(exc)) from exc
    return {
        "time_frame": time_frame,
        "resolution": resolution,
        "length": length,
        "payload": payload,
        "trailing": reader.remaining,
    }


def nal_units(payload: bytes):
    """Yield ``(nal_type, start, end)`` for each Annex-B NAL unit."""
    positions = []
    index = 0
    size = len(payload)
    while True:
        index = payload.find(b"\x00\x00\x01", index)
        if index < 0:
            break
        start = index + 3
        # A four byte start code is a three byte one with an extra leading zero.
        positions.append(start)
        index = start
    for number, start in enumerate(positions):
        end = positions[number + 1] - 3 if number + 1 < len(positions) else size
        while end > start and payload[end - 1] == 0:
            end -= 1  # trim the zeros belonging to the next start code
        if start < size:
            yield payload[start] & 0x1F, start, end


def is_keyframe(payload: bytes) -> bool:
    return any(nal_type in _KEYFRAME_NALS for nal_type, _, _ in nal_units(payload))


class StreamWriter:
    """One output stream, i.e. one resolution."""

    def __init__(self, path: str) -> None:
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._handle = open(path, "wb")
        self.frames = 0
        self.bytes = 0
        self.skipped_before_keyframe = 0
        self.keyframes = 0
        self._started = False
        self.first_ns: int | None = None
        self.last_ns: int | None = None

    def add(self, timestamp_ns: int, payload: bytes) -> dict | None:
        keyframe = is_keyframe(payload)
        if not self._started:
            if not keyframe:
                self.skipped_before_keyframe += 1
                return None
            self._started = True
        offset = self.bytes
        self._handle.write(payload)
        self.bytes += len(payload)
        self.frames += 1
        if keyframe:
            self.keyframes += 1
        if self.first_ns is None:
            self.first_ns = timestamp_ns
        self.last_ns = timestamp_ns
        return {
            "index": self.frames - 1,
            "t_ns": timestamp_ns,
            "byte_offset": offset,
            "length": len(payload),
            "keyframe": int(keyframe),
        }

    @property
    def fps(self) -> float:
        if self.frames < 2 or self.first_ns is None or self.last_ns is None:
            return 0.0
        span = (self.last_ns - self.first_ns) / 1e9
        return (self.frames - 1) / span if span > 0 else 0.0

    def close(self) -> None:
        self._handle.close()


def transcode_mp4(h264_path: str, fps: float) -> dict:
    """Re-encode a raw stream to `.mp4` with OpenCV.

    This decodes and re-encodes, so it is lossy; the `.h264` file beside it is
    the recorded bytes untouched. ffmpeg is not installed on the Jetson, but
    OpenCV carries its own.
    """
    result = {"path": None, "frames": 0, "error": None}
    try:
        import cv2
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"OpenCV not available: {exc}"
        return result

    capture = cv2.VideoCapture(h264_path)
    if not capture.isOpened():
        result["error"] = f"OpenCV could not open {h264_path}"
        return result
    writer = None
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if writer is None:
                height, width = frame.shape[:2]
                mp4_path = os.path.splitext(h264_path)[0] + ".mp4"
                writer = cv2.VideoWriter(
                    mp4_path,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    fps if fps > 1 else 14.0,
                    (width, height),
                )
                if not writer.isOpened():
                    result["error"] = "OpenCV could not open an mp4 writer"
                    return result
                result["path"] = mp4_path
            writer.write(frame)
            result["frames"] += 1
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    return result
