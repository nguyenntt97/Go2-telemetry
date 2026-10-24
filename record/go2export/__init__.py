"""Flatten a recorded Go2 session into one table per topic.

The pieces are kept apart so each can be tested on its own without a robot:
`bagreader` reads the sqlite3 bag, `msgdef` and `cdr` and `decode` turn blobs
into dicts, `flatten` and `writers` make tables, and `video` and `clouds`
handle the two payloads that do not belong in a table.
"""

__all__ = [
    "bagreader",
    "cdr",
    "clouds",
    "decode",
    "flatten",
    "msgdef",
    "video",
    "writers",
]
