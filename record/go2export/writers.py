"""Collect rows into columns and write them as CSV or Parquet.

Rows are accumulated columnar, so a topic with varying sequence lengths still
produces one stable set of columns. Parquet needs pyarrow; CSV needs nothing,
so CSV is always available on the Jetson.
"""

from __future__ import annotations

import csv
import os


def pyarrow_available() -> bool:
    try:
        import pyarrow  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


class ColumnTable:
    """Column store that tolerates rows with different keys."""

    def __init__(self) -> None:
        self._columns: dict[str, list] = {}
        self._rows = 0

    def __len__(self) -> int:
        return self._rows

    @property
    def columns(self) -> list:
        return list(self._columns)

    def column(self, name: str) -> list:
        return self._columns[name]

    def add(self, row: dict) -> None:
        for key, value in row.items():
            column = self._columns.get(key)
            if column is None:
                # A column seen for the first time is backfilled, so every
                # column stays the same length.
                column = self._columns[key] = [None] * self._rows
            column.append(value)
        self._rows += 1
        for key, column in self._columns.items():
            if len(column) < self._rows:
                column.append(None)

    def _normalized(self) -> dict:
        """Make each column a single type, so Parquet can take it."""
        out = {}
        for name, column in self._columns.items():
            kinds = {type(v) for v in column if v is not None}
            if len(kinds) > 1 and kinds != {int, float}:
                out[name] = ["" if v is None else str(v) for v in column]
            elif kinds == {int, float}:
                out[name] = [None if v is None else float(v) for v in column]
            else:
                out[name] = column
        return out


def write_csv(table: ColumnTable, path: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    columns = table.columns
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        rows = [table.column(name) for name in columns]
        for index in range(len(table)):
            writer.writerow(
                ["" if row[index] is None else row[index] for row in rows]
            )
    return path


def write_parquet(table: ColumnTable, path: str) -> str:
    import pyarrow
    import pyarrow.parquet

    os.makedirs(os.path.dirname(path), exist_ok=True)
    arrow_table = pyarrow.table(table._normalized())
    pyarrow.parquet.write_table(arrow_table, path, compression="snappy")
    return path


def topic_filename(topic: str) -> str:
    """`/lf/lowstate` -> `lf__lowstate`."""
    name = topic.strip("/").replace("/", "__")
    return "".join(char if char.isalnum() or char in "_-." else "_" for char in name)
