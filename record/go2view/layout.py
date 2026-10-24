"""The default view layout.

Built from the telemetry actually selected, so a session without UWB or video
does not open with empty panels.
"""

from __future__ import annotations

import rerun.blueprint as rrb

#: Plot tabs a group contributes, as (name, entity path).
_PLOT_TABS = {
    "pose": (
        ("Velocity", "/plots/velocity"),
        ("Yaw rate", "/plots/yaw_speed"),
        ("Gait", "/plots/gait"),
    ),
    "imu": (("IMU", "/plots/imu"),),
    "feet": (("Foot force", "/plots/feet/force"),),
    "battery": (("Battery", "/plots/battery"),),
    "uwb": (("UWB angles", "/plots/uwb"),),
    "odom": (("Odometry", "/plots/odom"),),
    "controller": (("Controller", "/plots/controller"),),
    "jetson": (("Jetson load", "/plots/jetson"),),
}


def build(selected_keys, video_paths=(), has_events: bool = False):
    """Return a Blueprint for the selected groups."""
    keys = set(selected_keys)

    tabs = []
    for key, entries in _PLOT_TABS.items():
        if key not in keys:
            continue
        for name, origin in entries:
            tabs.append(rrb.TimeSeriesView(origin=origin, name=name))
    for path in video_paths:
        tabs.append(rrb.Spatial2DView(origin=path, name=path.rsplit("/", 1)[-1]))
    if has_events:
        tabs.append(rrb.TextLogView(origin="/events", name="Events"))

    spatial = rrb.Spatial3DView(origin="/world", name="World")
    if not tabs:
        return rrb.Blueprint(spatial, collapse_panels=False)
    return rrb.Blueprint(
        rrb.Horizontal(spatial, rrb.Tabs(*tabs), column_shares=[3, 2]),
        collapse_panels=False,
    )
