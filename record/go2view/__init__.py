"""Visualize a recorded Go2 session in Rerun, served to a remote client.

`sessions` finds and picks a recording, `streams` says what each topic
contributes and logs it, `layout` builds the default panel arrangement. The
decoding underneath is `go2export`, so no ROS is needed and this runs wherever
the logs are mounted.
"""

__all__ = ["layout", "sessions", "streams"]
