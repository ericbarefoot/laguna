"""Timestamp utilities for camera logging."""

import time


def utc_local_ts() -> tuple[str, str]:
    """Return (utc_str, local_str) with millisecond precision for log lines."""
    now = time.time()
    utc = time.strftime("%H:%M:%S", time.gmtime(now)) + f".{int(now % 1 * 1000):03d}Z"
    local = time.strftime("%H:%M:%S", time.localtime(now))
    return utc, local
