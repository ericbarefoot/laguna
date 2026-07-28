"""Topographic profiler: thin coordinator over the gantry's own agent connection.

Scans now run entirely inside gantry_agent.py (see pi_bridge.py /
gantry_agent.py) — the same persistent Pi-side process that already handles
interactive axis commands, on a background thread with a live STOP path.
This class no longer deploys a separate script, opens a second SSH session
for the scan itself, or disconnects/reconnects the gantry agent around a
scan — it just calls start_scan()/wait_for_scan_result()/stop_scan() on the
already-connected gantry.connection, and retrieves the resulting CSV (which
still lives on the Pi's disk) via a small dedicated SFTP session.

OD2000 data collection uses HTTP polling (pdin/getdata over a persistent
connection), not MQTT — confirmed on hardware 2026-07-28 that polling
achieves ~380 Hz vs. the ~2 Hz ceiling of the AL1342's MQTT timer-push
mechanism. See docs/MQTT_AL1342_SETUP.md for the investigation.
"""

from __future__ import annotations

import datetime as _dt
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)


@dataclass
class ProfileResult:
    """Result of a topographic scan pass."""

    path: Path
    metadata: Dict[str, Any]
    df: Any = field(default=None)  # pandas DataFrame, loaded lazily


class TopographicProfiler:
    """Orchestrates a topographic scan via the gantry's own agent connection.

    Args:
        gantry: GantryController whose connection is a PiGantryConnection
            (or compatible — must expose start_scan/stop_scan/
            wait_for_scan_result). Must already be connected.
        pi_host: Hostname of the Pi (e.g., 'red.lab') — used only for the
            SFTP session that retrieves the result CSV.
        pi_user: SSH username on the Pi.
        pi_key: Path to SSH private key (optional).
        pdin_port: IO-Link port number the OD2000 is on (1–8).
        al1342_host: AL1342 IP address (raw IP, not hostname — the AL1342
            has no DNS resolution of its own; the Pi polls it directly).
        output_dir: Local directory where retrieved CSVs are saved.
    """

    def __init__(
        self,
        gantry,
        pi_host: str,
        pi_user: str,
        pi_key: Optional[str] = None,
        pdin_port: int = 1,
        al1342_host: str = "",
        output_dir: str = "/tmp",
    ):
        if not al1342_host:
            raise ValueError("al1342_host must be specified")
        self._gantry = gantry
        self._pi_host = pi_host
        self._pi_user = pi_user
        self._pi_key = pi_key
        self._pdin_port = pdin_port
        self._al1342_host = al1342_host
        self._output_dir = Path(output_dir)

    def scan(
        self,
        axis: str,
        end_mm: float,
        feed_rate_mm_s: float,
    ) -> ProfileResult:
        """Run a topographic scan pass and return the profile.

        Blocks until the scan completes (or errors). Call stop() from
        another thread to cancel it early — the agent still finishes
        normally through the same completion path, just with fewer samples.

        Args:
            axis: BLC axis prefix, e.g. 'A1' for axis 1.
            end_mm: Target position in mm (absolute).
            feed_rate_mm_s: Constant slew speed in mm/s.

        Returns:
            ProfileResult with path, metadata, and loaded DataFrame.

        Raises:
            RuntimeError: If the agent reports a scan error.
            SnapMotionError: If scan_start is rejected (blocked by the
                agent's safe_mode, or a scan is already in progress).
        """
        try:
            import pandas as pd  # type: ignore[import]
        except ImportError as e:
            raise ImportError(f"Missing dependency: {e}") from e

        timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%d_%H%M%S")
        remote_csv = f"/tmp/profile_{timestamp}.csv"
        remote_meta = f"/tmp/profile_{timestamp}_meta.json"
        local_csv = self._output_dir / f"profile_{timestamp}.csv"
        local_meta = self._output_dir / f"profile_{timestamp}_meta.json"

        logger.info("Starting scan: %s -> %.3f mm at %.3f mm/s", axis, end_mm, feed_rate_mm_s)
        ack = self._gantry.connection.start_scan(
            axis, end_mm, feed_rate_mm_s, self._al1342_host, self._pdin_port, remote_csv,
        )
        start_pos_mm = ack.get("start_pos_mm", 0.0)
        logger.info(
            "Scan started; start_pos=%.3f mm, accel=%.3f, decel=%.3f",
            start_pos_mm, ack.get("accel_mm_s2", 0), ack.get("decel_mm_s2", 0),
        )

        distance_mm = abs(end_mm - start_pos_mm)
        move_timeout = (distance_mm / feed_rate_mm_s if feed_rate_mm_s > 0 else 120.0) + 30.0

        result = self._gantry.connection.wait_for_scan_result(timeout=move_timeout)
        if "scan_error" in result:
            raise RuntimeError(f"scan error: {result['scan_error']}")

        logger.info(
            "Scan complete: %d samples, %.1f -> %.1f mm",
            result.get("samples", 0), result.get("actual_start_mm", 0), result.get("actual_end_mm", 0),
        )

        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._sftp_retrieve(remote_csv, local_csv, remote_meta, local_meta)
        logger.info("Retrieved profile CSV -> %s", local_csv)

        metadata = dict(result)
        metadata.update({
            "axis": axis,
            "end_mm": end_mm,
            "feed_rate_mm_s": feed_rate_mm_s,
            "actual_distance_mm": abs(
                result.get("actual_end_mm", end_mm) - result.get("actual_start_mm", start_pos_mm)
            ),
        })

        try:
            df = pd.read_csv(str(local_csv))
        except Exception as e:
            logger.warning("Could not load profile CSV as DataFrame: %s", e)
            df = None

        return ProfileResult(path=local_csv, metadata=metadata, df=df)

    def stop(self) -> None:
        """Cancel the currently running scan. Safe to call from another
        thread while scan() is blocked in wait_for_scan_result() — the
        agent's BST-on-cancel path still ends with a normal scan_done."""
        self._gantry.connection.stop_scan()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sftp_retrieve(self, remote_csv: str, local_csv: Path, remote_meta: str, local_meta: Path) -> None:
        """Retrieve the CSV (and best-effort the metadata sidecar) from the
        Pi's disk via a small, dedicated SFTP session — separate from the
        gantry's own persistent agent connection, which is reserved for the
        JSON command/scan protocol."""
        try:
            import paramiko  # type: ignore[import]
        except ImportError as e:
            raise ImportError(f"Missing dependency: {e}") from e

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        connect_kwargs: dict = {"username": self._pi_user, "port": 22}
        if self._pi_key:
            connect_kwargs["key_filename"] = self._pi_key
        client.connect(self._pi_host, **connect_kwargs)
        try:
            sftp = client.open_sftp()
            try:
                sftp.get(remote_csv, str(local_csv))
                try:
                    sftp.get(remote_meta, str(local_meta))
                except Exception:
                    pass
            finally:
                sftp.close()
        finally:
            client.close()
