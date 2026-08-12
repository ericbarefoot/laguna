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
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Optional, Sequence, Union

from ..motion_arbiter import DEFAULT_ARBITER

if TYPE_CHECKING:
    from .controller import GantryController
    from ...frames import FrameRegistry
    from ...rangefinder.calibration import LinearCalibration

logger = logging.getLogger(__name__)

#: Which decoded CSV column a rangefinder's calibration is fitted against —
#: mirrors RangefinderSubsystem._calibration_raw_value()'s per-device
#: override (OD2000Rangefinder uses the base class's distance_mm;
#: WTT12LRangefinder overrides to current_ma, since the PowerProx's analog
#: bridge is calibrated against the raw current loop, not a pre-decoded
#: distance). Keep these two in sync if a new rangefinder type is added.
_CALIBRATION_RAW_COLUMN = {
    "od2000": "distance_mm",
    "wtt12l": "current_ma",
    "wtt12l_powerprox": "current_ma",
}


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
        pdin_port: IO-Link port number the rangefinder (or, for
            sensor="wtt12l_powerprox", the DP4200 bridge it's wired
            through) is on (1–8).
        al1342_host: AL1342 IP address (raw IP, not hostname — the AL1342
            has no DNS resolution of its own; the Pi polls it directly).
        output_dir: Local directory where retrieved CSVs are saved.
        sensor: "od2000" (default) or "wtt12l_powerprox" — passed through
            to gantry_agent.py via start_scan(); see its SENSOR_DECODERS
            and docs/WTT12L_POWERPROX_SETUP.md. Fixed for this profiler's
            lifetime, same as pdin_port — construct a second
            TopographicProfiler if you need to scan with both sensors.
    """

    def __init__(
        self,
        gantry: "GantryController",
        pi_host: str,
        pi_user: str,
        pi_key: Optional[str] = None,
        pdin_port: int = 1,
        al1342_host: str = "",
        output_dir: str = "/tmp",
        sensor: str = "od2000",
    ):
        """Initialize the topographic profiler.

        Args:
            gantry: GantryController whose connection is a PiGantryConnection.
            pi_host: Hostname of the Pi (e.g., 'red.lab').
            pi_user: SSH username on the Pi.
            pi_key: Path to SSH private key (optional).
            pdin_port: IO-Link port number (1–8).
            al1342_host: AL1342 IP address.
            output_dir: Local directory where retrieved CSVs are saved.
            sensor: "od2000" (default) or "wtt12l_powerprox".
        """
        if not al1342_host:
            raise ValueError("al1342_host must be specified")
        self._gantry = gantry
        self._pi_host = pi_host
        self._pi_user = pi_user
        self._pi_key = pi_key
        self._pdin_port = pdin_port
        self._al1342_host = al1342_host
        self._output_dir = Path(output_dir)
        self._sensor = sensor

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

        # Millisecond resolution: two transects in the same second used to
        # collide and silently overwrite. Sensor name in the filename so a
        # directory holding both OD2000 and WTT12L output is sortable/
        # greppable without opening each file.
        timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%d_%H%M%S_%f")[:-3]
        stem = f"profile_{self._sensor}_{timestamp}"
        remote_csv = f"/tmp/{stem}.csv"
        remote_meta = f"/tmp/{stem}_meta.json"
        local_csv = self._output_dir / f"{stem}.csv"
        local_meta = self._output_dir / f"{stem}_meta.json"

        # Held for the whole physical traverse, not just the individual
        # commands the transport already serialises — see motion_arbiter.
        # Without this, a scheduled Gocator scan (which holds the same
        # arbiter via scan_with_gantry()) could command the gantry mid-pass.
        arbiter = getattr(self._gantry, "arbiter", DEFAULT_ARBITER)
        with arbiter.hold(f"topographic scan {axis} -> {end_mm:.1f}mm"):
            logger.info("Starting scan: %s -> %.3f mm at %.3f mm/s (sensor=%s)",
                        axis, end_mm, feed_rate_mm_s, self._sensor)
            ack = self._gantry.connection.start_scan(
                axis, end_mm, feed_rate_mm_s, self._al1342_host, self._pdin_port, remote_csv,
                sensor=self._sensor,
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
        """Cancel the currently running scan.

        Safe to call from another thread while scan() is blocked in
        wait_for_scan_result() — the agent's BST-on-cancel path still ends
        with a normal scan_done message.
        """
        self._gantry.connection.stop_scan()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sftp_retrieve(self, remote_csv: str, local_csv: Path, remote_meta: str, local_meta: Path) -> None:
        """Retrieve the CSV and metadata files from the Pi via SFTP.

        Retrieves CSV from remote Pi disk to local path. Attempts best-effort
        retrieval of metadata sidecar; its absence is not fatal. Uses a small,
        dedicated SFTP session separate from the gantry's own persistent agent
        connection, which is reserved for the JSON command/scan protocol.

        Args:
            remote_csv: Remote path to CSV file on Pi.
            local_csv: Local path to save CSV file.
            remote_meta: Remote path to metadata file on Pi.
            local_meta: Local path to save metadata file.

        Raises:
            ImportError: If paramiko is not installed.
            OSError: If SFTP retrieval fails.
        """
        try:
            import paramiko  # type: ignore[import]
        except ImportError as e:
            raise ImportError(f"Missing dependency: {e}") from e

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        connect_kwargs: dict = {"username": self._pi_user, "port": 22}
        if self._pi_key:
            # paramiko does not tilde-expand key_filename — see pi_bridge.py.
            connect_kwargs["key_filename"] = os.path.expanduser(self._pi_key)
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


def orient_profile(
    result: ProfileResult,
    *,
    instrument: str,
    frames: "FrameRegistry",
    calibration: Optional["LinearCalibration"] = None,
    config: Optional[Dict[str, Any]] = None,
    axis: Optional[str] = None,
    gantry_start: Optional[Sequence[float]] = None,
    output: Optional[Union[str, Path]] = None,
) -> Any:
    """Place a rangefinder transect's raw samples in experiment coordinates.

    Explicit and separate from acquisition — the same raw CSV can be
    re-oriented against a different calibration (or none) without
    re-scanning, since fitting a calibration is often done well after the
    fact. Raw columns are left untouched; ``experiment_x_mm``/``y_mm``/
    ``z_mm`` (and, if a calibration applies, ``height_mm``) are appended.

    Named distinctly from :func:`~laguna.frames.orient_scan`, its Gocator
    counterpart — same purpose and call shape (raw object in, same type
    out, optional file `output`), but importing both under the same name
    in one script forced an ``as`` alias every time, which is exactly the
    kind of friction a name should not create.

    Both OD2000 and WTT12L are mounted with a pure translation offset (no
    rotation — see ``frames.instruments.<name>``) and aren't carried on the
    Z arm, so only X/Y actually move with the gantry; Z is a constant
    mount offset plus whatever calibration resolves. Each sample's local
    point is therefore ``(0, 0, height_mm)`` — the sensor has no lateral
    extent of its own — run through the same ``FrameRegistry.to_experiment()``
    machinery ``orient_scan()`` uses for the Gocator, just per-scalar-sample
    instead of per-grid-cell.

    Args:
        result: A ProfileResult. ``FlumeLab.acquire_scan()`` writes
            ``gantry_axis``/``gantry_start`` into both the returned
            in-memory metadata and the CSV's ``_meta.json`` sidecar, so a
            ProfileResult reconstructed from disk in a later session
            (``ProfileResult(path=csv_path, metadata=json.loads(...))``,
            reading the *parsed* sidecar, not its path) still carries them.
            Scans made before this fix won't have them in the sidecar —
            pass `axis`/`gantry_start` explicitly for those.
        instrument: Instrument key ("od2000" or "wtt12l") — selects both the
            frame offset and which raw column a calibration fits against.
        frames: The lab's FrameRegistry.
        calibration: Fitted LinearCalibration to use. Takes precedence over
            `config`'s calibration_file if both are given.
        config: The instrument's own config section (e.g.
            ``lab.config.get("od2000")``) — if `calibration` is omitted,
            its ``calibration_file`` key is loaded automatically, same as
            ``RangefinderSubsystem`` does for the live-reading path. Omit
            both `calibration` and `config` to place samples at the
            gantry/mount position with no height correction — Z then
            reflects only the constant mount offset, not a real height.
        axis: Overrides ``result.metadata["gantry_axis"]``.
        gantry_start: Overrides ``result.metadata["gantry_start"]``.
        output: Optional CSV path to write the augmented DataFrame to. If
            omitted, nothing is written — the original CSV is never modified
            in place, so a different calibration can always be tried again
            from the same raw data.

    Returns:
        The DataFrame with orientation (and, if calibrated, height) columns
        appended.

    Raises:
        ValueError: If gantry_axis/gantry_start aren't available (from
            metadata or the axis/gantry_start args), the axis isn't X/Y/Z,
            the CSV lacks the columns needed (pos_mm, or calibration's raw
            column), or no DataFrame can be loaded.
    """
    import numpy as np
    import pandas as pd

    if calibration is None and config is not None:
        calibration_file = config.get("calibration_file")
        if calibration_file:
            from ...rangefinder.calibration import LinearCalibration

            calibration = LinearCalibration.from_csv(calibration_file)

    df = result.df if result.df is not None else pd.read_csv(str(result.path))
    df = df.copy()

    axis_name = axis if axis is not None else result.metadata.get("gantry_axis")
    if gantry_start is None:
        gantry_start = result.metadata.get("gantry_start")
    if axis_name is None or gantry_start is None:
        raise ValueError(
            "orient_profile() needs gantry_axis/gantry_start — from "
            "metadata (only present if FlumeLab.acquire_scan() produced "
            "this scan, or its _meta.json sidecar was loaded into "
            "result.metadata) or passed explicitly as axis=/gantry_start="
        )
    axis_index = {"X": 0, "Y": 1, "Z": 2}.get(axis_name)
    if axis_index is None:
        raise ValueError(f"gantry_axis {axis_name!r} is not one of X/Y/Z")
    if "pos_mm" not in df.columns:
        raise ValueError("orient_profile() requires a 'pos_mm' column (the travel-axis position)")

    n = len(df)
    gantry_positions = np.tile(np.asarray(gantry_start[:3], dtype=float), (n, 1))
    gantry_positions[:, axis_index] = df["pos_mm"].to_numpy(dtype=float)

    local_points = np.zeros((n, 3))
    if calibration is not None:
        raw_col = _CALIBRATION_RAW_COLUMN.get(instrument, "distance_mm")
        if raw_col not in df.columns:
            raise ValueError(
                f"calibration for {instrument!r} needs column {raw_col!r}, "
                f"not present in this CSV (have {list(df.columns)})"
            )
        df["height_mm"] = calibration.slope * df[raw_col].to_numpy(dtype=float) + calibration.intercept
        local_points[:, 2] = df["height_mm"].to_numpy(dtype=float)
    else:
        logger.warning(
            "orient_profile(%s): no calibration given or configured — "
            "experiment_z_mm reflects only the constant mount offset, not a real height",
            instrument,
        )

    experiment_points = frames.to_experiment(instrument, local_points, gantry_positions)
    df["experiment_x_mm"] = experiment_points[:, 0]
    df["experiment_y_mm"] = experiment_points[:, 1]
    df["experiment_z_mm"] = experiment_points[:, 2]

    if output is not None:
        output_path = Path(output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(output_path, index=False)

    return df
