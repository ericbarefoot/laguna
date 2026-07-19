"""Topographic profiler: Pi-local scan runner orchestration.

Deploys scan_runner.py to the Pi via SFTP, triggers it, waits for the result,
retrieves the CSV, and reconnects the gantry agent. All BLC serial commands
and MQTT subscription happen on the Pi — this class is a pure coordinator.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
import datetime as _dt
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger(__name__)

SCAN_RUNNER_SCRIPT = Path(__file__).parent.parent.parent / "pi" / "scan_runner.py"
REMOTE_SCAN_RUNNER = "/tmp/laguna_scan_runner.py"
READY_TIMEOUT = 15.0


@dataclass
class ProfileResult:
    """Result of a topographic scan pass."""

    path: Path
    metadata: Dict[str, Any]
    df: Any = field(default=None)  # pandas DataFrame, loaded lazily


class TopographicProfiler:
    """Orchestrates a topographic scan: deploy → trigger → retrieve → reconnect.

    This class does NOT send any serial commands itself. scan_runner.py on the
    Pi handles all BLC ASCII communication (ACP, ACL, DCL, SPD, BMT, MIF) and
    MQTT subscription in one Pi-local clock domain.

    Args:
        gantry: GantryController with a PiGantryConnection; its
            connection.disconnect()/connect() are used to release and
            reclaim the BLC serial port around the scan.
        pi_host: Hostname of the Pi (e.g., 'red.lab').
        pi_user: SSH username on the Pi.
        pi_key: Path to SSH private key (optional).
        serial_device: Serial device on the Pi for the BLC RS232 connection.
        baud: Baud rate for the BLC connection (default 9600).
        pdin_port: IO-Link port number the OD2000 is on (1–8).
        od2000_topic: MQTT topic the AL1342 publishes OD2000 data to.
        output_dir: Local directory where retrieved CSVs are saved.
    """

    def __init__(
        self,
        gantry,
        pi_host: str,
        pi_user: str,
        pi_key: Optional[str] = None,
        serial_device: str = "",
        baud: int = 9600,
        pdin_port: int = 1,
        od2000_topic: str = "laguna/od2000",
        output_dir: str = "/tmp",
    ):
        if not serial_device:
            raise ValueError("serial_device must be specified")
        self._gantry = gantry
        self._pi_host = pi_host
        self._pi_user = pi_user
        self._pi_key = pi_key
        self._serial_device = serial_device
        self._baud = baud
        self._pdin_port = pdin_port
        self._od2000_topic = od2000_topic
        self._output_dir = Path(output_dir)

    def scan(
        self,
        axis: str,
        end_mm: float,
        feed_rate_mm_s: float,
    ) -> ProfileResult:
        """Run a topographic scan pass and return the profile.

        Releases the gantry serial connection before the scan and
        reconnects it after. The scan_runner.py on the Pi performs all
        BLC serial communication and MQTT data collection.

        Args:
            axis: BLC axis prefix, e.g. 'A1' for axis 1.
            end_mm: Target position in mm (absolute).
            feed_rate_mm_s: Constant slew speed in mm/s.

        Returns:
            ProfileResult with path, metadata, and loaded DataFrame.

        Raises:
            RuntimeError: If scan_runner.py reports an error, or if
                the ready/done protocol times out.
        """
        try:
            import paramiko  # type: ignore[import]
            import pandas as pd  # type: ignore[import]
        except ImportError as e:
            raise ImportError(f"Missing dependency: {e}") from e

        timestamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%d_%H%M%S")
        remote_csv = f"/tmp/profile_{timestamp}.csv"
        remote_meta = f"/tmp/profile_{timestamp}_meta.json"
        local_csv = self._output_dir / f"profile_{timestamp}.csv"
        local_meta = self._output_dir / f"profile_{timestamp}_meta.json"

        # -------------------------------------------------------------- 1. release port
        logger.info("Disconnecting gantry agent to release serial port for scan")
        self._gantry.connection.disconnect()

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        connect_kwargs: dict = {"username": self._pi_user, "port": 22}
        if self._pi_key:
            connect_kwargs["key_filename"] = self._pi_key

        try:
            # ---------------------------------------------------------- 2. SSH
            client.connect(self._pi_host, **connect_kwargs)
            logger.info("SSH connected to %s", self._pi_host)

            # ---------------------------------------------------------- 3. deploy
            sftp = client.open_sftp()
            try:
                sftp.put(str(SCAN_RUNNER_SCRIPT), REMOTE_SCAN_RUNNER)
                logger.info("Deployed scan_runner.py → %s", REMOTE_SCAN_RUNNER)
            finally:
                sftp.close()

            # ---------------------------------------------------------- 4. exec
            cmd = (
                f"python3 {REMOTE_SCAN_RUNNER}"
                f" --serial-port {self._serial_device}"
                f" --baud {self._baud}"
                f" --axis {axis}"
                f" --end-mm {end_mm}"
                f" --feed-rate-mm-s {feed_rate_mm_s}"
                f" --od2000-topic {self._od2000_topic}"
                f" --pdin-port {self._pdin_port}"
                f" --output {remote_csv}"
            )
            logger.info("Launching scan_runner: %s", cmd)
            stdin_ch, stdout_ch, _stderr_ch = client.exec_command(cmd)
            channel = stdout_ch.channel

            # ---------------------------------------------------------- 5. await ready
            ready_msg = self._read_json_line(channel, timeout=READY_TIMEOUT,
                                             label="ready", stdin=stdin_ch)
            if "error" in ready_msg:
                raise RuntimeError(f"scan_runner error: {ready_msg['error']}")
            if not ready_msg.get("ready"):
                raise RuntimeError(f"Unexpected message from scan_runner: {ready_msg}")

            start_pos_mm = ready_msg.get("start_pos_mm", 0.0)
            logger.info("scan_runner ready; start_pos=%.3f mm, accel=%.3f, decel=%.3f",
                        start_pos_mm,
                        ready_msg.get("accel_mm_s2", 0),
                        ready_msg.get("decel_mm_s2", 0))

            # ---------------------------------------------------------- 6. await done
            distance_mm = abs(end_mm - start_pos_mm)
            move_timeout = (distance_mm / feed_rate_mm_s if feed_rate_mm_s > 0 else 120.0) + 30.0
            done_msg = self._read_json_line(channel, timeout=move_timeout,
                                            label="done", stdin=stdin_ch)
            if "error" in done_msg:
                raise RuntimeError(f"scan_runner error: {done_msg['error']}")
            if not done_msg.get("done"):
                raise RuntimeError(f"Unexpected message from scan_runner: {done_msg}")

            logger.info("Scan complete: %d samples, %.1f → %.1f mm",
                        done_msg.get("samples", 0),
                        done_msg.get("actual_start_mm", 0),
                        done_msg.get("actual_end_mm", 0))

            # ---------------------------------------------------------- 7. retrieve
            self._output_dir.mkdir(parents=True, exist_ok=True)
            sftp = client.open_sftp()
            try:
                sftp.get(remote_csv, str(local_csv))
                try:
                    sftp.get(remote_meta, str(local_meta))
                except Exception:
                    pass
            finally:
                sftp.close()
            logger.info("Retrieved profile CSV → %s", local_csv)

        finally:
            # ---------------------------------------------------------- 8. close SSH
            try:
                client.close()
            except Exception:
                pass

            # ---------------------------------------------------------- 9. reconnect gantry
            logger.info("Reconnecting gantry agent")
            try:
                self._gantry.connection.connect()
            except Exception as e:
                logger.error("Failed to reconnect gantry: %s", e)

        # ---------------------------------------------------------------- 10. return
        metadata = done_msg.copy()
        metadata.update({
            "axis": axis,
            "end_mm": end_mm,
            "feed_rate_mm_s": feed_rate_mm_s,
            "actual_distance_mm": abs(
                done_msg.get("actual_end_mm", end_mm)
                - done_msg.get("actual_start_mm", start_pos_mm)
            ),
        })

        try:
            df = pd.read_csv(str(local_csv))
        except Exception as e:
            logger.warning("Could not load profile CSV as DataFrame: %s", e)
            df = None

        return ProfileResult(path=local_csv, metadata=metadata, df=df)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read_json_line(self, channel, timeout: float, label: str, stdin=None) -> dict:
        """Read JSON lines from channel stdout until the expected message arrives."""
        deadline = time.monotonic() + timeout
        buf = b""
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"Timed out after {timeout:.0f}s waiting for '{label}' from scan_runner"
                )
            if channel.recv_ready():
                buf += channel.recv(4096)
            if channel.recv_stderr_ready():
                for line in channel.recv_stderr(4096).decode("utf-8", errors="replace").splitlines():
                    if line.strip():
                        logger.info("[scan_runner] %s", line.strip())
            if b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    logger.debug("[scan_runner stdout] %s", line)
                    continue
            elif channel.exit_status_ready():
                raise RuntimeError("scan_runner exited unexpectedly before sending '{label}'")
            else:
                time.sleep(0.05)
