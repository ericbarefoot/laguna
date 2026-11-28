#!/usr/bin/env python3
"""Networked Raspberry Pi camera array — synchronised capture over SSH.

This module is the authoritative home of CameraArray and CaptureResult.
The sandbox/camera_coordinator.py CLI re-exports from here.

Standalone CLI usage (same as the sandbox script):
    python3 -m laguna.camera.network
    python3 -m laguna.camera.network --cameras antares.laguna sirius.laguna
    python3 -m laguna.camera.network --fetch-images --output-dir ./captures
    python3 -m laguna.camera.network --ssh-key ~/.ssh/id_rsa --check-sync
"""

import json
import logging
import os
import math
import os
import shlex
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Optional

import paramiko

from ..subsystem_logging import SubsystemLogging
from .exposure import ExposureControls, ExposureSettings, compare_controls

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)


def assess_spread(spread_ms: float) -> str:
    """Return a quality label for inter-camera capture spread.

    Thresholds:
        EXCELLENT  < 20 ms
        GOOD       < 50 ms
        MARGINAL   < 200 ms
        POOR       >= 200 ms  — likely an NTP sync problem
    """
    if spread_ms < 20:
        return "EXCELLENT"
    if spread_ms < 50:
        return "GOOD"
    if spread_ms < 200:
        return "MARGINAL"
    return "POOR — check NTP sync"

# Agent script lives alongside this file in the package.
AGENT_SCRIPT = Path(__file__).parent / "agent.py"
REMOTE_AGENT_PATH = "/tmp/laguna_camera_agent.py"
REMOTE_CAPTURE_DIR = "/tmp/laguna_captures"

DEFAULT_CAMERAS = ["antares.laguna", "sirius.laguna"]
DEFAULT_SSH_USER = "pi"
DEFAULT_LEAD_TIME = 5.0


@dataclass
class CaptureResult:
    """Result of a single camera capture from the networked array.

    Attributes:
        hostname: Hostname of the Raspberry Pi camera.
        success: True if capture succeeded, False otherwise.
        filename: Path to captured image file on the Pi (None if capture failed).
        target_time: Target Unix timestamp for capture (PC clock).
        capture_time_mid: Actual mid-capture Unix timestamp (Pi clock frame).
        capture_time_mid_pc: Actual mid-capture Unix timestamp (converted to PC clock).
        capture_duration_ms: Duration of capture exposure in milliseconds.
        latency_ms: Time from target to actual capture start in milliseconds.
        clock_offset_s: Measured clock offset (Pi time − PC time) in seconds.
        error: Error message if capture failed.
        exposure_mode: "locked" if AE/AWB were disabled and fixed controls applied,
            "auto" otherwise (None if the agent did not say).
        exposure_time_us: Exposure the camera reported for the frame (locked captures).
        analogue_gain: Gain the camera reported for the frame (locked captures).
        colour_gains: ``[red, blue]`` white-balance gains reported for the frame.
        exposure_warning: Set when a locked capture's reported values differ from
            what was commanded; the frame is kept but should not be trusted.
    """

    hostname: str
    success: bool
    filename: Optional[str] = None
    target_time: Optional[float] = None
    capture_time_mid: Optional[float] = None       # in Pi's clock frame
    capture_time_mid_pc: Optional[float] = None    # converted to PC clock frame
    capture_duration_ms: Optional[float] = None
    latency_ms: Optional[float] = None
    clock_offset_s: Optional[float] = None         # Pi time − PC time
    error: Optional[str] = None
    exposure_mode: Optional[str] = None
    exposure_time_us: Optional[float] = None
    analogue_gain: Optional[float] = None
    colour_gains: Optional[List[float]] = None
    exposure_warning: Optional[str] = None


@dataclass
class CalibrationResult:
    """Outcome of calibrating one Pi's exposure and white balance.

    Attributes:
        hostname: Hostname of the Raspberry Pi camera.
        success: True if AE/AWB values were read back.
        controls: The converged values, ready to confirm and save.
        converged: True if the sampled frames agreed closely enough.
        spread: Largest relative max-min across the sampled frames.
        filename: Preview JPEG on the Pi (None on failure).
        local_preview: Where the preview was fetched to, if it was.
        error: Error message if calibration failed.
    """

    hostname: str
    success: bool
    controls: Optional[ExposureControls] = None
    converged: bool = False
    spread: Optional[float] = None
    filename: Optional[str] = None
    local_preview: Optional[Path] = None
    error: Optional[str] = None


class CameraArray(SubsystemLogging):
    """Manages a set of networked Raspberry Pi cameras for synchronised capture."""

    subsystem_name = "pi_cameras"

    def __init__(
        self,
        hosts: List[str] = DEFAULT_CAMERAS,
        ssh_user: str = DEFAULT_SSH_USER,
        ssh_key: Optional[str] = None,
        ssh_passphrase: Optional[str] = None,
        log_level: str = "INFO",
        event_log_verbosity: str = "INFO",
        simulated: bool = False,
        exposure: Optional[ExposureSettings] = None,
    ):
        """Initialize a networked Pi camera array.

        Args:
            hosts: List of Pi hostnames (default: DEFAULT_CAMERAS).
            ssh_user: SSH user for Pi connections (default: DEFAULT_SSH_USER).
            ssh_key: Path to SSH private key file (optional).
            ssh_passphrase: Passphrase for encrypted SSH key (optional).
            log_level: Logging level (default 'INFO').
            event_log_verbosity: Event log verbosity (default 'INFO').
            simulated: If True, skip real SSH and hardware I/O, return placeholder
                results (default False; see laguna.simulation module docstring).
            exposure: Fixed exposure / white-balance settings (default: automatic,
                the historical behaviour). See laguna.camera.exposure.

        Raises:
            ValueError: If ``exposure`` is a locked mode that has no controls for
                one of ``hosts`` — found here, before a run starts, so a gap never
                becomes silently automatic frames.
        """
        self.hosts = hosts
        self.ssh_user = ssh_user
        self.ssh_key = ssh_key
        self.ssh_passphrase = ssh_passphrase
        self.log_level = log_level
        self.event_log_verbosity = event_log_verbosity
        self._simulated = simulated
        self.exposure = exposure if exposure is not None else ExposureSettings()
        self.exposure.validate_hosts(self.hosts)
        #: Where fetched frames are written. setup_run() sets this from the
        #: run directory, or leaves the historical ./captures/pi default.
        self.output_dir = Path("./captures/pi")

    @classmethod
    def from_config(cls, config: "Config") -> "CameraArray":
        """Build from the lab's Config (its 'pi_cameras:' section).

        hosts defaults to [] here, not this class's own DEFAULT_CAMERAS
        (real named lab cameras) — a pi_cameras: section with some other
        key set but no explicit hosts: must not silently target real
        hardware. ssh_user defaults to "ucrs" here, not this class's own
        "pi" default — both match laguna.experiment.runner's historical
        setup_run() defaults, which this classmethod replaces.
        """
        pi_cfg = config.get("pi_cameras")
        return cls(
            hosts=pi_cfg.get("hosts", []),
            ssh_user=pi_cfg.get("ssh_user", "ucrs"),
            ssh_key=pi_cfg.get("ssh_key"),
            log_level=pi_cfg.get("log_level", "INFO"),
            event_log_verbosity=pi_cfg.get("event_log_verbosity", "INFO"),
            simulated=pi_cfg.get("simulated", False),
            exposure=ExposureSettings.from_config(pi_cfg.get("exposure")),
        )

    def _connect(self, hostname: str) -> paramiko.SSHClient:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        kwargs: dict = {"username": self.ssh_user, "timeout": 10}
        if self.ssh_key:
            # paramiko does not tilde-expand key_filename — see pi_bridge.py.
            kwargs["key_filename"] = os.path.expanduser(self.ssh_key)
        if self.ssh_passphrase:
            kwargs["passphrase"] = self.ssh_passphrase
        client.connect(hostname, **kwargs)
        return client

    def _deploy_agent(self, client: paramiko.SSHClient) -> None:
        sftp = client.open_sftp()
        sftp.put(str(AGENT_SCRIPT), REMOTE_AGENT_PATH)
        sftp.close()

    def _log(self, hostname: str, msg: str) -> None:
        logger.debug("[%s] %s", hostname, msg)

    def _exec_agent(
        self, hostname: str, agent_args: str, timeout: float
    ) -> "tuple[Optional[dict], Optional[str]]":
        """Deploy the agent to one Pi, run it, and parse its JSON line.

        Args:
            hostname: Pi to run on.
            agent_args: Already shell-quoted arguments after the agent path
                (e.g. ``"capture 1700000000.0 /tmp/x"``).
            timeout: Seconds from now (PC clock) to wait for the agent to finish.

        Returns:
            ``(data, None)`` on success, or ``(None, error_message)``. An
            ``{"error": ...}`` result from the agent is returned as an error.
        """
        self._log(hostname, "Connecting via SSH...")
        try:
            client = self._connect(hostname)
        except Exception as exc:
            self._log(hostname, f"SSH connect FAILED: {exc}")
            return None, f"SSH connect: {exc}"
        self._log(hostname, "Connected. Deploying agent script...")

        try:
            self._deploy_agent(client)
        except Exception as exc:
            self._log(hostname, f"Deploy FAILED: {exc}")
            client.close()
            return None, f"Deploy: {exc}"
        self._log(hostname, "Agent deployed. Launching...")

        try:
            cmd = f"python3 {REMOTE_AGENT_PATH} {agent_args}"
            _, _stdout_stream, _ = client.exec_command(cmd)
            channel = _stdout_stream.channel

            stdout_buf = b""
            stderr_buf = b""
            POLL_INTERVAL = 0.05
            deadline = time.time() + timeout
            while True:
                if channel.recv_ready():
                    stdout_buf += channel.recv(4096)
                if channel.recv_stderr_ready():
                    chunk = channel.recv_stderr(4096).decode(errors="replace")
                    stderr_buf += chunk.encode()
                    for line in chunk.splitlines():
                        self._log(hostname, line)
                if channel.exit_status_ready():
                    while channel.recv_ready():
                        stdout_buf += channel.recv(4096)
                    while channel.recv_stderr_ready():
                        chunk = channel.recv_stderr(4096).decode(errors="replace")
                        stderr_buf += chunk.encode()
                        for line in chunk.splitlines():
                            self._log(hostname, line)
                    break
                if time.time() > deadline:
                    self._log(hostname, f"TIMEOUT after {timeout:.0f}s — killing channel")
                    channel.close()
                    client.close()
                    return None, f"Agent timed out after {timeout:.0f}s"
                time.sleep(POLL_INTERVAL)

            exit_code = channel.recv_exit_status()
            raw_out = stdout_buf.decode(errors="replace").strip()
            self._log(hostname, f"Agent exited (code {exit_code}). stdout: {raw_out!r:.120}")
        except Exception as exc:
            self._log(hostname, f"exec FAILED: {exc}")
            client.close()
            return None, f"exec_command: {exc}"
        finally:
            client.close()

        if not raw_out:
            raw_err = stderr_buf.decode(errors="replace").strip()
            return None, f"No stdout from agent (exit {exit_code}). stderr: {raw_err}"

        try:
            data = json.loads(raw_out)
        except json.JSONDecodeError as exc:
            return None, f"JSON parse: {exc} — got: {raw_out!r}"

        if "error" in data:
            return None, data["error"]
        return data, None

    def _run_capture_on_host(
        self, hostname: str, target_time: float, timeout: float = 60.0
    ) -> CaptureResult:
        """Open SSH connection, deploy agent, execute capture, return result.

        timeout: seconds from now (PC clock) to wait for the agent to finish.

        In a locked exposure mode the host's fixed controls are sent to the
        agent, and the frame's reported values are checked against them: a
        difference sets ``exposure_warning`` on the result.
        """
        try:
            controls = self.exposure.controls_for(hostname)
        except KeyError as exc:
            # Refuse rather than capture in auto mode.
            return CaptureResult(hostname=hostname, success=False, error=str(exc.args[0]))

        args = f"capture {target_time:.6f} {shlex.quote(REMOTE_CAPTURE_DIR)}"
        if controls is not None:
            args += f" --controls {shlex.quote(json.dumps(controls.to_dict()))}"
        data, error = self._exec_agent(hostname, args, timeout)
        if error is not None:
            return CaptureResult(hostname=hostname, success=False, error=error)

        self._log(hostname, "Capture successful.")
        result = CaptureResult(
            hostname=hostname,
            success=True,
            filename=data.get("filename"),
            target_time=data.get("target_time"),
            capture_time_mid=data.get("capture_time_mid"),
            capture_duration_ms=data.get("capture_duration_ms"),
            latency_ms=data.get("latency_ms"),
            exposure_mode=data.get("exposure_mode"),
            exposure_time_us=data.get("exposure_time_us"),
            analogue_gain=data.get("analogue_gain"),
            colour_gains=data.get("colour_gains"),
        )
        if controls is not None:
            if result.exposure_mode != "locked":
                result.exposure_warning = (
                    f"fixed exposure requested but agent reported mode {result.exposure_mode!r}"
                )
            else:
                result.exposure_warning = compare_controls(
                    controls, result.exposure_time_us, result.analogue_gain, result.colour_gains
                )
        return result

    def calibrate(self, warmup_s: Optional[float] = None) -> Dict[str, CalibrationResult]:
        """Let AE/AWB converge on every Pi and read back the values they chose.

        Meant to be run by an operator at the start of an experiment, under
        the lighting the run will use, via ``laguna-picam calibrate``. It
        captures nothing for the dataset and is never scheduled.

        Args:
            warmup_s: Seconds to run auto mode before sampling (default: the
                ``exposure.warmup_s`` setting).

        Returns:
            Results keyed by hostname. Failures are returned, not raised, so
            one dead Pi does not hide the others' results.
        """
        warmup = self.exposure.warmup_s if warmup_s is None else warmup_s
        if self._simulated:
            return {
                h: CalibrationResult(
                    hostname=h, success=False, error="simulated: no hardware to calibrate"
                )
                for h in self.hosts
            }

        results: Dict[str, CalibrationResult] = {}

        def worker(hostname: str) -> None:
            args = f"calibrate {shlex.quote(REMOTE_CAPTURE_DIR)} --warmup-s {warmup:.3f}"
            data, error = self._exec_agent(hostname, args, timeout=warmup + 60)
            if error is not None:
                results[hostname] = CalibrationResult(hostname=hostname, success=False, error=error)
                return
            try:
                controls = ExposureControls.from_dict(
                    {
                        "exposure_time_us": data["exposure_time_us"],
                        "analogue_gain": data["analogue_gain"],
                        "awb_gains": data["colour_gains"],
                    },
                    hostname,
                )
            except (KeyError, ValueError, TypeError) as exc:
                results[hostname] = CalibrationResult(
                    hostname=hostname, success=False, error=f"bad calibration from agent: {exc}"
                )
                return
            results[hostname] = CalibrationResult(
                hostname=hostname,
                success=True,
                controls=controls,
                converged=bool(data.get("converged")),
                spread=data.get("spread"),
                filename=data.get("filename"),
            )

        threads = [threading.Thread(target=worker, args=(h,), daemon=True) for h in self.hosts]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=warmup + 90)
        return {h: results[h] for h in self.hosts if h in results} | {
            h: CalibrationResult(hostname=h, success=False, error="calibration did not finish")
            for h in self.hosts
            if h not in results
        }

    def fetch_calibration_previews(
        self, results: Dict[str, CalibrationResult], output_dir: Path
    ) -> None:
        """Download each calibration preview JPEG so the operator can eyeball it.

        Sets ``local_preview`` on each result that was fetched. A failed fetch
        is reported on stdout and does not fail calibration.
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        for res in results.values():
            if not res.success or not res.filename:
                continue
            try:
                client = self._connect(res.hostname)
                sftp = client.open_sftp()
                local = output_dir / f"{res.hostname}_{Path(res.filename).name}"
                sftp.get(res.filename, str(local))
                sftp.close()
                client.close()
                res.local_preview = local
            except Exception as exc:
                print(f"  [warn] Could not fetch preview from {res.hostname}: {exc}")

    def log_exposure_settings(self) -> None:
        """Record the exposure mode (and fixed values) for each host in the event log.

        Called once at the start of a run so the log says what the frames
        that follow were captured with.
        """
        if not self.exposure.locked:
            logger.warning(
                "pi_cameras: exposure and white balance are AUTOMATIC — frames will vary "
                "with scene and camera start-up. See pi_cameras.exposure in the docs."
            )
            self.log_event("exposure_settings", level="WARNING", mode="auto")
            return
        for host in self.hosts:
            c = self.exposure.controls_for(host)
            self.log_event(
                "exposure_settings",
                host=host,
                mode=self.exposure.mode,
                exposure_time_us=c.exposure_time_us,
                analogue_gain=c.analogue_gain,
                awb_gains=",".join(f"{g:g}" for g in c.awb_gains),
            )

    def connect(self) -> bool:
        """No persistent connection needed — SSH is opened per capture."""
        return True

    def disconnect(self) -> None:
        """No persistent connection to close."""
        pass

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # Captures are one-shot SSH commands fired by the scheduler, which
    # FlumeLab halts ahead of these calls; nothing here runs persistently,
    # and a capture already on the Pis cannot be recalled (it finishes and
    # its images are still fetched). So there is nothing to quiesce. The
    # verbs exist so a halt is uniform and a missing verb in the event log
    # means a real gap.

    def pause(self) -> Optional[str]:
        """Nothing to quiesce: captures are one-shot and scheduler-driven."""
        return None

    def resume(self) -> Optional[str]:
        """Nothing to restore: pause() changed nothing."""
        return None

    def stop(self) -> Optional[str]:
        """Nothing to quiesce: captures are one-shot and scheduler-driven."""
        return None

    def estop(self) -> Optional[str]:
        """Nothing to quiesce: captures are one-shot. Never raises."""
        return None

    def get_status(self) -> Dict[str, Any]:
        """Return configured state. Does not make SSH connections."""
        return {
            "subsystem": self.subsystem_name,
            "hosts": self.hosts,
            "ssh_user": self.ssh_user,
            "ssh_key": self.ssh_key,
            "exposure_mode": self.exposure.mode,
        }

    def check_connectivity(self) -> Dict[str, bool]:
        """SSH into each host and check that it responds."""
        results: Dict[str, bool] = {}
        for hostname in self.hosts:
            try:
                client = self._connect(hostname)
                _, stdout, _ = client.exec_command("echo ok")
                ok = stdout.read().decode().strip() == "ok"
                client.close()
                results[hostname] = ok
            except Exception:
                results[hostname] = False
        return results

    def check_clock_sync(self) -> Dict[str, float]:
        """Return clock offset (Pi time − PC time) in seconds for each host."""
        offsets: Dict[str, float] = {}
        for hostname in self.hosts:
            try:
                client = self._connect(hostname)
                t_before = time.time()
                _, stdout, _ = client.exec_command("python3 -c 'import time; print(time.time())'")
                remote_time = float(stdout.read().decode().strip())
                t_after = time.time()
                client.close()
                rtt = t_after - t_before
                offsets[hostname] = remote_time - (t_before + rtt / 2)
            except Exception:
                offsets[hostname] = float("nan")
        return offsets

    def _measure_offsets(self) -> Dict[str, float]:
        """Measure and print clock offset for each host (used before trigger_capture)."""
        offsets: Dict[str, float] = {}
        for hostname in self.hosts:
            try:
                client = self._connect(hostname)
                t_before = time.time()
                _, stdout, _ = client.exec_command(
                    "python3 -c 'import time; print(time.time())'"
                )
                remote_time = float(stdout.read().decode().strip())
                t_after = time.time()
                client.close()
                rtt = t_after - t_before
                offset = remote_time - (t_before + rtt / 2)
                offsets[hostname] = offset
                logger.debug("clock sync  %s: offset=%+.3fs  RTT=%dms",
                             hostname, offset, rtt * 1000)
            except Exception as exc:
                offsets[hostname] = float("nan")
                logger.debug("clock sync  %s: FAILED (%s), assuming offset=0", hostname, exc)
        return offsets

    def trigger_capture(self, lead_time: float = DEFAULT_LEAD_TIME) -> List[CaptureResult]:
        """Trigger simultaneous capture on all cameras.

        Measures per-host clock offsets then adjusts each camera's target
        timestamp so they all fire at the same wall-clock moment.
        """
        if self._simulated:
            # No real SSH/timing work — proves the schedule fires the
            # trigger, not that the cameras would actually agree on timing.
            # Timing fields are NaN, not a fabricated spread, per
            # laguna.simulation's module docstring.
            pc_target = time.time() + lead_time
            return [
                CaptureResult(
                    hostname=host, success=True, filename="<simulated>",
                    target_time=pc_target, capture_time_mid=float("nan"),
                    capture_time_mid_pc=float("nan"), capture_duration_ms=float("nan"),
                    latency_ms=float("nan"), clock_offset_s=float("nan"),
                )
                for host in self.hosts
            ]

        logger.debug("Measuring clock offsets before trigger...")
        offsets = self._measure_offsets()

        pc_target = time.time() + lead_time
        logger.debug("PC target set: T+%.1fs from now (%.3f)", lead_time, pc_target)

        results: List[Optional[CaptureResult]] = [None] * len(self.hosts)

        def worker(idx: int, hostname: str) -> None:
            offset = offsets.get(hostname, 0.0)
            if math.isnan(offset):
                offset = 0.0
            host_target = pc_target + offset
            self._log(hostname, f"host target_time={host_target:.3f} (offset {offset:+.3f}s)")
            result = self._run_capture_on_host(hostname, host_target, timeout=lead_time + 30)
            result.clock_offset_s = offset
            if result.capture_time_mid is not None:
                result.capture_time_mid_pc = result.capture_time_mid - offset
            results[idx] = result

        threads = [
            threading.Thread(target=worker, args=(i, h), daemon=True)
            for i, h in enumerate(self.hosts)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=lead_time + 30)

        return [r for r in results if r is not None]

    def fetch_images(
        self, results: List[CaptureResult], output_dir: Path
    ) -> Dict[str, Path]:
        """SFTP-download captured images from each Pi to output_dir on the PC."""
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        local_paths: Dict[str, Path] = {}

        if self._simulated:
            # No real SFTP, no real file written — a placeholder path,
            # clearly not a real filename, so the archival log_event()
            # call below still fires and still proves the trigger->fetch
            # pipeline ran, without pretending there's an actual image.
            for result in results:
                if not result.success:
                    continue
                local = output_dir / f"{result.hostname}_simulated.jpg"
                local_paths[result.hostname] = local
                self.log_event("capture", host=result.hostname, file=str(local))
            return local_paths

        for result in results:
            if not result.success or not result.filename:
                continue
            try:
                client = self._connect(result.hostname)
                sftp = client.open_sftp()
                stem = Path(result.filename).name
                local = output_dir / f"{result.hostname}_{stem}"
                sftp.get(result.filename, str(local))
                sftp.close()
                client.close()
                local_paths[result.hostname] = local
                self.log_event(
                    "capture", host=result.hostname, file=str(local),
                    **self._exposure_fields(result),
                )
                if result.exposure_warning:
                    self.log_event(
                        "exposure_mismatch", level="WARNING", host=result.hostname,
                        file=str(local),
                        # Notes are space-separated key=value pairs.
                        detail=result.exposure_warning.replace(" ", "_"),
                    )
            except Exception as exc:
                print(f"  [warn] Could not fetch image from {result.hostname}: {exc}")

        failed = [r.hostname for r in results if not r.success]
        if failed:
            self.log_event("capture_partial_failure", level="WARNING", failed=failed)

        return local_paths

    @staticmethod
    def _exposure_fields(result: CaptureResult) -> Dict[str, Any]:
        """Event-log fields describing how a frame was exposed (empty if unknown)."""
        if result.exposure_time_us is None:
            return {}
        fields: Dict[str, Any] = {
            "exposure_mode": result.exposure_mode,
            "exposure_time_us": result.exposure_time_us,
            "analogue_gain": result.analogue_gain,
        }
        if result.colour_gains:
            fields["colour_gains"] = ",".join(f"{g:g}" for g in result.colour_gains)
        return fields

    def report_simultaneity(self, results: List[CaptureResult]) -> dict:
        """Compute timing statistics and print a summary."""
        successful = [r for r in results if r.success and r.capture_time_mid is not None]

        summary: dict = {
            "n_cameras": len(self.hosts),
            "n_successful": len(successful),
            "n_failed": len(results) - len(successful),
            "spread_ms": None,
            "assessment": None,
            "per_camera": {},
        }

        print(f"\n{'='*50}")
        print(f"Capture report — {len(successful)}/{len(results)} cameras succeeded")
        print(f"{'='*50}")

        for r in results:
            if r.success:
                offset_str = (
                    f"  clock_offset={r.clock_offset_s:+.3f}s"
                    if r.clock_offset_s is not None else ""
                )
                print(
                    f"  {r.hostname}: "
                    f"t_mid_pc={r.capture_time_mid_pc:.6f}  "
                    f"latency={r.latency_ms:+.1f}ms  "
                    f"exposure={r.capture_duration_ms:.0f}ms"
                    f"{offset_str}"
                )
                summary["per_camera"][r.hostname] = {
                    "capture_time_mid_pc": r.capture_time_mid_pc,
                    "latency_ms": r.latency_ms,
                    "capture_duration_ms": r.capture_duration_ms,
                    "clock_offset_s": r.clock_offset_s,
                }
            else:
                print(f"  {r.hostname}: FAILED — {r.error}")

        if len(successful) >= 2:
            times = [r.capture_time_mid_pc for r in successful if r.capture_time_mid_pc is not None]
            if len(times) >= 2:
                spread_ms = (max(times) - min(times)) * 1000
                assessment = assess_spread(spread_ms)
                summary["spread_ms"] = spread_ms
                summary["assessment"] = assessment
                print(f"\n  Spread between cameras: {spread_ms:.1f} ms — {assessment}")
            else:
                print("\n  Not enough successful captures with valid times to compute spread.")
        else:
            print("\n  Not enough successful captures to compute spread.")

        print(f"{'='*50}\n")
        return summary


def _resolve_passphrase(cli_value: Optional[str]) -> Optional[str]:
    """Resolve SSH passphrase from multiple sources in priority order.

    Checks sources in this order:
    1. cli_value argument (if provided)
    2. LAGUNA_SSH_PASSPHRASE environment variable
    3. ~/.config/laguna/ssh_passphrase file
    4. Interactive getpass prompt

    Args:
        cli_value: SSH passphrase from CLI (takes priority if provided).

    Returns:
        The resolved passphrase string, or None if no passphrase found or provided.
    """
    import getpass
    import os

    if cli_value:
        print("  Using passphrase from --ssh-passphrase argument.", flush=True)
        return cli_value

    env_val = os.environ.get("LAGUNA_SSH_PASSPHRASE")
    if env_val:
        print("  Using passphrase from LAGUNA_SSH_PASSPHRASE env var.", flush=True)
        return env_val

    secret_file = Path.home() / ".config" / "laguna" / "ssh_passphrase"
    if secret_file.exists():
        passphrase = secret_file.read_text().strip()
        if passphrase:
            print(f"  Using passphrase from {secret_file}", flush=True)
            return passphrase
        else:
            print(f"  Warning: {secret_file} exists but is empty.", flush=True)
    else:
        print(f"  No passphrase file found at {secret_file}.", flush=True)

    try:
        return getpass.getpass("SSH key passphrase: ") or None
    except (EOFError, KeyboardInterrupt):
        print("\n  No passphrase provided.", flush=True)
        return None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Trigger simultaneous capture on Pi cameras.")
    parser.add_argument("--cameras", nargs="+", default=DEFAULT_CAMERAS)
    parser.add_argument("--ssh-user", default=DEFAULT_SSH_USER)
    parser.add_argument("--ssh-key", default=None, help="Path to SSH private key")
    parser.add_argument("--ssh-passphrase", default=None, help="Passphrase for encrypted SSH key")
    parser.add_argument("--lead-time", type=float, default=DEFAULT_LEAD_TIME)
    parser.add_argument("--fetch-images", action="store_true")
    parser.add_argument("--output-dir", default="./captures")
    parser.add_argument("--check-sync", action="store_true", help="Check NTP clock sync before capture")
    args = parser.parse_args()

    passphrase = _resolve_passphrase(args.ssh_passphrase)

    array = CameraArray(
        hosts=args.cameras,
        ssh_user=args.ssh_user,
        ssh_key=args.ssh_key,
        ssh_passphrase=passphrase,
    )

    if args.check_sync:
        print("Checking clock synchronization...")
        offsets = array.check_clock_sync()
        for host, offset_s in offsets.items():
            print(f"  {host}: offset = {offset_s * 1000:.1f} ms")
        print()

    print(f"Triggering capture on: {args.cameras}")
    print(f"Lead time: {args.lead_time:.1f}s — target = now + {args.lead_time:.1f}s")

    results = array.trigger_capture(lead_time=args.lead_time)
    array.report_simultaneity(results)

    if args.fetch_images:
        output_dir = Path(args.output_dir)
        print(f"Fetching images to {output_dir} ...")
        local = array.fetch_images(results, output_dir)
        for host, path in local.items():
            print(f"  {host} -> {path}")
