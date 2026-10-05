"""DSLR camera subsystem: any number of Canon EOS bodies, bound by serial.

Wraps laguna.camera.canon (vendored from dualcam-timelapse, see that
module's docstring) as a FlumeLab subsystem. Each configured camera is found
by its body serial on every connect and resume — never by a remembered USB
port — and refuses to connect unless its pre-flight passes.

**A failed capture is a stop-and-fix event.** capture_all() never retries:
a frame missed at time t can't be retaken at t + 5 s and still mean the same
thing. It reports each camera's outcome, and laguna.experiment.runner
escalates any failure to a lab-wide pause so a human looks before more is
lost. Card copies of a shot whose download failed are kept, and the event
log names them, so the image itself is usually recoverable from the card.
"""

import logging
import threading
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from ..subsystem_logging import SubsystemLogging
from .canon import CanonDslr, CaptureRecord, DslrError, Exposure

if TYPE_CHECKING:
    from ..config import Config

logger = logging.getLogger(__name__)


class DslrCameraSubsystem(SubsystemLogging):
    """Canon DSLRs captured in parallel on one trigger.

    Note: gphoto2 and libgphoto2 are Linux-specific; install the ``dslr``
    extra (``pip install laguna[dslr]``).

    Example::

        lab.add("dslr_cameras")      # from the config's dslr_cameras: section
        lab.connect_all()
        records = lab.dslr_cameras.capture_all()
    """

    subsystem_name = "dslr_cameras"

    def __init__(
        self,
        cameras: Optional[Dict[str, Dict[str, Any]]] = None,
        card_reserve_shots: int = 500,
        capture_target: str = "card",
        log_level: str = "INFO",
        event_log_verbosity: str = "INFO",
        simulated: bool = False,
    ) -> None:
        """Initialize from per-camera config mappings.

        Args:
            cameras: ``{name: {serial, exposure, output_dir, imageformat?,
                device?}}``. ``serial`` and ``exposure`` are required unless
                simulated.
            card_reserve_shots: Default free-card reserve for every camera;
                a camera's own ``card_reserve_shots`` overrides it.
            capture_target: For every camera: ``"card"`` keeps a backup
                copy on the memory card, ``"ram"`` is for bodies with no card
                (the download is then the only copy). Section-wide only —
                libgphoto2 holds one target for the whole host; see
                laguna.camera.canon.CAPTURE_TARGETS.
            log_level: Logging level; see laguna.subsystem_logging.
            event_log_verbosity: Event log verbosity; see
                laguna.subsystem_logging.
            simulated: Skip gphoto2/USB entirely — connect()/capture_all()
                succeed without touching real cameras, returning placeholder
                filenames instead of real images.

        Raises:
            ValueError: If a real (non-simulated) camera lacks serial or a
                complete exposure, or sets its own ``capture_target``.
        """
        self.log_level = log_level
        self.event_log_verbosity = event_log_verbosity
        self._simulated = simulated
        self._is_connected = False
        self._lock = threading.Lock()
        #: Set by stop()/estop() when they find a capture in flight.
        self._disconnect_when_idle = False
        self.cameras: Dict[str, CanonDslr] = {}
        self._camera_names: List[str] = list(cameras or {})
        #: Cameras whose output_dir was set on purpose; set_output_root()
        #: leaves these alone (GH #43).
        self._explicit_output = {n for n, c in (cameras or {}).items() if (c or {}).get("output_dir")}
        if simulated:
            return
        for name, cfg in (cameras or {}).items():
            if "capture_target" in cfg:
                raise ValueError(
                    f"dslr_cameras.{name}.capture_target: set capture_target once for "
                    "the whole dslr_cameras section — libgphoto2 keeps a single target "
                    "for every camera on the host, so per-camera values clobber each other"
                )
            if not cfg.get("serial"):
                raise ValueError(
                    f"dslr_cameras.{name} needs a serial — run "
                    "`python -m laguna.camera.canon` to list attached cameras"
                )
            self.cameras[name] = CanonDslr(
                name=name,
                serial=cfg["serial"],
                exposure=Exposure.from_config(cfg.get("exposure") or {}),
                output_dir=cfg.get("output_dir", f"./captures/{name}"),
                imageformat=cfg.get("imageformat"),
                device=cfg.get("device"),
                card_reserve_shots=int(cfg.get("card_reserve_shots", card_reserve_shots)),
                capture_target=capture_target,
            )

    @classmethod
    def from_config(cls, config: "Config") -> "DslrCameraSubsystem":
        """Build from the lab's Config: its ``dslr_cameras:`` section.

        Relative ``output_dir`` values resolve against the config file's
        directory, so a config means the same thing from any working dir.

        Raises:
            ValueError: If config.config_file is None — there is no YAML
                location to resolve relative paths against.
        """
        dslr_cfg = config.get("dslr_cameras")
        if config.config_file is None:
            raise ValueError(
                "DslrCameraSubsystem.from_config() needs config.config_file "
                "to resolve output_dir paths — build the Config from a YAML "
                "file (FlumeLab(config_file=...) or Config(config_file=...))."
            )
        main_dir = Path(config.config_file).resolve().parent
        cameras: Dict[str, Dict[str, Any]] = {}
        for name, cam_cfg in (dslr_cfg.get("cameras") or {}).items():
            resolved = dict(cam_cfg or {})
            if "output_dir" in resolved:
                out = Path(resolved["output_dir"]).expanduser()
                if not out.is_absolute():
                    out = (main_dir / out).resolve()
                resolved["output_dir"] = str(out)
            cameras[name] = resolved
        return cls(
            cameras=cameras,
            card_reserve_shots=int(dslr_cfg.get("card_reserve_shots", 500)),
            capture_target=dslr_cfg.get("capture_target", "card"),
            log_level=dslr_cfg.get("log_level", "INFO"),
            event_log_verbosity=dslr_cfg.get("event_log_verbosity", "INFO"),
            simulated=dslr_cfg.get("simulated", False),
        )

    @property
    def camera_names(self) -> List[str]:
        """Configured camera names, in config order."""
        return list(self._camera_names)

    def set_output_root(self, root: Path) -> None:
        """Send files to ``root/<camera name>`` (run directories).

        Only for cameras with no ``output_dir`` of their own: an explicit
        one (a separate drive, say) is a deliberate choice and wins.
        """
        for name, cam in self.cameras.items():
            if name in self._explicit_output:
                logger.info("[%s] keeping configured output_dir %s", name, cam.output_dir)
            else:
                cam.output_dir = Path(root) / name

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Bind every configured camera by serial and run its pre-flight.

        All-or-nothing: a run with a camera missing is not the run that was
        planned, so one failure leaves every camera disconnected.

        Returns:
            True when every camera is connected and passed pre-flight.
        """
        if self._simulated:
            self._is_connected = True
            return True
        from .gvfs import release_gphoto_usb

        released = release_gphoto_usb()
        if released:
            logger.info("Released %d gvfsd-gphoto2 process(es)", released)
        problems = self._connect_each()
        if problems:
            for problem in problems:
                logger.error("%s", problem)
            self.log_event("connect", level="ERROR", result="error", problems="; ".join(problems))
            self.disconnect()
            return False
        self._is_connected = True
        self.log_event(
            "connect",
            cameras=",".join(f"{n}@{c.port}" for n, c in self.cameras.items()),
        )
        return True

    def _connect_each(self) -> List[str]:
        problems = []
        for name, cam in self.cameras.items():
            bound = tuple(c.port for c in self.cameras.values() if c.port)
            try:
                cam.connect(skip_ports=bound)
            except DslrError as exc:
                problems.append(str(exc))
            except Exception as exc:
                problems.append(f"[{name}] unexpected connect error: {exc}")
        return problems

    def disconnect(self) -> None:
        """Close every camera's PTP session. Never raises."""
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False

    # ------------------------------------------------------------------
    # Safety verbs (see laguna.safety)
    # ------------------------------------------------------------------
    # Nothing here is hazardous, but FlumeLab calls each subsystem's verb in
    # turn, so a verb that blocks delays every subsystem after it — the
    # gantry included. A capture can block for 90 s inside libgphoto2 (a
    # full card does exactly that), so no verb ever waits on one. Nor may a
    # verb close a camera another thread is mid-call on: libgphoto2 is not
    # safe for that and a crash would take the whole process — and every
    # later subsystem's halt — down with it. Instead the disconnect is
    # deferred to the end of the in-flight capture.

    def pause(self) -> Optional[str]:
        """Return at once; cameras stay connected. Never blocks."""
        if self._lock.locked():
            return "a DSLR capture was in flight; it will finish (or fail) on its own"
        return None

    def resume(self) -> Optional[str]:
        """Re-bind every camera by serial and re-run its pre-flight."""
        if self._simulated:
            return None
        if not self._lock.acquire(timeout=0):
            return "a DSLR capture is still in flight — resume again once it finishes"
        try:
            self._disconnect_when_idle = False
            self.disconnect()
            if not self.connect():
                return "DSLR cameras failed to reconnect — see the connect event for why"
        finally:
            self._lock.release()
        return None

    def stop(self) -> Optional[str]:
        """Disconnect now, or as soon as the in-flight capture ends. Never blocks."""
        return self._halt()

    def estop(self) -> Optional[str]:
        """Same as stop(): there is no harder halt for a camera. Never raises or blocks."""
        return self._halt()

    def _halt(self) -> Optional[str]:
        try:
            if not self._lock.acquire(timeout=0):
                self._disconnect_when_idle = True
                return "a DSLR capture was in flight; cameras disconnect when it ends"
            try:
                self.disconnect()
            finally:
                self._lock.release()
        except Exception as exc:
            return f"DSLR disconnect failed: {exc}"
        return None

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def capture_all(self, runtime_s: Optional[float] = None) -> Dict[str, CaptureRecord]:
        """Fire every camera at once and wait for every download.

        Args:
            runtime_s: Experiment runtime of this trigger, embedded in the
                filenames. Defaults to the attached clock's elapsed time.

        Returns:
            ``{camera name: CaptureRecord}`` for every configured camera.
            A record with ``ok`` False means data was missed; the caller
            decides whether to escalate (the runner always does).
        """
        if runtime_s is None and self._clock is not None:
            runtime_s = self._clock.elapsed()
        stamp = _stamp(runtime_s)

        if self._simulated:
            records = {}
            for name in self._camera_names:
                record = CaptureRecord(camera=name)
                if self._is_connected:
                    record.files = [Path(f"<simulated>/{name}_{stamp}.jpg")]
                else:
                    record.error = "not connected"
                records[name] = record
            self._log_records(records)
            return records

        # Never queue behind a capture that is still running: the frame this
        # trigger stands for can't be taken late, so report it missed now.
        if not self._lock.acquire(timeout=0):
            records = {
                name: CaptureRecord(camera=name, error="previous capture still in progress")
                for name in self._camera_names
            }
            self._log_records(records)
            return records
        try:
            records: Dict[str, CaptureRecord] = {}
            threads = [
                threading.Thread(
                    target=lambda n=name, c=cam: records.__setitem__(n, c.capture(f"{n}_{stamp}")),
                    name=f"dslr-{name}",
                    daemon=True,
                )
                for name, cam in self.cameras.items()
            ]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            if self._disconnect_when_idle:
                self._disconnect_when_idle = False
                self.disconnect()
        finally:
            self._lock.release()
        self._log_records(records)
        return records

    def _log_records(self, records: Dict[str, CaptureRecord]) -> None:
        for name, record in records.items():
            if record.ok:
                # file= is the JPEG when there is one: FlumeLab's end-of-run
                # summary collects capture paths from that key alone.
                files = sorted(record.files, key=lambda p: p.suffix.lower() not in (".jpg", ".jpeg"))
                extra = {"raw": ",".join(str(p) for p in files[1:])} if len(files) > 1 else {}
                self.log_event("capture", camera=name, file=str(files[0]), **extra)
            else:
                card = ",".join(f"{f}/{n}" for f, n in record.card_files) or "none"
                self.log_event("capture_failed", level="ERROR", result="error",
                               camera=name, reason=record.error, card_files=card)

    def get_status(self) -> Dict[str, Any]:
        """Return a status snapshot. Reads nothing from the cameras."""
        return {
            "subsystem": self.subsystem_name,
            "is_connected": self._is_connected,
            "simulated": self._simulated,
            "cameras": {
                name: {"serial": cam.serial, "port": cam.port, "connected": cam.is_connected}
                for name, cam in self.cameras.items()
            },
        }


def _stamp(runtime_s: Optional[float]) -> str:
    """Filename stamp: experiment runtime plus wall time to the millisecond.

    Runtime ties the file to the event log; wall time keeps names unique
    across runs and readable without the log.
    """
    wall = datetime.now().strftime("%Y%m%dT%H%M%S.%f")[:-3]
    if runtime_s is None:
        return wall
    return f"t{runtime_s:09.1f}_{wall}"
