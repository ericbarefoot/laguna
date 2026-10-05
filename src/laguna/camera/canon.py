"""Canon EOS DSLR control over USB via libgphoto2.

Vendored from Minsik's (@yukms) dualcam-timelapse
(https://github.com/yukms/dualcam-timelapse) — the connect/settings/capture
core of its ``CanonCamera`` — and reworked to fit laguna (GH issue #60). The
timelapse loop, YAML loading and positional port pairing are gone: laguna's
scheduler, ``Config`` and serial-number binding replace them.

**Cameras are identified by body serial, never by USB port.** A gphoto2
port (``usb:BBB,DDD``) embeds the kernel's device number, which changes on
every re-enumeration — a power cycle, a replug, an extender hiccup. The USB
descriptor's own serial is empty on the Rebel T7, so neither udev nor
libusb can tell two identical bodies apart; only the EOS serial read over
PTP after opening (``eosserialnumber``) can. Binding by port alone would
let a reconnect silently swap two cameras' output folders.

**Pre-flight refuses rather than warns.** gphoto2 cannot turn the mode
dial, and in any mode but M the camera quietly ignores the shutter and/or
aperture it was sent, so a run would collect a full series at the wrong
exposure without an error. The same goes for autofocus hunting between
frames and auto power-off dropping the camera off USB mid-run. Every
setting written is read back and compared.

**Every download is verified; card files are a backup.** Each local copy
is checked against the camera's copy's size and file signature. With
``capture_target="card"`` (the default) captures go to the memory card,
whose copy becomes eligible for deletion only once verified, and is freed
oldest-first only when free space falls below a reserve — so the card
holds a rolling backup of the most recent shots for as long as it has
room. ``capture_target="ram"`` is for a body with no card: the camera keeps
the shot only until it is downloaded, so a failed download is a lost
frame with no second copy anywhere.

``scripts/setup_dslr_udev.py`` lists attached cameras with their serial,
port and physical USB path, and writes matching udev rules.
"""

from __future__ import annotations

import logging
import os
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

CANON_VENDOR_ID = "04a9"

#: Config widget values the pre-flight requires. The T7 reports these
#: strings; other EOS bodies may differ, hence named constants.
MANUAL_EXPOSURE_MODE = "Manual"
MANUAL_FOCUS_MODE = "Manual"
#: laguna's capture_target option -> the camera's capturetarget value.
#:
#: Not a camera setting: libgphoto2's ptp2 driver keeps ONE capturetarget
#: for the whole host (``ptp2=capturetarget=`` in ~/.config/gphoto/settings)
#: and pushes it to whichever body is about to shoot. Two cameras with
#: different targets therefore clobber each other — the last pre-flight wins
#: for both, and a cardless body told "card" blocks ~90 s per capture before
#: failing. Read-back can't catch it (it reads the same shared value), so
#: DslrCameraSubsystem only accepts one target for every camera.
CAPTURE_TARGETS = {"card": "Memory card", "ram": "Internal RAM"}
AUTO_POWER_OFF_DISABLED = ("Off", "Disable", "Disabled", "0")

#: How long to keep draining camera events after capture() for the second
#: file of a RAW+JPEG pair. The T7 reports it within ~1 s.
_EXTRA_FILE_WAIT_MS = 3000

#: After writing settings, wait for this long a gap in camera events (up to
#: SETTLE_MAX_S) before declaring the body ready.
SETTLE_QUIET_MS = 1000
SETTLE_MAX_S = 5.0

#: Pre-flight refuses with less free card space than this. A T7 with no
#: room (no card, a full card, or the lock tab on) blocks ~90 s per capture
#: before failing, so this has to be caught before the run, not during it.
MIN_FREE_SHOTS = 10

#: At most this many card shots are freed per capture, so one capture tick
#: can't stall for long behind a deletion backlog.
_MAX_FREE_PER_CAPTURE = 10

#: (offset, bytes) file signatures the download check accepts, by
#: lowercase extension. RAW files are checked only for their container
#: header (TIFF for CR2, ISO-BMFF for CR3).
_SIGNATURES = {
    ".jpg": (0, b"\xff\xd8"),
    ".jpeg": (0, b"\xff\xd8"),
    ".cr2": (0, b"II*\x00"),
    ".cr3": (4, b"ftyp"),
}


class DslrError(RuntimeError):
    """Base class for DSLR failures the caller should act on."""


class CameraIdentityError(DslrError):
    """The camera on a port is not the one configured (serial mismatch)."""


class CameraNotReadyError(DslrError):
    """Pre-flight failed: the camera is not in a state fit to collect data."""


class CaptureVerificationError(DslrError):
    """A downloaded file does not match its card copy."""


def _gp():
    """Import gphoto2 lazily so laguna imports without the optional extra."""
    import gphoto2 as gp

    return gp


@dataclass
class Exposure:
    """Manual exposure, written to the camera and verified by read-back."""

    iso: str
    aperture: str
    shutter: str

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "Exposure":
        """Build from a config mapping; every field is required.

        Raises:
            ValueError: If any of iso/aperture/shutter is missing.
        """
        missing = [k for k in ("iso", "aperture", "shutter") if k not in cfg]
        if missing:
            raise ValueError(f"exposure is missing {missing}")
        return cls(str(cfg["iso"]), str(cfg["aperture"]), str(cfg["shutter"]))

    def as_widgets(self) -> Dict[str, str]:
        """Map to gphoto2 config widget names."""
        return {"iso": self.iso, "aperture": self.aperture, "shutterspeed": self.shutter}


@dataclass
class CaptureRecord:
    """Everything one shutter release produced."""

    camera: str
    files: List[Path] = field(default_factory=list)
    card_files: List[Tuple[str, str]] = field(default_factory=list)
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        """True when every file the shot produced is safely on disk."""
        return self.error is None and bool(self.files)


@dataclass
class DetectedCamera:
    """A Canon found on the bus by discover_cameras()."""

    model: str
    port: str
    serial: Optional[str]
    usb_path: Optional[str]


def port_from_device(device: str) -> Optional[str]:
    """Translate a ``/dev/bus/usb/BBB/DDD`` node (or a udev symlink to one).

    Returns:
        The gphoto2 port string ``usb:BBB,DDD``, or None if the path does not
        exist or isn't a USB device node.
    """
    try:
        real = Path(os.path.realpath(device))
    except OSError:
        return None
    if not real.exists() or real.parent.parent != Path("/dev/bus/usb"):
        return None
    return f"usb:{real.parent.name},{real.name}"


def usb_path_for_port(port: str, sysfs: str = "/sys/bus/usb/devices") -> Optional[str]:
    """Physical USB path (e.g. ``1-9.1``) of the device behind a gphoto2 port.

    This is what a udev rule matches with ``KERNELS==``: it stays the same
    across re-enumeration as long as the cable stays in the same hub port.
    """
    try:
        bus, dev = (int(x) for x in port.split(":", 1)[1].split(","))
    except (IndexError, ValueError):
        return None
    for entry in Path(sysfs).glob("*"):
        try:
            if (int((entry / "busnum").read_text()) == bus
                    and int((entry / "devnum").read_text()) == dev):
                return entry.name
        except (OSError, ValueError):
            continue
    return None


def detect_ports() -> List[Tuple[str, str]]:
    """Return ``(model, port)`` for every camera gphoto2 can see."""
    gp = _gp()
    found = gp.Camera.autodetect()
    return [(found.get_name(i), found.get_value(i)) for i in range(found.count())]


def open_camera(port: str):
    """Open a gphoto2 camera on a specific port.

    Returns:
        ``(camera, context)``, initialised.
    """
    gp = _gp()
    context = gp.Context()
    camera = gp.Camera()
    ports = gp.PortInfoList()
    ports.load()
    camera.set_port_info(ports[ports.lookup_path(port)])
    camera.init(context)
    return camera, context


def read_serial(camera, context) -> Optional[str]:
    """Read the EOS body serial over PTP; None if the body doesn't report one."""
    try:
        return str(camera.get_config(context).get_child_by_name("eosserialnumber").get_value())
    except Exception:
        return None


def discover_cameras() -> List[DetectedCamera]:
    """Open each attached camera just long enough to read its serial.

    Only for listing (the CLI). CanonDslr.connect() does its own scan and
    keeps the matching camera open rather than reopening it — Canon bodies
    need a settling period between PTP sessions.
    """
    out = []
    for model, port in detect_ports():
        serial = None
        try:
            camera, context = open_camera(port)
            try:
                serial = read_serial(camera, context)
            finally:
                camera.exit(context)
        except Exception as exc:
            logger.warning("Could not open %s on %s: %s", model, port, exc)
        out.append(DetectedCamera(model, port, serial, usb_path_for_port(port)))
    return out


def _check_signature(path: Path) -> bool:
    signature = _SIGNATURES.get(path.suffix.lower())
    if signature is None:
        return True
    offset, expected = signature
    with open(path, "rb") as fh:
        fh.seek(offset)
        return fh.read(len(expected)) == expected


class CanonDslr:
    """One Canon EOS body, bound by serial and guarded by a pre-flight.

    Not thread-safe on its own; DslrCameraSubsystem gives each camera its own
    thread and its own gphoto2 context, so cameras never share one.
    """

    def __init__(
        self,
        name: str,
        serial: str,
        exposure: Exposure,
        output_dir: str,
        imageformat: Optional[str] = None,
        device: Optional[str] = None,
        card_reserve_shots: int = 500,
        capture_target: str = "card",
    ) -> None:
        """Describe a camera; nothing touches USB until connect().

        Args:
            name: Laguna's name for this camera, used in filenames and logs.
            serial: EOS body serial (``python -m laguna.camera.canon``).
            exposure: Manual exposure to apply and verify.
            output_dir: Directory downloads are written to.
            imageformat: Camera ``imageformat`` value (e.g. ``"RAW + L"``);
                None leaves the camera's setting alone.
            device: Optional udev symlink to this camera's USB node, tried
                first as a hint. Its serial is still checked.
            card_reserve_shots: Free card space (in shots, as the camera
                reports it) below which verified card files are deleted.
                Unused with ``capture_target="ram"``.
            capture_target: ``"card"`` (default) keeps a backup on the
                memory card; ``"ram"`` is for a body with no card, and
                leaves the downloaded copy as the only one.

        Raises:
            ValueError: If ``capture_target`` is not ``"card"`` or ``"ram"``.
        """
        if capture_target not in CAPTURE_TARGETS:
            raise ValueError(
                f"capture_target must be one of {sorted(CAPTURE_TARGETS)}, not {capture_target!r}"
            )
        self.name = name
        self.serial = str(serial)
        self.exposure = exposure
        self.output_dir = Path(output_dir).expanduser()
        self.imageformat = imageformat
        self.device = device
        self.card_reserve_shots = card_reserve_shots
        self.capture_target = capture_target
        self.port: Optional[str] = None
        self._camera = None
        self._context = None
        #: Card files (folder, name) per shot, oldest first, whose local
        #: copies have been verified — the only card files ever deleted.
        self._verified_on_card: Deque[List[Tuple[str, str]]] = deque()

    @property
    def is_connected(self) -> bool:
        """Whether a PTP session is open."""
        return self._camera is not None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, skip_ports: Tuple[str, ...] = ()) -> None:
        """Find this camera by serial, open it, and run the pre-flight.

        Tries the ``device`` hint first, then every other attached camera.
        Cameras with a different serial are closed again untouched.

        Args:
            skip_ports: Ports already bound to other cameras — not opened.

        Raises:
            CameraIdentityError: No attached camera has this serial.
            CameraNotReadyError: Found, but the pre-flight failed.
        """
        if self.is_connected:
            return
        candidates = [p for _model, p in detect_ports() if p not in skip_ports]
        hint = port_from_device(self.device) if self.device else None
        if self.device and hint is None:
            logger.warning("[%s] device %s not present; scanning by serial", self.name, self.device)
        if hint in candidates:
            candidates.remove(hint)
            candidates.insert(0, hint)

        seen = []
        for port in candidates:
            try:
                camera, context = open_camera(port)
            except Exception as exc:
                logger.warning("[%s] could not open %s: %s", self.name, port, exc)
                seen.append(f"{port}=unopenable")
                continue
            serial = read_serial(camera, context)
            if serial == self.serial:
                self._camera, self._context, self.port = camera, context, port
                break
            seen.append(f"{port}={serial}")
            try:
                camera.exit(context)
            except Exception:
                pass
        else:
            raise CameraIdentityError(
                f"[{self.name}] no camera with serial {self.serial} attached "
                f"(saw: {', '.join(seen) or 'none'})"
            )
        if hint and self.port != hint:
            logger.warning(
                "[%s] found on %s, not at %s (%s) — check the udev rule or cabling",
                self.name, self.port, self.device, hint,
            )
        logger.info("[%s] connected on %s (serial %s)", self.name, self.port, self.serial)
        try:
            self._flush_events()
            self.preflight()
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """Close the PTP session. Never raises."""
        if self._camera is not None:
            try:
                self._camera.exit(self._context)
            except Exception as exc:
                logger.warning("[%s] error during disconnect: %s", self.name, exc)
        self._camera = None
        self._context = None
        self.port = None

    def _settle(self, max_s: float = SETTLE_MAX_S) -> None:
        """Wait until the body stops reporting events after a settings change.

        A capture fired while the body is still applying settings fails with
        ``[-110] I/O in progress``; dualcam slept a fixed 3 s here.
        """
        gp = _gp()
        deadline = time.monotonic() + max_s
        while time.monotonic() < deadline:
            try:
                kind, _ = self._camera.wait_for_event(SETTLE_QUIET_MS, self._context)
            except Exception:
                return
            if kind == gp.GP_EVENT_TIMEOUT:
                return

    def _flush_events(self, timeout_ms: int = 500) -> None:
        try:
            self._camera.wait_for_event(timeout_ms, self._context)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Settings
    # ------------------------------------------------------------------

    def _config(self):
        return self._camera.get_config(self._context)

    def read_setting(self, key: str) -> Optional[str]:
        """Read one config widget's value; None if the body lacks it."""
        try:
            return str(self._config().get_child_by_name(key).get_value())
        except Exception:
            return None

    def _write_and_verify(self, key: str, value: str) -> None:
        config = self._config()
        try:
            widget = config.get_child_by_name(key)
        except Exception as exc:
            raise CameraNotReadyError(f"[{self.name}] camera has no '{key}' setting") from exc
        if str(widget.get_value()) != value:
            try:
                choices = [widget.get_choice(i) for i in range(widget.count_choices())]
            except Exception:
                choices = None
            if choices is not None and value not in choices:
                raise CameraNotReadyError(
                    f"[{self.name}] '{value}' is not a valid {key}; choices: {choices}"
                )
            widget.set_value(value)
            self._camera.set_config(config, self._context)
        actual = self.read_setting(key)
        if actual != value:
            raise CameraNotReadyError(
                f"[{self.name}] {key} read back as {actual!r} after setting {value!r}"
            )

    def preflight(self) -> None:
        """Refuse to collect unless the camera is fit to.

        Checks the things gphoto2 cannot fix (mode dial, focus switch, auto
        power-off), then writes and reads back everything it can.

        Raises:
            CameraNotReadyError: With every failed check listed.
        """
        problems = []
        mode = self.read_setting("autoexposuremode")
        if mode != MANUAL_EXPOSURE_MODE:
            problems.append(f"mode dial is {mode!r}; set it to M")
        focus = self.read_setting("focusmode")
        if focus != MANUAL_FOCUS_MODE:
            problems.append(f"focus is {focus!r}; set the lens switch to MF")
        apo = self.read_setting("autopoweroff")
        if apo is not None and apo not in AUTO_POWER_OFF_DISABLED:
            problems.append(f"auto power-off is {apo!r}; disable it in the camera menu")
        if problems:
            raise CameraNotReadyError(f"[{self.name}] " + "; ".join(problems))

        self._write_and_verify("capturetarget", CAPTURE_TARGETS[self.capture_target])
        if self.imageformat:
            self._write_and_verify("imageformat", self.imageformat)
        # After imageformat: the count is per shot at the current format. In
        # RAM mode it is the RAM buffer's room, so it matters there too.
        shots = self.available_shots()
        if shots is None or shots < MIN_FREE_SHOTS:
            where = (
                "card — is a card inserted, not full, and its lock tab off?"
                if self.capture_target == "card" else "camera RAM buffer"
            )
            raise CameraNotReadyError(f"[{self.name}] room for {shots} shots on the {where}")
        if self.capture_target == "card" and shots < self.card_reserve_shots:
            logger.warning(
                "[%s] card has room for %d shots, below card_reserve_shots=%d: each "
                "shot will be deleted from the card as soon as it is verified, so the "
                "card keeps no backup",
                self.name, shots, self.card_reserve_shots,
            )
        for key, value in self.exposure.as_widgets().items():
            self._write_and_verify(key, value)
        self._settle()
        logger.info(
            "[%s] pre-flight ok: ISO %s, f/%s, %s s, %s, to %s",
            self.name, self.exposure.iso, self.exposure.aperture,
            self.exposure.shutter, self.read_setting("imageformat"), self.capture_target,
        )

    def available_shots(self) -> Optional[int]:
        """Free card space as the camera reports it, in shots."""
        value = self.read_setting("availableshots")
        try:
            return int(value) if value is not None else None
        except ValueError:
            return None

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def _expected_files(self) -> int:
        fmt = self.imageformat or self.read_setting("imageformat") or ""
        return 2 if "+" in fmt else 1

    def capture(self, stem: str) -> CaptureRecord:
        """Release the shutter, download and verify every file it produced.

        Files land as ``<output_dir>/<stem><ext>``. Each is written to a
        ``.part`` file, checked against the card copy, then renamed, so a
        file without ``.part`` is always a verified copy.

        Args:
            stem: Filename without extension; the caller makes it unique.

        Returns:
            A CaptureRecord; ``error`` set (never raised) when any file is
            missing or fails verification — card files are then kept.
        """
        gp = _gp()
        record = CaptureRecord(camera=self.name)
        if not self.is_connected:
            record.error = "not connected"
            return record
        expected = self._expected_files()
        try:
            first = self._camera.capture(gp.GP_CAPTURE_IMAGE, self._context)
            record.card_files.append((first.folder, first.name))
            deadline = time.monotonic() + _EXTRA_FILE_WAIT_MS / 1000
            while len(record.card_files) < expected and time.monotonic() < deadline:
                kind, data = self._camera.wait_for_event(250, self._context)
                if kind == gp.GP_EVENT_FILE_ADDED:
                    record.card_files.append((data.folder, data.name))
        except Exception as exc:
            record.error = f"capture failed: {exc}"
            return record

        if len(record.card_files) < expected:
            record.error = (
                f"expected {expected} files, camera reported {len(record.card_files)}: "
                f"{record.card_files}"
            )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        errors = []
        for folder, name in record.card_files:
            # One retry: the shutter has already fired, so retrying the
            # transfer costs no timing — and in RAM mode it's the last chance.
            for attempt in (1, 2):
                try:
                    record.files.append(self._download(folder, name, stem))
                    break
                except Exception as exc:
                    if attempt == 2:
                        errors.append(f"download of {folder}/{name} failed: {exc}")
                    else:
                        logger.warning("[%s] download of %s/%s failed, retrying: %s",
                                       self.name, folder, name, exc)
        if errors:
            record.error = "; ".join(filter(None, [record.error, *errors]))
        if self.capture_target == "ram":
            # The camera's RAM buffer holds each shot until it is deleted;
            # release it whether or not the download succeeded — there is
            # no later chance to recover it from RAM anyway.
            for folder, name in record.card_files:
                try:
                    self._camera.file_delete(folder, name, self._context)
                except Exception as exc:
                    logger.debug("[%s] RAM release of %s/%s: %s", self.name, folder, name, exc)
            if record.error is not None:
                logger.error("[%s] %s (no card: this frame is lost)", self.name, record.error)
        elif record.error is None:
            self._verified_on_card.append(list(record.card_files))
            self._free_card_space()
        else:
            logger.error("[%s] %s (card copies kept)", self.name, record.error)
        return record

    def _download(self, folder: str, name: str, stem: str) -> Path:
        gp = _gp()
        target = self.output_dir / f"{stem}{Path(name).suffix.lower()}"
        if target.exists():
            raise CaptureVerificationError(f"{target} already exists; refusing to overwrite")
        part = target.with_name(target.name + ".part")
        card_size = self._camera.file_get_info(folder, name, self._context).file.size
        # The binding's 4th positional slot is an output CameraFile, not the
        # context — passing context there fails every download.
        cam_file = gp.CameraFile()
        self._camera.file_get(folder, name, gp.GP_FILE_TYPE_NORMAL, cam_file, self._context)
        cam_file.save(str(part))
        local_size = part.stat().st_size
        if local_size != card_size:
            raise CaptureVerificationError(
                f"size mismatch: card {card_size} B, local {local_size} B ({part})"
            )
        if not _check_signature(part):
            raise CaptureVerificationError(f"{part} does not look like a {part.suffix} file")
        os.replace(part, target)
        logger.info("[%s] saved %s", self.name, target)
        return target

    def _free_card_space(self) -> None:
        """Delete oldest verified card shots while free space is below the reserve."""
        freed = 0
        while self._verified_on_card and freed < _MAX_FREE_PER_CAPTURE:
            shots = self.available_shots()
            if shots is None or shots >= self.card_reserve_shots:
                return
            for folder, name in self._verified_on_card.popleft():
                try:
                    self._camera.file_delete(folder, name, self._context)
                except Exception as exc:
                    logger.warning("[%s] could not delete %s/%s: %s", self.name, folder, name, exc)
            freed += 1
        shots = self.available_shots()
        if shots is not None and shots < self.card_reserve_shots and not self._verified_on_card:
            logger.warning(
                "[%s] card has room for %d shots and nothing laguna may delete — "
                "free space on the card before it fills",
                self.name, shots,
            )


# ----------------------------------------------------------------------
# udev
# ----------------------------------------------------------------------

def udev_rule(name: str, usb_path: str) -> str:
    """A udev rule giving the camera at physical ``usb_path`` the symlink /dev/<name>.

    Matches the hub port, not the camera: the T7's USB serial is empty, so
    this is the only stable thing udev can see. gvfs is left to
    scripts/toggle-canon-gvfs.sh, which owns that on/off state.
    """
    return (
        f'SUBSYSTEM=="usb", ENV{{DEVTYPE}}=="usb_device", '
        f'ATTR{{idVendor}}=="{CANON_VENDOR_ID}", KERNELS=="{usb_path}", '
        f'SYMLINK+="{name}"'
    )
