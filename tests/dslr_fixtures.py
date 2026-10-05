"""A fake ``gphoto2`` module backed by simulated Canon bodies on a fake bus.

Install with ``install_fake_gphoto2(monkeypatch, bus)``; laguna.camera.canon
imports gphoto2 lazily, so it picks the fake up. A ``FakeBus`` maps gphoto2
port strings to ``FakeBody`` instances — move a body to a new port to model
re-enumeration, or delete it to model a camera dropping off USB.
"""

import sys
import threading
import time
import types
from typing import Dict, List, Optional, Tuple

GP_CAPTURE_IMAGE = 0
GP_EVENT_TIMEOUT = 1
GP_EVENT_FILE_ADDED = 2
GP_FILE_TYPE_NORMAL = 1

JPEG_BYTES = b"\xff\xd8" + b"\x00" * 98
CR2_BYTES = b"II*\x00" + b"\x00" * 196


class GPhoto2Error(Exception):
    """Stands in for gphoto2.GPhoto2Error."""


class FakeBody:
    """One simulated camera: settings, a card, and failure switches."""

    def __init__(self, serial: str, **settings: str) -> None:
        """Create a body ready to pass pre-flight unless settings override."""
        self.serial = serial
        self.settings: Dict[str, str] = {
            "eosserialnumber": serial,
            "autoexposuremode": "Manual",
            "focusmode": "Manual",
            "autopoweroff": "Off",
            "capturetarget": "Internal RAM",
            "imageformat": "L",
            "iso": "800",
            "aperture": "5.6",
            "shutterspeed": "1/60",
            "availableshots": "5000",
            "datetimeutc": "946684800",   # 2000-01-01: a reset T7 clock
            "syncdatetime": "0",
        }
        self.settings.update(settings)
        self.choices: Dict[str, List[str]] = {
            "imageformat": ["L", "RAW", "RAW + L"],
            "iso": ["100", "200", "400", "800", "1600"],
        }
        #: Keys whose writes the body silently ignores (e.g. shutter in Av).
        self.sticky: set = set()
        self.card: Dict[Tuple[str, str], bytes] = {}
        self.deleted: List[Tuple[str, str]] = []
        self.shots = 0
        self.fail_capture = False
        #: When set, shoot() signals ``capture_started`` then blocks until
        #: this event is set — models libgphoto2 hanging mid-capture.
        self.capture_gate: Optional[threading.Event] = None
        self.capture_started = threading.Event()
        self.truncate_download = False
        self.open_sessions = 0

    def shoot(self) -> List[Tuple[str, str]]:
        """Write this shot's files to the card; return their (folder, name)."""
        if self.capture_gate is not None:
            self.capture_started.set()
            self.capture_gate.wait(timeout=5)
        if self.fail_capture:
            raise GPhoto2Error("[-110] I/O in progress")
        self.shots += 1
        folder = "/store_00020001/DCIM/100CANON"
        files = []
        fmt = self.settings["imageformat"]
        if "RAW" in fmt:
            files.append(((folder, f"IMG_{self.shots:04d}.CR2"), CR2_BYTES))
        if "L" in fmt:
            files.append(((folder, f"IMG_{self.shots:04d}.JPG"), JPEG_BYTES))
        for key, data in files:
            self.card[key] = data
        self.settings["availableshots"] = str(int(self.settings["availableshots"]) - 1)
        return [key for key, _ in files]


class FakeBus:
    """Port string -> body."""

    def __init__(self, bodies: Optional[Dict[str, FakeBody]] = None) -> None:
        """Start with the given attached bodies."""
        self.ports: Dict[str, FakeBody] = dict(bodies or {})

    def move(self, old: str, new: str) -> None:
        """Re-enumerate a body onto a new port."""
        self.ports[new] = self.ports.pop(old)


def install_fake_gphoto2(monkeypatch, bus: FakeBus) -> types.ModuleType:
    """Put a fake gphoto2 module backed by ``bus`` into sys.modules."""
    gp = types.ModuleType("gphoto2")
    gp.GP_CAPTURE_IMAGE = GP_CAPTURE_IMAGE
    gp.GP_EVENT_TIMEOUT = GP_EVENT_TIMEOUT
    gp.GP_EVENT_FILE_ADDED = GP_EVENT_FILE_ADDED
    gp.GP_FILE_TYPE_NORMAL = GP_FILE_TYPE_NORMAL
    gp.GPhoto2Error = GPhoto2Error

    class Context:
        pass

    class CameraList:
        def __init__(self, items):
            self._items = items

        def count(self):
            return len(self._items)

        def get_name(self, i):
            return self._items[i][0]

        def get_value(self, i):
            return self._items[i][1]

    class PortInfoList:
        def load(self):
            self._ports = list(bus.ports)

        def lookup_path(self, port):
            return self._ports.index(port)

        def __getitem__(self, i):
            return self._ports[i]

    class Widget:
        def __init__(self, body, key):
            self._body, self._key = body, key
            self._value = body.settings[key]

        def get_value(self):
            return self._value

        def set_value(self, value):
            self._value = value

        def count_choices(self):
            if self._key not in self._body.choices:
                raise GPhoto2Error("not a radio widget")
            return len(self._body.choices[self._key])

        def get_choice(self, i):
            return self._body.choices[self._key][i]

    class Config:
        def __init__(self, body):
            self._body = body
            self._widgets = {}

        def get_child_by_name(self, key):
            if key not in self._body.settings:
                raise GPhoto2Error(f"[-2] no widget {key}")
            return self._widgets.setdefault(key, Widget(self._body, key))

    class FilePath:
        def __init__(self, folder, name):
            self.folder, self.name = folder, name

    class CameraFile:
        def __init__(self):
            self._data = b""

        def save(self, path):
            with open(path, "wb") as fh:
                fh.write(self._data)

    class Camera:
        @staticmethod
        def autodetect():
            return CameraList([("Canon EOS Rebel T7", p) for p in bus.ports])

        def __init__(self):
            self._port = None
            self._body = None
            self._events: List[Tuple[str, str]] = []

        def set_port_info(self, port):
            self._port = port

        def init(self, context=None):
            if self._port not in bus.ports:
                raise GPhoto2Error("[-105] Unknown model")
            self._body = bus.ports[self._port]
            self._body.open_sessions += 1

        def exit(self, context=None):
            if self._body is not None:
                self._body.open_sessions -= 1
            self._body = None

        def _live(self):
            if self._body is None or bus.ports.get(self._port) is not self._body:
                raise GPhoto2Error("[-7] I/O problem")
            return self._body

        def get_config(self, context=None):
            return Config(self._live())

        def set_config(self, config, context=None):
            body = self._live()
            for key, widget in config._widgets.items():
                if key not in body.sticky:
                    body.settings[key] = widget.get_value()
            # Like the T7: syncdatetime is a trigger that sets the clock to
            # the host's time, not a value that sticks.
            if str(body.settings.get("syncdatetime")) == "1" and "syncdatetime" not in body.sticky:
                body.settings["datetimeutc"] = str(int(time.time()))
                body.settings["syncdatetime"] = "0"

        def wait_for_event(self, timeout, context=None):
            self._live()
            if self._events:
                folder, name = self._events.pop(0)
                return GP_EVENT_FILE_ADDED, FilePath(folder, name)
            return GP_EVENT_TIMEOUT, None

        def capture(self, kind, context=None):
            files = self._live().shoot()
            self._events.extend(files[1:])
            return FilePath(*files[0])

        def file_get_info(self, folder, name, context=None):
            data = self._live().card[(folder, name)]
            return types.SimpleNamespace(file=types.SimpleNamespace(size=len(data)))

        def file_get(self, folder, name, kind, camera_file, context=None):
            # Mirrors the real binding: the 4th slot must be a CameraFile.
            if not isinstance(camera_file, CameraFile):
                raise TypeError("in method 'Camera_file_get', argument 5 of type 'CameraFile *'")
            body = self._live()
            data = body.card[(folder, name)]
            if body.truncate_download:
                data = data[: len(data) // 2]
            camera_file._data = data
            return camera_file

        def file_delete(self, folder, name, context=None):
            body = self._live()
            del body.card[(folder, name)]
            body.deleted.append((folder, name))
            body.settings["availableshots"] = str(int(body.settings["availableshots"]) + 1)

    gp.CameraFile = CameraFile
    gp.Context = Context
    gp.Camera = Camera
    gp.PortInfoList = PortInfoList
    monkeypatch.setitem(sys.modules, "gphoto2", gp)
    # No real gvfs to release under test.
    monkeypatch.setattr("laguna.camera.gvfs.release_gphoto_usb", lambda: 0)
    return gp
