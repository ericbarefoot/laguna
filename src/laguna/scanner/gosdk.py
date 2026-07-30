"""Minimal ctypes binding for the LMI GoSdk C library.

This module is deliberately policy-free: it loads ``libkApi.so`` /
``libGoSdk.so``, declares the constants, structs and function prototypes
laguna actually uses, and raises on non-``kOK`` returns. Everything about
*how* to run a scan lives in :mod:`laguna.scanner.gocator`.

The vendor SDK ships no Python bindings (C/C#/VB.NET samples only), and no
prebuilt x86_64 shared library — see ``scripts/build_gosdk.sh``. Function
signatures below are transcribed from the SDK headers; each group cites the
header it came from so they can be re-checked against a future SDK version.
SDK reference notes: ``docs/reference/gocator/GOCATOR_SDK_NOTES.md``.

Polling (``GoSystem_ReceiveData``) is used rather than GoSdk's async
data-handler callback: the callback fires on a thread the SDK spawns itself,
which would need careful GIL marshaling through a ctypes ``CFUNCTYPE``, and
buys nothing here since a scan is a discrete bracketed event.
"""

from __future__ import annotations

import ctypes
import os
from ctypes import (
    POINTER,
    Structure,
    byref,
    c_char_p,
    c_double,
    c_int16,
    c_int32,
    c_int64,
    c_size_t,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path
from typing import Any, List, Optional

# ----------------------------------------------------------------------
# kApi scalar type aliases (Platform/kApi/kApi/kApiDef.h)
# ----------------------------------------------------------------------

kStatus = c_int32
kBool = c_int32
kSize = c_size_t
k16s = c_int16
k32s = c_int32
k32u = c_uint32
k64s = c_int64
k64u = c_uint64
k64f = c_double
kObject = c_void_p  # every GoSdk handle (GoSystem, GoSensor, GoSetup, ...)

kOK = 1  # kApiDef.h:543 — note kERROR is 0, so "truthy" is NOT success
kERROR = 0
kERROR_TIMEOUT = -993  # kApiDef.h:523

kTRUE = 1
kFALSE = 0

kIP_VERSION_4 = 4  # Io/kNetwork.h:28

# ----------------------------------------------------------------------
# GoSdk enums (Gocator/GoSdk/GoSdk/GoSdkDef.h)
# ----------------------------------------------------------------------

GO_ROLE_MAIN = 0  # GoSdkDef.h:202

# GoMode
GO_MODE_SURFACE = 3  # GoSdkDef.h:281

# GoTrigger — GoSdkDef.h:297-303
GO_TRIGGER_TIME = 0
GO_TRIGGER_ENCODER = 1
GO_TRIGGER_INPUT = 2
GO_TRIGGER_SOFTWARE = 3

# GoSurfaceGenerationType — GoSdkDef.h:2627-2633
GO_SURFACE_GENERATION_TYPE_CONTINUOUS = 0
GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH = 1
GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH = 2
GO_SURFACE_GENERATION_TYPE_ROTATIONAL = 3

# GoSurfaceGenerationStartTrigger — GoSdkDef.h:2644-2653
GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL = 0
GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL = 1
GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE = 2

# GoDataMessageType — GoSdkDef.h:1975-2024
GO_DATA_MESSAGE_TYPE_STAMP = 0
GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD = 5
GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE = 7
GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE = 8
GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY = 9
GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD = 28

#: Gocator transmits ranges as 16-bit signed ints; 0x8000 means "no data"
#: (occlusion, no laser return). samples/C/ReceiveSurface/src/ReceiveSurface.c:20
INVALID_RANGE_16BIT = -32768


class GoSdkError(RuntimeError):
    """A GoSdk call returned a non-kOK status."""

    def __init__(self, function: str, status: int):
        self.function = function
        self.status = status
        super().__init__(f"{function} failed with kStatus={status}")


class GoSdkTimeout(GoSdkError):
    """A GoSdk call returned kERROR_TIMEOUT (-993)."""


# ----------------------------------------------------------------------
# Structs
# ----------------------------------------------------------------------


class kIpAddress(Structure):
    """Io/kNetwork.h:88-92."""

    _fields_ = [
        ("version", k32s),
        ("address", ctypes.c_ubyte * 16),
    ]


class GoStamp(Structure):
    """Messages/GoDataTypes.h:123-144.

    ``timestamp`` is in internal units approximating microseconds, where
    true µs = timestamp / 1.024 (per the header's own comment).
    """

    _fields_ = [
        ("frameIndex", k64u),
        ("timestamp", k64u),
        ("encoder", k64s),
        ("encoderAtZ", k64s),
        ("status", k64u),
        ("id", k32u),
        ("reserved32u", k32u),
        ("reserved64u", k64u),
        ("ptpTime", k64u),
    ]


class kPoint3d16s(Structure):
    """Platform/kApi/kApi/kApiDef.h:1370-1375."""

    _fields_ = [("x", k16s), ("y", k16s), ("z", k16s)]


# ----------------------------------------------------------------------
# Library discovery
# ----------------------------------------------------------------------

#: Relative to a GO_SDK root, where build_gosdk.sh puts a Release build.
_DEFAULT_LIB_SUBDIR = "lib/linux_x64"

_DEFAULT_SDK_DIRS = (
    "~/Downloads/14400-6.5.2.5_SOFTWARE_GO_SDK/GO_SDK",
    "/opt/GO_SDK",
    "/usr/local/GO_SDK",
)


def find_lib_dir(explicit: Optional[str] = None) -> Path:
    """Locate the directory holding ``libGoSdk.so`` and ``libkApi.so``.

    Resolution order: `explicit` argument, ``$LAGUNA_GOSDK_LIB_DIR``,
    ``$LAGUNA_GOSDK_DIR/lib/linux_x64``, then a few conventional locations.

    Raises:
        FileNotFoundError: If no candidate directory contains both libraries,
            with the searched paths and a pointer to the build script.
    """
    candidates: List[Path] = []

    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("LAGUNA_GOSDK_LIB_DIR"):
        candidates.append(Path(os.environ["LAGUNA_GOSDK_LIB_DIR"]).expanduser())
    if os.environ.get("LAGUNA_GOSDK_DIR"):
        candidates.append(
            Path(os.environ["LAGUNA_GOSDK_DIR"]).expanduser() / _DEFAULT_LIB_SUBDIR
        )
    for root in _DEFAULT_SDK_DIRS:
        candidates.append(Path(root).expanduser() / _DEFAULT_LIB_SUBDIR)

    for candidate in candidates:
        if (candidate / "libGoSdk.so").exists() and (candidate / "libkApi.so").exists():
            return candidate

    searched = "\n  ".join(str(c) for c in candidates)
    raise FileNotFoundError(
        "Could not find libGoSdk.so + libkApi.so. Searched:\n  "
        f"{searched}\n"
        "The vendor SDK ships no prebuilt x86_64 libraries — build them with\n"
        "  scripts/build_gosdk.sh\n"
        "then set LAGUNA_GOSDK_LIB_DIR (or the gocator config's sdk_lib_dir)."
    )


class GoSdkLib:
    """Loaded GoSdk shared libraries with prototypes applied.

    Args:
        lib_dir: Directory containing libGoSdk.so/libkApi.so. Auto-discovered
            via :func:`find_lib_dir` when omitted.
    """

    def __init__(self, lib_dir: Optional[str] = None):
        self.lib_dir = find_lib_dir(lib_dir)
        # kApi must be RTLD_GLOBAL: libGoSdk.so resolves kApi symbols at load.
        self.kapi = ctypes.CDLL(
            str(self.lib_dir / "libkApi.so"), mode=ctypes.RTLD_GLOBAL
        )
        self.go = ctypes.CDLL(
            str(self.lib_dir / "libGoSdk.so"), mode=ctypes.RTLD_GLOBAL
        )
        self._declare()

    # ------------------------------------------------------------------
    # Prototype declarations
    # ------------------------------------------------------------------

    def _declare(self) -> None:
        go, kapi = self.go, self.kapi

        def sig(lib: Any, name: str, restype: Any, *argtypes: Any) -> None:
            """Apply a prototype, tolerating symbols absent from this SDK build."""
            try:
                fn = getattr(lib, name)
            except AttributeError:  # pragma: no cover - SDK version drift
                return
            fn.restype = restype
            fn.argtypes = list(argtypes)

        # --- kApi: Io/kNetwork.h ---
        sig(kapi, "kIpAddress_Parse", kStatus, POINTER(kIpAddress), c_char_p)

        # --- GoSdkLib.h / GoSdkDef.h ---
        sig(go, "GoSdk_Construct", kStatus, POINTER(kObject))
        sig(go, "GoDestroy", kStatus, kObject)

        # --- GoSystem.h ---
        sig(go, "GoSystem_Construct", kStatus, POINTER(kObject), kObject)
        sig(
            go,
            "GoSystem_FindSensorByIpAddress",
            kStatus,
            kObject,
            POINTER(kIpAddress),
            POINTER(kObject),
        )
        sig(go, "GoSystem_EnableData", kStatus, kObject, kBool)
        sig(go, "GoSystem_ReceiveData", kStatus, kObject, POINTER(kObject), k64u)
        sig(go, "GoSystem_Start", kStatus, kObject)
        sig(go, "GoSystem_Stop", kStatus, kObject)
        sig(go, "GoSystem_SetDataCapacity", kStatus, kObject, kSize)
        sig(go, "GoSystem_ClearData", kStatus, kObject)

        # --- GoSensor.h ---
        sig(go, "GoSensor_Connect", kStatus, kObject)
        sig(go, "GoSensor_Disconnect", kStatus, kObject)
        sig(go, "GoSensor_IsConnected", kBool, kObject)
        sig(go, "GoSensor_Flush", kStatus, kObject)
        sig(go, "GoSensor_Trigger", kStatus, kObject)
        sig(go, "GoSensor_Setup", kObject, kObject)
        sig(go, "GoSensor_Transform", kObject, kObject)
        sig(go, "GoSensor_Id", k32u, kObject)
        sig(go, "GoSensor_State", k32s, kObject)

        # --- GoSetup.h ---
        sig(go, "GoSetup_SetScanMode", kStatus, kObject, k32s)
        sig(go, "GoSetup_ScanMode", k32s, kObject)
        sig(go, "GoSetup_SetTriggerSource", kStatus, kObject, k32s)
        sig(go, "GoSetup_TriggerSource", k32s, kObject)
        sig(go, "GoSetup_EnableMaxFrameRate", kStatus, kObject, kBool)
        sig(go, "GoSetup_MaxFrameRateEnabled", kBool, kObject)
        sig(go, "GoSetup_SetFrameRate", kStatus, kObject, k64f)
        sig(go, "GoSetup_FrameRate", k64f, kObject)
        sig(go, "GoSetup_FrameRateLimitMin", k64f, kObject)
        sig(go, "GoSetup_FrameRateLimitMax", k64f, kObject)
        sig(go, "GoSetup_SetExposure", kStatus, kObject, k32s, k64f)
        sig(go, "GoSetup_Exposure", k64f, kObject, k32s)
        sig(go, "GoSetup_EnableUniformSpacing", kStatus, kObject, kBool)
        sig(go, "GoSetup_UniformSpacingEnabled", kBool, kObject)
        sig(go, "GoSetup_SurfaceGeneration", kObject, kObject)

        # --- GoTransform.h (travel speed for encoderless scanning) ---
        sig(go, "GoTransform_SetSpeed", kStatus, kObject, k64f)
        sig(go, "GoTransform_Speed", k64f, kObject)
        sig(go, "GoTransform_SetEncoderResolution", kStatus, kObject, k64f)
        sig(go, "GoTransform_EncoderResolution", k64f, kObject)

        # --- GoSurfaceGeneration.h ---
        sig(go, "GoSurfaceGeneration_SetGenerationType", kStatus, kObject, k32s)
        sig(go, "GoSurfaceGeneration_GenerationType", k32s, kObject)
        sig(
            go,
            "GoSurfaceGenerationFixedLength_SetStartTrigger",
            kStatus,
            kObject,
            k32s,
        )
        sig(go, "GoSurfaceGenerationFixedLength_StartTrigger", k32s, kObject)
        sig(go, "GoSurfaceGenerationFixedLength_SetLength", kStatus, kObject, k64f)
        sig(go, "GoSurfaceGenerationFixedLength_Length", k64f, kObject)
        sig(go, "GoSurfaceGenerationFixedLength_LengthLimitMin", k64f, kObject)
        sig(go, "GoSurfaceGenerationFixedLength_LengthLimitMax", k64f, kObject)

        # --- Messages/GoDataSet.h, Messages/GoDataTypes.h ---
        sig(go, "GoDataSet_Count", kSize, kObject)
        sig(go, "GoDataSet_At", kObject, kObject, kSize)
        sig(go, "GoDataMsg_Type", k32s, kObject)

        sig(go, "GoStampMsg_Count", kSize, kObject)
        sig(go, "GoStampMsg_At", POINTER(GoStamp), kObject, kSize)

        for prefix in ("GoUniformSurfaceMsg", "GoSurfacePointCloudMsg"):
            sig(go, f"{prefix}_Length", kSize, kObject)   # rows (Y / travel)
            sig(go, f"{prefix}_Width", kSize, kObject)    # cols (X / laser line)
            sig(go, f"{prefix}_XResolution", k32u, kObject)  # nm
            sig(go, f"{prefix}_YResolution", k32u, kObject)  # nm
            sig(go, f"{prefix}_ZResolution", k32u, kObject)  # nm
            sig(go, f"{prefix}_XOffset", k32s, kObject)      # µm
            sig(go, f"{prefix}_YOffset", k32s, kObject)      # µm
            sig(go, f"{prefix}_ZOffset", k32s, kObject)      # µm
            sig(go, f"{prefix}_Exposure", k32u, kObject)     # ns

        sig(go, "GoUniformSurfaceMsg_RowAt", POINTER(k16s), kObject, kSize)
        sig(go, "GoSurfacePointCloudMsg_RowAt", POINTER(kPoint3d16s), kObject, kSize)

    # ------------------------------------------------------------------
    # Call helpers
    # ------------------------------------------------------------------

    def check(self, name: str, status: int) -> int:
        """Raise unless `status` is kOK.

        Raises:
            GoSdkTimeout: On kERROR_TIMEOUT.
            GoSdkError: On any other non-kOK status.
        """
        if status == kOK:
            return status
        if status == kERROR_TIMEOUT:
            raise GoSdkTimeout(name, status)
        raise GoSdkError(name, status)

    def call(self, name: str, *args: Any) -> int:
        """Invoke a kStatus-returning GoSdk function and check its result."""
        fn = getattr(self.go, name)
        return self.check(name, fn(*args))

    def handle(self, name: str, *args: Any) -> c_void_p:
        """Invoke a handle-returning GoSdk accessor (e.g. ``GoSensor_Setup``).

        Raises:
            GoSdkError: If the accessor returns NULL.
        """
        fn = getattr(self.go, name)
        result = fn(*args)
        if not result:
            raise GoSdkError(name, kERROR)
        return c_void_p(result)

    def parse_ip(self, ip: str) -> kIpAddress:
        """Parse a dotted-quad string into a kIpAddress.

        Raises:
            GoSdkError: If the address doesn't parse.
        """
        address = kIpAddress()
        self.check(
            "kIpAddress_Parse",
            self.kapi.kIpAddress_Parse(byref(address), ip.encode()),
        )
        return address
