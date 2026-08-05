"""PDIN payload decoders for the OD2000 and WTT12L rangefinder devices."""

from typing import Any, Dict


def decode_od2000_pdin(hex_str: str) -> Dict[str, Any]:
    """Decode a raw OD2000 7002T15 PDIN hex string to engineering values.

    The OD2000 process data is 6 bytes (12 hex chars):
      bytes 0-3  big-endian int32: distance in nm
      byte  4    scale (normally 0)
      byte  5    bit 0 = Q1, bit 1 = Q2

    Args:
        hex_str: 12-character hex string from AL1342 pdin payload field.

    Returns:
        Dict with distance_nm, distance_mm, scale, q1, q2.
    """
    raw = bytes.fromhex(hex_str)
    distance_nm = int.from_bytes(raw[0:4], "big", signed=True)
    return {
        "distance_nm": distance_nm,
        "distance_mm": distance_nm / 1_000_000,
        "scale": raw[4],
        "q1": bool(raw[5] & 0x01),
        "q2": bool(raw[5] & 0x02),
    }


def decode_wtt12l_pdin(hex_str: str) -> Dict[str, Any]:
    """Decode a raw SICK WTT12L PowerProx PDIN hex string to engineering values.

    Per SICK "Technical Information: Photoelectric sensors, SICK Smart
    Sensors / IO-Link" (www.sick.com/8022709), table 6 ("Process data
    structure - WTTxx with or without Smart Task 'Base Logic'"), the WTT12L
    process data is 4 bytes (8 hex chars), big-endian:
      bytes 0-1  unsigned int16: distance to object, mm
      byte  2    reserved
      byte  3    bit 0 = QL1, bit 1 = QL2, bits 2-7 reserved

    Status: unconfirmed and currently unreachable. On hardware (2026-07-28),
    the WTT12L-A2523's native IO-Link process data never validated — the
    AL1342 consistently returned code 530 ("invalid process data") for
    `pdin/getdata`, and every dynamic ISDU read (Detection mode, Sender
    configuration, Distance to object, etc.) returned IDX_NOTAVAIL, while
    static identification ISDUs (vendor/product/serial) read back fine. The
    root cause was never isolated — see docs/WTT12L_POWERPROX_SETUP.md. The
    working path ended up being the sensor's analog output routed through an
    ifm DP4200 IO-Link analog-input bridge instead; see
    decode_dp4200_wtt12l_analog_pdin() for that path's (hardware-confirmed)
    decoder.

    Args:
        hex_str: 8-character hex string from AL1342 pdin payload field.

    Returns:
        Dict with distance_mm, ql1, ql2.
    """
    raw = bytes.fromhex(hex_str)
    distance_mm = int.from_bytes(raw[0:2], "big", signed=False)
    return {
        "distance_mm": distance_mm,
        "ql1": bool(raw[3] & 0x01),
        "ql2": bool(raw[3] & 0x02),
    }


def decode_dp4200_wtt12l_analog_pdin(
    hex_str: str,
    near_mm: float = 100.0,
    far_mm: float = 1400.0,
) -> Dict[str, Any]:
    """Decode a WTT12L-A2523 distance reading taken via its analog output,
    digitized by an ifm DP4200 IO-Link analog-input bridge.

    Used because the WTT12L's own IO-Link process data never validated on
    this AL1342 (see decode_wtt12l_pdin()'s docstring) — the sensor's Qa
    analog output (4-20 mA, un-taught default span near_mm..far_mm) is fed
    into a DP4200 channel instead, and the DP4200 is the thing actually
    plugged into the AL1342 IO-Link port.

    The DP4200 process data is 4 bytes (8 hex chars), big-endian, two
    16-bit fields — one per DP4200 input channel:
      bytes 0-1  channel 1 raw reading, confirmed to be current in µA
      bytes 2-3  channel 2 raw reading — confirmed on hardware 2026-07-28 to
                 sit at a constant 0xFD01 regardless of target distance,
                 consistent with an unconnected/open channel 2 input; not
                 decoded here.

    Byte layout and the "channel 1 = µA" interpretation are confirmed on
    hardware: two readings at physically distinct distances (600 mm ->
    10.384 mA, 1115 mm -> ~16.13 mA) both back-solve to an implied sensor
    full-scale distance within ~5% of the WTT12L-A2523's actual rated max
    range (1,400 mm) — strong cross-check, though the exact near_mm/far_mm
    span was never independently confirmed via the sensor's own teach
    parameters (it's the datasheet's stated un-taught default). Decoded
    distance carries a wider error margin than the OD2000 or a validated
    native WTT12L reading — expect ~20-30 mm of slop on top of the sensor's
    own ±15-20 mm accuracy spec.

    Args:
        hex_str: 8-character hex string from AL1342 pdin payload field.
        near_mm: Distance corresponding to 4 mA (un-taught default = 100 mm
            per the WTT12L-A2523 datasheet).
        far_mm: Distance corresponding to 20 mA (un-taught default = the
            sensor's rated max range, 1,400 mm for the -A2523 variant).

    Returns:
        Dict with current_ma (channel 1) and distance_mm.
    """
    raw = bytes.fromhex(hex_str)
    channel1_raw = int.from_bytes(raw[0:2], "big", signed=False)
    current_ma = channel1_raw / 1000.0
    distance_mm = near_mm + (current_ma - 4.0) / 16.0 * (far_mm - near_mm)
    return {
        "current_ma": current_ma,
        "distance_mm": distance_mm,
    }
