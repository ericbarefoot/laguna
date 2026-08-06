"""PDIN payload decoders for SICK OD2000 and WTT12L rangefinder devices.

Provides functions to decode PDIN (Process Data IN) hex strings from
rangefinders connected via ifm AL1342 IO-Link master.
"""

from typing import Any, Dict


def decode_od2000_pdin(hex_str: str) -> Dict[str, Any]:
    """Decode a raw OD2000 7002T15 PDIN hex string to engineering values.

    Args:
        hex_str: 12-character hex string from AL1342 PDIN payload.

    Returns:
        Dict with keys: distance_nm, distance_mm, scale, q1, q2.
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

    Per SICK "Technical Information: Photoelectric sensors" (www.sick.com/8022709),
    the WTT12L process data is 4 bytes (8 hex chars), big-endian:
    bytes 0-1 = unsigned int16 distance (mm), byte 3 bits 0-1 = QL1/QL2 flags.

    Note: This decoder is unconfirmed and unreachable on tested hardware
    (see docs/WTT12L_POWERPROX_SETUP.md); prefer decode_dp4200_wtt12l_analog_pdin()
    when the sensor is connected via DP4200 analog input bridge.

    Args:
        hex_str: 8-character hex string from AL1342 PDIN payload.

    Returns:
        Dict with keys: distance_mm, ql1, ql2.
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
    """Decode WTT12L distance via its analog output through DP4200 bridge.

    The WTT12L's Qa analog output (4-20 mA) is fed into an ifm DP4200
    IO-Link analog-input bridge. Channel 1 raw reading (bytes 0-1) is
    converted to distance via the 4 mA → near_mm, 20 mA → far_mm span.

    Args:
        hex_str: 8-character hex string from AL1342 PDIN payload.
        near_mm: Distance at 4 mA (default 100 mm).
        far_mm: Distance at 20 mA (default 1400 mm).

    Returns:
        Dict with keys: current_ma (channel 1 in mA), distance_mm.
    """
    raw = bytes.fromhex(hex_str)
    channel1_raw = int.from_bytes(raw[0:2], "big", signed=False)
    current_ma = channel1_raw / 1000.0
    distance_mm = near_mm + (current_ma - 4.0) / 16.0 * (far_mm - near_mm)
    return {
        "current_ma": current_ma,
        "distance_mm": distance_mm,
    }
