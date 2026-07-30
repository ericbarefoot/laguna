"""Shared HTTP client for on-demand (acyclic) reads/writes against an ifm
AL1342 IO-Link master.

Promoted out of duplicated urllib code in
examples/example_06_od2000_acyclic_read.py and
scripts/calibrate_rangefinder.py — both talked to the same two AL1342
endpoints (pdin/getdata, iolwriteacyclic) independently. See
docs/MQTT_AL1342_SETUP.md: the AL1342 has no DNS of its own (address it by
raw IP) and this is plain HTTP POST, not MQTT — MQTT's timer-push
mechanism has a ~2Hz floor unsuitable for on-demand single reads or acyclic
writes.
"""

from __future__ import annotations

import json
import urllib.request


def read_pdin_hex(al1342_ip: str, pdin_port: int, timeout: float = 5.0) -> str:
    """Read the current pdin hex string from an IO-Link device on `pdin_port`.

    Raises:
        RuntimeError: If the AL1342 returns a non-200 code (e.g. the port
            number is wrong or the device isn't connected).
    """
    adr = f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/pdin/getdata"
    payload = json.dumps({"code": "request", "cid": -1, "adr": adr}).encode()
    req = urllib.request.Request(
        f"http://{al1342_ip}/", data=payload, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = json.loads(resp.read())
    if body.get("code") != 200:
        raise RuntimeError(
            f"AL1342 returned code {body.get('code')} for {adr} — "
            f"check pdin_port and that the device is connected"
        )
    return body["data"]["value"]


def write_acyclic(
    al1342_ip: str,
    pdin_port: int,
    index: int,
    subindex: int,
    value: str,
    timeout: float = 5.0,
) -> None:
    """Write an IO-Link acyclic parameter (ISDU) to the device on `pdin_port`.

    Args:
        index: IODD parameter index.
        subindex: IODD parameter subindex.
        value: Hex-encoded value string, per the device's IODD.

    Raises:
        RuntimeError: If the AL1342 doesn't accept the write (e.g. wrong
            pdin_port, or the device rejects the index/subindex).
    """
    payload = json.dumps({
        "code": "request", "cid": -1,
        "adr": f"/iolinkmaster/port[{pdin_port}]/iolinkdevice/iolwriteacyclic",
        "data": {"index": index, "subindex": subindex, "value": value},
    }).encode()
    req = urllib.request.Request(
        f"http://{al1342_ip}/", data=payload, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = json.loads(resp.read())
    if body.get("code") != 200:
        raise RuntimeError(
            f"AL1342 returned code {body.get('code')} writing index={index} "
            f"subindex={subindex} on port {pdin_port}"
        )
