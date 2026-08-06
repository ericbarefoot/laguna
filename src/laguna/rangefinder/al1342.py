"""HTTP client for on-demand reads/writes to an ifm AL1342 IO-Link master.

Provides functions to read PDIN payloads and write acyclic ISDU parameters
via plain HTTP POST to the AL1342 (not MQTT), suitable for on-demand
single reads or interactive writes.
"""

from __future__ import annotations

import json
import urllib.request


def read_pdin_hex(al1342_ip: str, pdin_port: int, timeout: float = 5.0) -> str:
    """Read the current PDIN hex string from an IO-Link device on `pdin_port`.

    Args:
        al1342_ip: IP address of the AL1342 IO-Link master.
        pdin_port: IO-Link port number (1-8).
        timeout: HTTP request timeout in seconds (default 5.0).

    Returns:
        Hex-encoded PDIN payload string.

    Raises:
        RuntimeError: If the AL1342 returns a non-200 code (e.g. the port
            number is wrong or the device is not connected).
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
        al1342_ip: IP address of the AL1342 IO-Link master.
        pdin_port: IO-Link port number (1-8).
        index: IODD parameter index.
        subindex: IODD parameter subindex.
        value: Hex-encoded value string, per the device's IODD.
        timeout: HTTP request timeout in seconds (default 5.0).

    Raises:
        RuntimeError: If the AL1342 does not accept the write (e.g. wrong
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
