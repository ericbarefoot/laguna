"""Synchronous request/reply over MQTT pub/sub, for actuation subsystems.

Plain pub/sub only tells you a command was *published*, not that hardware
acted on it. Weir and flow are motion/actuation, not sensor streaming, so
`go_to_elevation()`/`set_flowrate()` etc. need to keep meaning "hardware
acknowledged" — this module layers a blocking request/reply on top of
MqttSubscriber to preserve that: publish a command with a unique
`request_id`, then poll the replies topic until a reply echoing that same
`request_id` shows up, or the timeout elapses.

The confluence-side command handler is expected to echo `request_id`
unchanged in its reply payload — see docs on the Teknic_ClearCore and
Fuji_Frenic_VFD confluence Interfaces.
"""

from __future__ import annotations

import time
import uuid
from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from .subscriber import MqttSubscriber


class RequestTimeout(Exception):
    """Raised when no reply matching the request_id arrives within the timeout."""


def request(
    mqtt: "MqttSubscriber",
    commands_topic: str,
    replies_topic: str,
    command: str,
    args: Optional[Dict[str, Any]] = None,
    timeout: float = 5.0,
    poll_interval: float = 0.05,
) -> Dict[str, Any]:
    """Publish a command and block for its correlated reply.

    Subscribes to `replies_topic` (a no-op if already subscribed) and
    discards anything already buffered there before publishing, so a stale
    reply to an earlier, already-timed-out request can't be mistaken for
    this one's.

    Args:
        mqtt: Connected MqttSubscriber to publish/subscribe through.
        commands_topic: Topic to publish the command envelope to.
        replies_topic: Topic the confluence node replies on.
        command: Command name, forwarded verbatim in the envelope.
        args: Command arguments, forwarded verbatim in the envelope.
        timeout: Seconds to wait for a matching reply before raising.
        poll_interval: Seconds to sleep between drain attempts.

    Returns:
        The reply payload dict (whatever the confluence node published,
        including its own `request_id` echo).

    Raises:
        RequestTimeout: If no matching reply arrives within `timeout`.
    """
    request_id = uuid.uuid4().hex
    mqtt.subscribe(replies_topic)
    mqtt.drain(replies_topic)
    mqtt.publish(commands_topic, {"request_id": request_id, "command": command, "args": args or {}})

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for reply in mqtt.drain(replies_topic):
            if isinstance(reply, dict) and reply.get("request_id") == request_id:
                return reply
        time.sleep(poll_interval)

    raise RequestTimeout(
        f"no reply to command {command!r} (request_id={request_id}) within {timeout}s "
        f"on {replies_topic!r}"
    )
