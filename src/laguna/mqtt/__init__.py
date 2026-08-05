"""MQTT subscriber subsystem for laguna.

Wraps paho-mqtt with the standard laguna connect/disconnect/get_status interface
and a thread-safe per-topic message buffer so callers can drain samples at any
rate without missing messages.
"""

from .subscriber import MqttSubscriber

__all__ = ["MqttSubscriber"]
