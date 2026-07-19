"""Shared test doubles for the macron gantry driver test suite."""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Union

from laguna.robot.macron.connection import SnapConnection


class FakeSnapConnection(SnapConnection):
    """Scripted SnapConnection double for offline unit tests.

    ``responses`` maps an exact command string to either:
      - a response value token (str), as MMCCommands._send() would receive
        it after connection.py's envelope parsing has already happened
      - a callable taking the command string and returning a value token
      - an Exception instance to raise instead

    Every sent command is recorded in ``.sent`` in order, for assertions on
    exact wire format.
    """

    def __init__(self, responses: Dict[str, Union[str, Exception, Callable[[str], str]]] = None):
        self.responses = dict(responses or {})
        self.sent: List[str] = []
        self._connected = False

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def send(self, command: str) -> str:
        self.sent.append(command)
        if command not in self.responses:
            raise AssertionError(f"FakeSnapConnection: no scripted response for {command!r}")
        result = self.responses[command]
        if isinstance(result, Exception):
            raise result
        if callable(result):
            result = result(command)
        return result
