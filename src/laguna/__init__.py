"""
Laguna: Robotic flume control system for experimental data acquisition and processing.
"""

from importlib.metadata import version as _version

__version__ = _version("laguna")
__author__ = "Lab Team"

from .core import FlumeLab
from .timing import CheckpointStore, EventLog, ExperimentClock, Scheduler

__all__ = [
    "FlumeLab",
    "ExperimentClock",
    "Scheduler",
    "CheckpointStore",
    "EventLog",
]
