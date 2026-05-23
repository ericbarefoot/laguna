"""
Laguna: Robotic flume control system for experimental data acquisition and processing.
"""

__version__ = "0.1.0"
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
