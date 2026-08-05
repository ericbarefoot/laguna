"""Config-section name -> subsystem class, for FlumeLab.add(name)/add_all().

An explicit dict rather than decorator-based auto-registration: there are
only a handful of subsystems, and a table you can read top to bottom avoids
import-order/circular-import surprises. See docs/CONTRIBUTING.md's "Adding
a New Subsystem" section for the from_config(cls, config: Config) contract
every entry here implements.

laguna.camera.manager.CameraManager is deliberately not registered — it
takes a list of per-camera dicts rather than a single config-file section,
and has no from_config() precedent; construct and lab.add() it directly.

laguna.mqtt.subscriber.MqttSubscriber is also deliberately not registered,
even though it has a from_config() and a subsystem_name: each rangefinder
already builds its own private MqttSubscriber from the shared 'mqtt:'
section (see RangefinderSubsystem.from_config()), and nothing reads a
standalone lab.mqtt — registering it here would make add_all() open a
second, redundant real broker connection for any config that happens to
have a top-level 'mqtt:' section (e.g. one written just to override
broker_host/broker_port for the rangefinders), including under
simulate=True, since laguna.simulation's drop-list only covers registered
subsystems. Build and lab.add() an MqttSubscriber directly if you actually
need a standalone one.
"""

from typing import Dict, Type

from .camera.dslr import DslrCameraSubsystem
from .camera.network import CameraArray
from .flow.controller import SaflFlowController
from .gauge.sensor import SaflWaterLevelSensor
from .rangefinder.subsystem import OD2000Rangefinder, WTT12LRangefinder
from .robot.macron.controller import GantryController
from .scanner.gocator import GocatorScanner
from .weir.controller import SaflWeirController

SUBSYSTEM_REGISTRY: Dict[str, Type] = {
    "gantry": GantryController,
    "weir": SaflWeirController,
    "gauge": SaflWaterLevelSensor,
    "flow": SaflFlowController,
    "gocator": GocatorScanner,
    "od2000": OD2000Rangefinder,
    "wtt12l": WTT12LRangefinder,
    "pi_cameras": CameraArray,
    "dslr_cameras": DslrCameraSubsystem,
}
