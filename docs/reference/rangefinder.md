# Rangefinder

OD2000 and WTT12L PowerProx laser/photoelectric distance sensors, both
reached through the ifm AL1342 IO-Link master. See the
[subsystem guide](../subsystems/rangefinder.md) for usage and
[RANGEFINDER_PROFILING.md](../RANGEFINDER_PROFILING.md) for the topographic
scanning path that fuses these readings with gantry motion.

## Subsystem

::: laguna.rangefinder

## AL1342 transport

::: laguna.rangefinder.al1342

## PDIN decoders

::: laguna.rangefinder.decoders

## Calibration

::: laguna.rangefinder.calibration

## MQTT subscriber

Background subscriber feeding continuous-monitoring readings — currently
the rangefinder subsystem's only caller.

::: laguna.mqtt.subscriber
