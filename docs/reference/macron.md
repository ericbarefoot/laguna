# Macron gantry driver

The driver for the Modusystems OEM-2T / Snap2Motion 3-axis-plus-rotary
gantry. See the [usage guide](../GANTRY_GUIDE.md) for how to drive it and
the [technical reference](../MACRON_GANTRY.md) for protocol/hardware
background — this page is the generated API surface underneath both.

## Connection

::: laguna.robot.macron.connection

## Commands

::: laguna.robot.macron.commands

## G-code

::: laguna.robot.macron.gcode

## Fences (trajectory safety)

::: laguna.robot.macron.fences

## Homing

::: laguna.robot.macron.homing

## Controller (facade)

::: laguna.robot.macron.controller

## Pi bridge

::: laguna.robot.macron.pi_bridge

## Pi-side agent

!!! warning "Hand-duplicated safety allowlist"
    `gantry_agent.py` runs standalone on the Pi and deliberately does **not**
    import from `pi_bridge.py` — it keeps its own copy of `SAFE_COMMANDS` so
    it can be deployed without installing the `laguna` package. The two
    allowlists must be kept in sync by hand; see the module docstring below
    for the defense-in-depth rationale.

::: laguna.robot.macron.gantry_agent
