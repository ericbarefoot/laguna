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

## Motion arbiter

One lock over the gantry, so two things (a scheduled action and a
manually-issued move, say) can't drive it at once.

::: laguna.robot.motion_arbiter

## Position store

Persists the gantry's last-known axis positions across power cycles — the
recovery path while `home()` is disabled (see its module docstring).

::: laguna.robot.macron.position_store

## Topographic profiler

::: laguna.robot.macron.profiler

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
