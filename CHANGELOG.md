# Changelog

All notable changes to this project are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

Add entries under `[Unreleased]` as you work; `scripts/bump_version.py` moves
them into a dated release section when you cut a version.

## [Unreleased]

## [0.1.0] - 2026-08-03

Formalizes semantic versioning for a codebase that had already grown to cover:

- `FlumeLab` core: opt-in subsystem registration, shared clock, scheduler,
  and event log.
- Gantry motion control (Macron driver) with safety verbs, fence checking,
  and Pi-bridge transport.
- Weir, flow, and gauge subsystems for SAFL hydraulic hardware.
- DSLR and Pi camera capture subsystems.
- MQTT-based rangefinder ingestion (OD2000, WTT12L) and calibration.
- Gocator 3D scanner integration and survey planning.
- Experiment scheduling from CSV/Excel, checkpointing, and a runner CLI.
- Full-experiment simulation/rehearsal mode with no hardware attached.
