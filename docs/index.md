# laguna docs

`laguna` is the control framework for the hydraulic flume lab: a
`FlumeLab` orchestrator coordinates independent, opt-in hardware subsystems
(weir, gauge, flow, cameras, and now the gantry) under a shared clock,
scheduler, and event log.

## Start here

- **[Architecture](ARCHITECTURE.md)** — module map: what lives where, and
  how `FlumeLab` ties subsystems together. (Some of it predates the current
  `core.py` API — cross-check anything load-bearing against the source.)
- **[Quick reference](QUICKREF.md)** — install steps and the fastest path
  to a running experiment.
- **[Workflow guides](guides/index.md)** — worked, runnable examples: your
  first experiment, rehearsing safely with `simulate=True`, scheduling a
  multi-subsystem run, and driving the gantry.

## Gantry (Modusystems OEM-2T / Snap2Motion) & rangefinders

- **[Usage guide](GANTRY_GUIDE.md)** — how to drive it from Python, the
  ASCII command reference split into motion vs. read-only, the safety
  model, and how it plugs into the scheduler alongside weir/gauge. Start
  here if you want to *use* the gantry.
- **[Technical reference](MACRON_GANTRY.md)** — protocol derivation, digital
  IO channel decode, file-by-file breakdown, current hardware-verification
  status. Start here if you're *extending* the driver itself.
- **[Rangefinder profiling](RANGEFINDER_PROFILING.md)** — topographic
  scanning: how `TopographicProfiler`/`gantry_agent.py` fuse gantry motion
  with live OD2000/WTT12L readings into a CSV profile.
- **[MQTT & AL1342 setup](MQTT_AL1342_SETUP.md)** — one-time hardware
  bring-up for the ifm AL1342 IO-Link master (Mosquitto config, static IP,
  port discovery) that both rangefinders sit behind.
- **[WTT12L PowerProx setup](WTT12L_POWERPROX_SETUP.md)** — why the
  WTT12L's native IO-Link path never validated, and the DP4200
  analog-bridge workaround used instead.
- **[Unit calibration finding](archive/GANTRY_UNIT_CALIBRATION.md)** — the
  confirmed 15 mm/unit gantry scale-factor bug and the software workaround
  (`gantry.mm_per_acp_unit`) now baked into `MMCCommands`/`gantry_agent.py`.
- **[Sensor noise characterization](archive/SENSOR_NOISE_CHARACTERIZATION.md)** —
  separating real vibration/obstacle structure in scan data from sensor
  read noise.

## Other subsystems

- **[Subsystem guides](subsystems/camera.md)** — camera, weir, gauge, flow,
  schedule, timing, and the experiment runner, one page each.
- **[Camera USB setup](CAMERA_USB_SETUP.md)** — udev rules and USB gotchas
  for the DSLR/Pi camera arrays.
- **[Contributing](CONTRIBUTING.md)** — dev setup and contribution basics.

## API reference

- **[Generated from docstrings](reference/index.md)** — every public class
  and method in `src/laguna/`, rendered directly from the code via
  [mkdocstrings](https://mkdocstrings.github.io/). If this disagrees with
  the narrative guides above, the code (and this page) is the tiebreaker.

---

## Viewing these docs as a site

This folder is set up for [MkDocs](https://www.mkdocs.org/) with the
Material theme plus [mkdocstrings](https://mkdocstrings.github.io/) for API
docs, configured in `mkdocs.yml` at the repo root.

```bash
make docs-install   # pip install -e ".[docs]" — mkdocs, mkdocs-material, mkdocstrings
make docs-serve     # http://127.0.0.1:8000, live-reloads as you edit docs/*.md
make docs-build     # mkdocs build --strict — writes a static site to site/
```

**Always view it over `http://`, never by opening `site/index.html`
directly (`file://`).** The Material theme's instant-navigation and search
both use `fetch()`, which browsers block on `file://` — the site will look
built but navigation and search will silently fail. `mkdocs serve` and any
real HTTP server both work; double-clicking the built HTML does not.

## History

Past investigation notes, dated findings, and superseded roadmaps live in
[the archive](archive/index.md) rather than cluttering active docs.
