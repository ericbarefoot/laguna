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
- **[Unit calibration finding](GANTRY_UNIT_CALIBRATION.md)** — the
  confirmed 15 mm/unit gantry scale-factor bug and the software workaround
  (`gantry.mm_per_acp_unit`) now baked into `MMCCommands`/`gantry_agent.py`.
- **[Sensor noise characterization](SENSOR_NOISE_CHARACTERIZATION.md)** —
  separating real vibration/obstacle structure in scan data from sensor
  read noise.

## Recent work (2026-07-18 → 2026-07-29)

Roughly chronological, each item links to the doc with the full writeup:

1. **MQTT + OD2000 rangefinder integration** — AL1342 bring-up, PDIN
   decode, `RangefinderSubsystem` MQTT streaming path
   ([MQTT & AL1342 setup](MQTT_AL1342_SETUP.md)).
2. **Gantry ACP-unit scale finding** — confirmed 1 ACP unit = 15mm on
   X/Y/Z, not 1mm ([Unit calibration finding](GANTRY_UNIT_CALIBRATION.md)).
3. **Gantry control and scanning unified into `gantry_agent.py`**, and
   verified on real hardware — moves, brake engage/disengage, STOP, and a
   full topographic scan ([Technical reference](MACRON_GANTRY.md)).
4. **WTT12L PowerProx bring-up** via a DP4200 analog bridge, after its
   native IO-Link path failed to validate
   ([WTT12L PowerProx setup](WTT12L_POWERPROX_SETUP.md)).
5. **Line-scan scripts consolidated** (`scripts/run_line_scan.py`,
   replacing several near-duplicate `server-setup/plans/*.py` scripts)
   with real-world-unit output and a `--sensor od2000|wtt12l_powerprox`
   flag.
6. **Sensor noise characterized** — separating real scan structure from
   sensor read noise
   ([Sensor noise characterization](SENSOR_NOISE_CHARACTERIZATION.md)).
7. **API consistency refactor**: `GantryController`/rangefinder
   subsystems now follow the same `connect()`/`activate()`/`read_mm()`
   shape as `laguna.weir`, with simple `FlumeLab.move_to()`/
   `acquire_scan()` verbs on top; the 15mm/unit conversion moved from
   scattered per-script constants into a single `gantry.mm_per_acp_unit`
   config toggle (see the unit calibration doc above).

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

## Extending this documentation (notes for a future Claude session)

**2026-07-18 update:** a docs-overhaul pass (branch `feat-docs-overhaul`)
added `mkdocstrings` (API reference under `reference/`, generated from
docstrings — see the plugin config in `mkdocs.yml`), a custom "Flume Blue"
color skin (`docs/stylesheets/flume.css`, wired via `theme.palette.primary/
accent: custom`), a `Makefile` for `make docs-serve`/`make docs-build`, a
`Subsystems` nav section (`docs/subsystems/*.md`), and consolidated the
duplicate root-level markdown files (old root `ARCHITECTURE.md`,
`GETTING_STARTED.md`, `QUICKSTART.md`, `IMPLEMENTATION_SUMMARY.md`,
`WEIR_GAUGE_CAMERA_SETUP.md`) into this tree. If a root markdown file you'd
expect to find is missing, check here first before recreating it.

If you're picking this up cold before that: this site skeleton
(`mkdocs.yml` + `docs/index.md`) was added 2026-07-17 alongside
`GANTRY_GUIDE.md`, at the end of a session that got the gantry driver's
read-only hardware verification working. If asked to continue this work, in
rough priority order:

1. **Wire the gantry into `src/laguna/experiment/runner.py`'s
   `setup_run()`.** It's the one concrete, well-scoped gap called out in
   `GANTRY_GUIDE.md`'s "Scheduling it alongside weir / gauge" section —
   `gauge`/`weir`/`flow`/cameras are all instantiated and registered there
   from their YAML config sections; the gantry isn't yet, even though its
   config defaults and `from_config()` are ready. The guide has the exact
   diff shape needed.
2. **Add a runnable example** under `experiments/` or `examples/` combining
   gantry + weir + gauge under one `FlumeLab`, once (1) is done — mirror
   `experiments/weir_gauge_camera_experiment.py`'s structure (that's the
   current, non-stale example script; some files under `examples/` are
   stale and reference APIs that no longer exist — don't copy those
   uncritically).
3. **Add pages, not just prose, as the docs grow**: if you add a new `.md`
   file under `docs/`, also add it to `mkdocs.yml`'s `nav:` list, or it
   won't appear in the site navigation (MkDocs doesn't auto-discover pages
   by default with an explicit `nav:` present). For a new module's API
   surface, prefer adding a `::: laguna.your.module` block to the relevant
   page under `docs/reference/` over hand-writing signatures — it stays
   correct automatically as the code changes.
4. **Don't re-derive protocol/IO facts from scratch** — they're already
   verified and written up in `MACRON_GANTRY.md`; treat that file as ground
   truth and link to it rather than duplicating its content elsewhere.
5. If the user asks about diagrams beyond the ASCII-box ones already in
   these docs: GitHub and MkDocs Material both render Mermaid fences
   natively (```` ```mermaid ````), so that's a reasonable upgrade path if
   more detailed sequence/state diagrams are wanted later — no new tooling
   required beyond what's already configured here.
