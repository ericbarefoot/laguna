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

## Gantry (Modusystems OEM-2T / Snap2Motion)

- **[Usage guide](GANTRY_GUIDE.md)** — how to drive it from Python, the
  ASCII command reference split into motion vs. read-only, the safety
  model, and how it plugs into the scheduler alongside weir/gauge. Start
  here if you want to *use* the gantry.
- **[Technical reference](MACRON_GANTRY.md)** — protocol derivation, digital
  IO channel decode, file-by-file breakdown, current hardware-verification
  status. Start here if you're *extending* the driver itself.

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
