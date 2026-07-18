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

- **[Camera USB setup](CAMERA_USB_SETUP.md)** — udev rules and USB gotchas
  for the DSLR/Pi camera arrays.

---

## Viewing these docs as a site

This folder is set up for [MkDocs](https://www.mkdocs.org/) with the
Material theme, configured in `mkdocs.yml` at the repo root. Nothing is
installed by default — to preview locally:

```bash
pip install mkdocs mkdocs-material
mkdocs serve      # http://127.0.0.1:8000, live-reloads as you edit docs/*.md
mkdocs build      # writes a static site to site/ if you want to host it somewhere
```

## Extending this documentation (notes for a future Claude session)

If you're picking this up cold: this site skeleton (`mkdocs.yml` +
`docs/index.md`) was added 2026-07-17 alongside `GANTRY_GUIDE.md`, at the
end of a session that got the gantry driver's read-only hardware
verification working. Nothing beyond that has been built out yet. If asked
to continue this work, in rough priority order:

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
   by default with an explicit `nav:` present).
4. **Don't re-derive protocol/IO facts from scratch** — they're already
   verified and written up in `MACRON_GANTRY.md`; treat that file as ground
   truth and link to it rather than duplicating its content elsewhere.
5. If the user asks about diagrams beyond the ASCII-box ones already in
   these docs: GitHub and MkDocs Material both render Mermaid fences
   natively (```` ```mermaid ````), so that's a reasonable upgrade path if
   more detailed sequence/state diagrams are wanted later — no new tooling
   required beyond what's already configured here.
