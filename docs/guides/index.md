# Workflow guides

Worked, runnable examples for the core things you'll actually do with
`laguna`. Each one is grounded in a real script under `examples/` and has
been run end to end (most under `simulate=True`, with no hardware needed)
while writing this documentation.

- **[Getting started](getting-started.md)** — the smallest complete
  experiment: connect, run, disconnect. Start here.
- **[Rehearsing safely with `simulate=True`](simulate.md)** — exercise a
  whole experiment script with no hardware attached. Do this before running
  anything new against real equipment.
- **[Scheduling a multi-subsystem experiment](scheduled-experiment.md)** —
  recurring polls, one-shot camera triggers, and crash-safe
  checkpoint/resume.
- **[Driving the gantry safely](gantry-motion.md)** — `safe_mode`, fence
  checking, moves, and topographic scans.

For the terse, generated-from-docstrings API surface (every public
class/method with its signature), see [API reference](../reference/index.md).
These guides are the opposite: narrative and usage-focused, not
exhaustive — the docstrings are the source of truth for exact signatures.
