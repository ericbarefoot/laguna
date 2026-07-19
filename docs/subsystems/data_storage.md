# Data & storage

**Status: early scaffolding, not wired into anything.** `DataProcessor`
(`src/laguna/data/__init__.py`) and `RemoteStorage`
(`src/laguna/storage/__init__.py`) are not imported by `core.py`,
`experiment/runner.py`, any example, or any test — a repo-wide search finds
zero references to either class outside their own module. Neither has a
`subsystem_name`, so neither can be `lab.add()`-ed like `weir`/`gauge`/
`flow`/cameras even if you wanted to. Treat everything below as a sketch of
intended shape, not a working feature.

## `DataProcessor`

Buffers dicts in memory (`add_data_point()`/`add_data_points()`) and is
meant to filter/calibrate/resample them before writing out. As written
today:

- `process_data()` does no filtering, calibration, or resampling — it
  returns a shallow copy of the buffer. The docstring's "applies filtering,
  calibration, and other transformations" is aspirational; the method body
  is a straight passthrough with `# TODO` comments marking where that logic
  would go.
- `save_data()` and `export_data()` **log success and return `True`/a path
  string without writing any file.** No HDF5/CSV serialization exists yet
  — calling these will report success while doing nothing to disk. This is
  the single most important thing to know before reaching for this class:
  it currently cannot lose your data, but it also cannot save it.

## `RemoteStorage`

A `storage_type`-dispatched façade (`s3`/`sftp`/`local`) over three backend
classes, all stubs:

| Backend | State |
|---|---|
| `S3Backend` | `connect()` logs and returns `True`; upload/download/list are all `pass` (no `boto3` calls) |
| `SFTPBackend` | Same — `connect()` is a no-op success; no `paramiko` calls despite paramiko being a core dependency elsewhere in the codebase (`camera.network`, `robot.macron.pi_bridge`) |
| `LocalBackend` | `connect()` genuinely works (trivially — local storage is always "available"); upload/download/list are still `pass` |

Every backend's `connect()` reports success unconditionally, so
`RemoteStorage.connect()` returning `True` tells you nothing about whether
a real connection was made — don't use that return value as a readiness
check for anything real yet.

## If you're picking this up to actually build it

There's no design decision recorded anywhere for *why* these two modules
exist separately from the pattern the rest of the codebase uses (opt-in
`FlumeLab` subsystems with `connect()`/`get_status()`/a `subsystem_name`).
Before writing real backend logic, it's worth first deciding whether these
should become proper subsystems in that same shape, or stay as
utility classes invoked by an experiment script directly — the current
code doesn't commit to either.

## Further reading

- `src/laguna/data/__init__.py`, `src/laguna/storage/__init__.py` — the modules themselves; short enough to read directly rather than relying on this page.
- [Experiment runner](experiment.md) — the actual subsystem-wiring pattern these would need to follow to become opt-in `FlumeLab` subsystems.
