"""Core orchestrator module for FlumeLab.

FlumeLab uses an opt-in model: subsystems are attached with lab.add(subsystem),
lab.add("name") (looks up laguna.registry and builds it from config), or
lab.add_all() (every subsystem whose section is present in the loaded config).
This avoids hardcoding hardware assumptions in the core.
"""
from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Iterator, List, Optional
import logging
import threading
import time

from .config import Config
from .frames import FrameRegistry
from .run_context import RunContext
from .safety import (
    DEFAULT_SENTINELS,
    CallableTrigger,
    SafetyMonitor,
    SafetyState,
    SafetyTier,
    SentinelFileTrigger,
)
from .timing import CheckpointStore, EventLog, ExperimentClock, Scheduler

if TYPE_CHECKING:
    from .rangefinder import OD2000Rangefinder, WTT12LRangefinder
    from .robot.macron.controller import GantryController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

#: Third-party libraries pinned to WARNING even under debug=True — "debug my
#: code", not "debug every dependency's own chatter" (matches the existing
#: paramiko silencing precedent in laguna.experiment.runner.setup_run()).
_THIRD_PARTY_LOGGERS = ("paramiko", "paramiko.transport")

#: Tags a FileHandler as this class's own operational-log handler, so a
#: later FlumeLab construction in the same process (a REPL session, a test
#: suite) can find and remove the previous one instead of piling up
#: duplicate handlers that each keep writing to an old run's log file.
_OPERATIONAL_LOG_HANDLER_ATTR = "_laguna_operational_log"


class FlumeLab:
    """Main orchestrator for the flume lab robotic system.

    Creates a timing backbone immediately; hardware subsystems are registered
    explicitly via ``lab.add(subsystem)``.

    Attributes:
        gantry: Registered gantry controller when present.
        od2000: Registered OD2000 rangefinder when present.
        wtt12l: Registered WTT12L rangefinder when present.

    Example::

        lab = FlumeLab("config.yaml")
        lab.add_all()  # builds + registers every subsystem present in config.yaml
        lab.connect_all()
        with lab.experiment() as clock:
            lab.scheduler.run(duration=300)

    Attributes:
        clock:      Experiment clock (wall time + runtime)
        scheduler:  Action scheduler tied to the experiment clock
        event_log:  Append-only CSV event log
    """

    gantry: "GantryController"
    od2000: "OD2000Rangefinder"
    wtt12l: "WTT12LRangefinder"

    def __init__(
        self,
        config_file: Optional[str] = None,
        simulate: bool = False,
        speed_factor: Optional[float] = None,
        debug: bool = False,
    ) -> None:
        logger.info("Initializing FlumeLab system...")

        self.config = Config(config_file=config_file)

        #: Rehearsal mode. The scheduler, clock, event log, manifest, frames,
        #: survey planner and safety verbs are all real — only the wire is
        #: simulated. See laguna.simulation for what this can and cannot
        #: catch.
        self.simulate = simulate
        #: Experiment seconds per real second. Only honoured with
        #: simulate=True: accelerating a run that is driving real hardware
        #: would just make the schedule outrun the machine.
        self.speed_factor = float(speed_factor or 1.0)
        if speed_factor and not simulate:
            raise ValueError(
                "speed_factor only applies to simulate=True. Real hardware "
                "cannot be sped up — a gantry takes as long as it takes — so "
                "accelerating a live run would just make the schedule outrun "
                "the machine."
            )
        if simulate:
            from .simulation import simulate_config

            self.config.config_dict = simulate_config(self.config.config_dict)
            # simulate_config() drops sections with no simulated backend
            # (everything but gantry/gocator) from config_dict outright —
            # explicit_sections must lose them too, or add_all() would try
            # to build a real weir/flow/gauge/camera controller against
            # real hardware during what is supposed to be a hardware-free
            # rehearsal (see laguna.simulation's module docstring).
            self.config.explicit_sections &= set(self.config.config_dict.keys())
            logger.warning(
                "SIMULATION MODE — no hardware will be contacted. Structural "
                "mistakes (schedules, survey extents, missing config) surface; "
                "physical ones (mounting signs, unreachable targets) do not."
            )
            if self.speed_factor != 1.0:
                logger.warning(
                    "Clock running at %.0fx — a %.0f-second experiment finishes "
                    "in %.1f real seconds.",
                    self.speed_factor, 3600.0, 3600.0 / self.speed_factor,
                )

        # Instrument mounts + the experiment's reference frame. Always
        # present; an absent 'frames:' section yields identity transforms and
        # zero offsets, so a rig without one behaves exactly as before.
        # See laguna.frames.
        self.frames = FrameRegistry.from_config(self.config.get_value("frames"))

        # Safety lifecycle — see laguna.safety. The monitor is created here
        # but stays idle until watch_for_estop() adds triggers and starts it.
        self._safety_state = SafetyState.RUNNING
        self.safety_monitor = SafetyMonitor(on_trip=self._on_safety_trigger)

        # Ties this run's outputs together and records the piecewise
        # runtime<->wall mapping. run_dir defaults to None, which leaves every
        # subsystem writing exactly where it always did — see laguna.run_context.
        self.run = RunContext(
            root=self.config.get_value("timing.run_dir"),
            speed_factor=self.speed_factor,
        )

        #: Global troubleshooting switch. True sets every laguna.* logger
        #: (including each subsystem's own — overriding its individual
        #: log_level config, see laguna.subsystem_logging) to DEBUG, which
        #: is where tessellated motion segments and other low-level detail
        #: surface; third-party libraries stay pinned to WARNING regardless.
        #: False leaves each subsystem's own log_level in charge, as today.
        self.debug = debug
        self._configure_operational_log()

        # Timing subsystem — always present
        self.clock = ExperimentClock(speed_factor=self.speed_factor)
        # Observe every pause/resume, whoever caused it. Scheduler.stop()
        # pauses the clock directly, so recording only in FlumeLab.pause()
        # silently missed those intervals and left the saved timeline wrong.
        self.clock.on_pause = self.run.paused
        self.clock.on_resume = self.run.resumed
        event_log_path = Path(self.config.get_value("timing.event_log", "./experiment_events.csv"))
        if self.simulate:
            # A rehearsal must never be able to land in the same file as a
            # real experiment's archival record — this file ships as
            # metadata alongside published data, so a simulated row mixed
            # in undetected would be a real data-integrity problem.
            # Suffixed unconditionally, even if timing.event_log was set
            # explicitly, since the same config file is often reused
            # for both a real run and a rehearsal of it.
            event_log_path = event_log_path.with_name(
                f"{event_log_path.stem}_simulated{event_log_path.suffix}"
            )
        self.event_log = EventLog(str(event_log_path))
        if self.simulate:
            # Belt-and-suspenders alongside the filename split above: even
            # if this file's contents end up copied/merged elsewhere, the
            # row itself still says what it is.
            self.event_log.log(
                0.0, "flume_lab", "simulate_mode",
                notes=f"rehearsal — no hardware contacted; speed_factor={self.speed_factor}",
            )
        self.scheduler = Scheduler(clock=self.clock, event_log=self.event_log)

        # Registry for opt-in hardware subsystems
        self._subsystems: Dict[str, Any] = {}

        self._duration: Optional[float] = None   # set by start(), used by resume()
        self._start_wall: Optional[float] = None  # wall time of lab.start()
        self.is_running = False
        logger.info("FlumeLab timing backbone ready — add subsystems via lab.add()")

    def _configure_operational_log(self) -> None:
        """Wire up the operational log tier: connections + broad motion at
        INFO, low-level/tessellated detail at DEBUG — distinct from the
        terse, archival event_log CSV (see laguna.subsystem_logging's
        module docstring for the full split). Reuses standard Python
        logging rather than a second structured file format; persisted
        under the run directory (run.root/run.run_id/laguna.log) when one
        is configured, terminal-only otherwise.
        """
        from .subsystem_logging import set_global_debug

        # Always set explicitly (never leave the prior value) — a REPL or
        # test suite constructing FlumeLab(debug=True) then FlumeLab()
        # again in the same process must not leave DEBUG stuck from the
        # first instance. NOTSET defers back to root's basicConfig(INFO).
        # Third-party loggers are only ever pinned quieter here, never
        # reset — debug=False must not clobber a level some other code
        # (e.g. runner.py's verbose_cameras handling) already configured.
        root_logger = logging.getLogger("laguna")
        root_logger.setLevel(logging.DEBUG if self.debug else logging.NOTSET)
        if self.debug:
            for noisy in _THIRD_PARTY_LOGGERS:
                logging.getLogger(noisy).setLevel(logging.WARNING)
        set_global_debug(self.debug)

        for handler in list(root_logger.handlers):
            if getattr(handler, _OPERATIONAL_LOG_HANDLER_ATTR, False):
                root_logger.removeHandler(handler)
                handler.close()

        if self.run.root is not None:
            log_path = self.run.root / self.run.run_id / "laguna.log"
            log_path.parent.mkdir(parents=True, exist_ok=True)
            handler = logging.FileHandler(log_path)
            handler.setFormatter(logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            ))
            setattr(handler, _OPERATIONAL_LOG_HANDLER_ATTR, True)
            root_logger.addHandler(handler)
            logger.info("Operational log: %s", log_path)

    # ------------------------------------------------------------------
    # Opt-in subsystem registration
    # ------------------------------------------------------------------

    def __getattr__(self, name: str) -> Any:
        """Return a registered subsystem by name when accessed as an attribute."""
        if name.startswith("_"):
            raise AttributeError(name)
        subsystem = self._subsystems.get(name)
        if subsystem is None:
            raise AttributeError(f"{type(self).__name__!r} object has no attribute {name!r}")
        return subsystem

    def add(self, subsystem: Any) -> "FlumeLab":
        """Register a hardware subsystem by its ``subsystem_name`` attribute.

        The subsystem is stored both in ``self._subsystems`` (keyed by name)
        and as a direct attribute (``self.<subsystem_name>``), making
        ``lab.cameras``, ``lab.gantry``, etc. work naturally. Subsystems
        exposing ``attach_event_log()`` (see laguna.subsystem_logging) are
        wired to this lab's event_log/clock automatically, so they can log
        their own key actions without the caller doing it by hand.

        Args:
            subsystem: Either a built object with a ``subsystem_name`` class
                       or instance attribute (e.g. CameraManager,
                       GantryController), or a registry name (e.g.
                       ``"gantry"``, see laguna.registry.SUBSYSTEM_REGISTRY)
                       to build via that class's ``from_config(self.config)``.

        Returns:
            self — so calls can be chained: ``lab.add(cam).add("gantry")``
        """
        if isinstance(subsystem, str):
            from .registry import SUBSYSTEM_REGISTRY

            subsystem_cls = SUBSYSTEM_REGISTRY.get(subsystem)
            if subsystem_cls is None:
                raise ValueError(
                    f"{subsystem!r} is not in laguna.registry.SUBSYSTEM_REGISTRY; "
                    "pass a built subsystem instance instead, or add it to the registry."
                )
            subsystem = subsystem_cls.from_config(self.config)

        name = getattr(subsystem, "subsystem_name", None)
        if not name:
            raise ValueError(
                f"{type(subsystem).__name__} has no 'subsystem_name' attribute; "
                "cannot register as a FlumeLab subsystem."
            )
        self._subsystems[name] = subsystem
        setattr(self, name, subsystem)
        if hasattr(subsystem, "attach_event_log"):
            subsystem.attach_event_log(self.event_log, self.clock)
        logger.info("Registered subsystem '%s' (%s)", name, type(subsystem).__name__)
        return self

    def add_all(self) -> "FlumeLab":
        """Add every subsystem whose config section was explicitly present in the YAML.

        Opt-in follows ``self.config.explicit_sections`` (set by
        ``Config.load_from_file()``), not ``config_dict`` — ``_get_defaults()``
        populates every section unconditionally, so config_dict alone can't
        tell "explicitly configured" from "just the default." Section names
        with no registry entry (e.g. ``timing``, ``frames``) are skipped.

        Returns:
            self — so calls can be chained: ``lab.add_all().connect_all()``
        """
        from .registry import SUBSYSTEM_REGISTRY

        for name in sorted(self.config.explicit_sections):
            if name in SUBSYSTEM_REGISTRY:
                self.add(name)
        return self

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect_all(self) -> bool:
        """Establish connections to all registered subsystems.

        Calls ``connect()`` or ``start()`` on each subsystem if the method
        exists, and collects failures.

        Returns:
            True if all connections successful, False if any failed.
        """
        logger.info("Connecting to all subsystems...")
        ok = True

        for name, subsystem in self._subsystems.items():
            # Prefer connect(); fall back to start() for camera-style APIs.
            connector = getattr(subsystem, "connect", None) or getattr(subsystem, "start", None)
            if connector is None:
                continue
            if not connector():
                logger.warning("Failed to connect subsystem '%s'", name)
                ok = False

        if ok:
            logger.info("All subsystems connected successfully")
        return ok

    def disconnect_all(self) -> None:
        """Disconnect all registered subsystems.

        Also closes out the run manifest if one was opened by start() and
        not already closed — this is the guaranteed conclusion point for
        the start()/resume() REPL flow and for run_blocking() (which calls
        this in a finally block), unlike experiment(), which closes its own
        run in the context manager's exit.
        """
        logger.info("Disconnecting all subsystems...")
        for name, subsystem in self._subsystems.items():
            disconnector = getattr(subsystem, "disconnect", None) or getattr(subsystem, "stop", None)
            if disconnector:
                disconnector()
        self.is_running = False
        if self.run.started_wall is not None and self.run.ended_wall is None:
            self.run.ended()
        logger.info("All subsystems disconnected")

    # ------------------------------------------------------------------
    # Experiment context manager
    # ------------------------------------------------------------------

    @contextmanager
    def experiment(
        self,
        resume: bool = False,
        checkpoint_file: Optional[str] = None,
    ) -> Iterator[ExperimentClock]:
        """Context manager that starts/stops the clock and logs experiment boundaries.

        Yields the ExperimentClock so the caller can call clock.wait_until(),
        clock.elapsed(), etc. directly inside the with-block.

        A CheckpointStore is created automatically (use resume=True on restart).

        Example::

            lab = FlumeLab("config.yaml")
            lab.connect_all()
            with lab.experiment(resume=False) as clock:
                lab.scheduler.repeat(every=5, action=lab.cameras.trigger_capture)
                lab.scheduler.run(duration=300)

        Args:
            resume:          If True, reload a previous checkpoint file rather
                             than starting fresh.
            checkpoint_file: Override the path from config
                             (timing.checkpoint_file).
        """
        cp_path = checkpoint_file or self.config.get_value(
            "timing.checkpoint_file", "./experiment_checkpoint.json"
        )
        store = CheckpointStore(cp_path, resume=resume)

        self.clock.start()
        self.run.started()
        self.event_log.log(
            0.0, "flume_lab", "experiment_start", notes=f"run_id={self.run.run_id}"
        )
        try:
            yield self.clock
        finally:
            self.clock.stop()
            self.run.ended()
            self.event_log.log(
                self.clock.elapsed(), "flume_lab", "experiment_stop",
                notes=f"run_id={self.run.run_id}",
            )
            self.event_log.close()

    # ------------------------------------------------------------------
    # Status / safety
    # ------------------------------------------------------------------

    def get_system_status(self) -> Dict[str, Any]:
        """Return a snapshot of all subsystem states.

        Each registered subsystem is queried via ``get_status()`` if available;
        otherwise a minimal ``{'registered': True}`` placeholder is used.
        The timing backbone is always included.
        """
        wall, runtime = self.clock.now()
        status: Dict[str, Any] = {
            "timing": {
                "clock_running": self.clock.is_running and not self.clock.is_paused,
                "clock_paused": self.clock.is_paused,
                "runtime_s": runtime,
                "wall_time": wall,
            },
        }
        for name, subsystem in self._subsystems.items():
            getter = getattr(subsystem, "get_status", None)
            status[name] = getter() if getter else {"registered": True}
        return status

    def start(self, duration: float) -> threading.Thread:
        """Start the experiment clock and scheduler in a background thread.

        Intended for interactive / REPL use after main_interactive() returns.
        Nothing runs until this is called.

        Args:
            duration: How long to run the scheduler, in experiment-time seconds.
                      Stored so that resume() with no arguments continues for
                      the remaining time.

        Returns:
            The scheduler thread (daemon). Call lab.stop() to pause early.
        """
        self._duration = duration
        self._start_wall = time.time()
        self.clock.start()
        self.run.started()
        self.event_log.log(
            0.0, "flume_lab", "experiment_start", notes=f"run_id={self.run.run_id}"
        )
        thread = threading.Thread(
            target=self.scheduler.run,
            args=(duration,),
            kwargs={"on_complete": self.print_summary},
            daemon=True,
            name="scheduler-main",
        )
        thread.start()
        logger.info("Experiment started for %.0f seconds.", duration)
        logger.info("  lab.stop()          — pause")
        logger.info("  lab.resume(N)       — resume for N more seconds")
        logger.info("  lab.disconnect_all() — clean shutdown when finished")
        return thread

    def print_summary(self) -> None:
        """Print a summary of the completed experiment to stdout.

        Called automatically when the scheduled duration expires. Can also be
        called manually at any point during or after an experiment.
        """
        import csv as _csv
        from datetime import datetime

        runtime = self.clock.elapsed()
        start_str = (
            datetime.fromtimestamp(self._start_wall).strftime("%Y-%m-%d %H:%M:%S")
            if self._start_wall else "unknown"
        )

        # Parse the event log for counts and capture paths
        event_counts: Dict[tuple, int] = {}
        capture_paths: list = []
        try:
            with open(self.event_log._path, newline="") as f:
                for row in _csv.DictReader(f):
                    sub = row.get("subsystem", "")
                    evt = row.get("event_type", "")
                    if sub in ("flume_lab", "scheduler"):
                        continue
                    event_counts[(sub, evt)] = event_counts.get((sub, evt), 0) + 1
                    if sub in ("pi_cameras", "dslr_cameras") and evt == "capture":
                        result = row.get("result", "")
                        for part in result.split():
                            if part.startswith("file="):
                                capture_paths.append((sub, part[len("file="):]))
        except Exception:
            pass

        sep = "=" * 52
        lines = [
            "",
            sep,
            "  Experiment Complete",
            sep,
            f"  Started   {start_str}",
            f"  Runtime   {runtime:.1f} s  ({runtime / 60:.1f} min)",
            f"  Log       {self.event_log._path}",
            "",
            "  Subsystems",
        ]

        for name, subsystem in self._subsystems.items():
            getter = getattr(subsystem, "get_status", None)
            connected = "?"
            detail = ""
            if getter:
                st = getter()
                connected = "connected" if st.get("is_connected", True) else "not connected"
                if name == "pi_cameras" and "hosts" in st:
                    detail = f"  ({', '.join(st['hosts'])})"
            lines.append(f"    {name:<14} {connected}{detail}")

        if event_counts:
            lines.append("")
            lines.append("  Events")
            for (sub, evt), count in sorted(event_counts.items()):
                lines.append(f"    {sub:<16} {evt:<26} {count:>4}×")

        if capture_paths:
            from pathlib import Path as _Path
            by_sub: Dict[str, list] = {}
            for sub, p in capture_paths:
                by_sub.setdefault(sub, []).append(p)
            lines.append("")
            lines.append("  Captures")
            for sub, paths in sorted(by_sub.items()):
                dirs = {str(_Path(p).parent) for p in paths if "/" in p or "\\" in p}
                lines.append(f"    {sub:<16} {len(paths)} images")
                for d in sorted(dirs):
                    lines.append(f"      {d}/")

        lines += [sep, ""]
        print("\n".join(lines), flush=True)

    def stop(self) -> None:
        """Pause the experiment. Delegates to :meth:`pause`.

        This has always been a pause despite the name — it logs
        experiment_pause, halts the scheduler, and leaves hardware connected.
        It used to quiesce only the weir; it now quiesces every subsystem
        that implements pause(), and pauses the clock. Kept as an alias
        because existing scripts and run_blocking() call it.
        """
        self.pause(reason="lab.stop()")

    def resume(self, remaining_s: Optional[float] = None) -> threading.Thread:
        """Resume after stop(): restart scheduler loop in background thread.

        Args:
            remaining_s: How long to run, in experiment-time seconds. If omitted,
                         uses the time remaining from the original lab.start() call
                         (i.e. start_duration − elapsed_runtime).

        Returns:
            The scheduler thread, so caller can .join() it if desired.
        """
        if remaining_s is None:
            if self._duration is None:
                raise RuntimeError("No duration stored — call lab.start(duration) before resume()")
            remaining_s = max(0.0, self._duration - self.clock.elapsed())
            logger.info(
                "Resuming for %.1f remaining seconds (%.1fs elapsed of %.1fs total)",
                remaining_s, self.clock.elapsed(), self._duration,
            )
        else:
            logger.info("Resuming experiment for %.1f more seconds...", remaining_s)
        return self.scheduler.run_async(remaining_s)

    # ------------------------------------------------------------------
    # Simple verbs — thin delegates to the richer per-subsystem API, for
    # the most common actions (see laguna.robot.macron.controller.GantryController
    # and laguna.robot.macron.profiler.TopographicProfiler for everything
    # these don't cover).
    # ------------------------------------------------------------------

    def move_to(self, vector: Optional[list] = None, **axes) -> bool:
        """Move the gantry to an absolute position.

        Thin delegate to ``self.gantry.move_to()`` — see
        GantryController.move_to() for the full vector
        (``move_to([x, y, z, theta])``) vs. per-axis keyword
        (``move_to(X=100)``) forms.

        Raises:
            RuntimeError: If no 'gantry' subsystem is registered.
        """
        gantry = self._subsystems.get("gantry")
        if gantry is None:
            raise RuntimeError("move_to() requires a 'gantry' subsystem — lab.add(GantryController(...))")
        return gantry.move_to(vector, **axes)

    def place(
        self,
        instrument: str,
        experiment_point: list,
        speed: Optional[float] = None,
    ) -> bool:
        """Move so `instrument` measures at a point in the *experiment* frame.

        The counterpart to move_to(): name the place you want measured rather
        than the robot position that gets you there. Because each instrument
        is mounted somewhere different, the same experiment point yields a
        different gantry command per instrument — which is exactly what lets
        you re-run a transect with a second sensor::

            lab.place("od2000", [100, 200, 0])     # OD2000's dot on the target
            lab.place("wtt12l", [100, 200, 0])     # WTT12L's dot on the SAME spot

        Offsets and the experiment frame come from ``lab.frames`` (the
        ``frames:`` config section). An instrument with no configured frame is
        assumed to measure at the gantry's commanded point, so this reduces to
        move_to() on an unconfigured rig.

        Args:
            instrument: Instrument key, e.g. ``"od2000"``.
            experiment_point: [x, y, z] in experiment coordinates.
            speed: Optional feed rate, mm/s.

        Returns:
            True if a move was issued.

        Raises:
            RuntimeError: If no 'gantry' subsystem is registered.
        """
        target = self.frames.gantry_target_for(instrument, experiment_point)
        logger.info(
            "place(%s, %s) -> gantry %s",
            instrument, list(experiment_point), [round(v, 3) for v in target],
        )
        # Theta is outside the Cartesian frame model — leave it untouched by
        # using the keyword form rather than a full vector.
        return self.move_to(X=target[0], Y=target[1], Z=target[2], speed=speed)

    def acquire_scan(
        self,
        instrument: str,
        start: Optional[list] = None,
        end: Optional[list] = None,
        output: Optional[str] = None,
        feed_rate_mm_s: Optional[float] = None,
    ):
        """Move to `start` (if given) and scan to `end`, saving a topographic profile.

        `start`/`end` are full position vectors, one value per configured
        gantry axis (same order as move_to()'s vector form) — exactly one
        component may differ between them, since a single scan pass only
        moves one axis (see TopographicProfiler.scan()); that's the axis
        actually scanned.

        Args:
            instrument: "od2000" or "wtt12l" (alias for "wtt12l_powerprox")
                — selects which rangefinder feeds the scan, and whose
                config section (al1342_host, pdin_port) is used.
            start: Optional full position vector to move to before
                scanning. If omitted, the scan starts from wherever the
                gantry already is.
            end: Full position vector marking where the scan axis should
                stop. Required.
            output: Optional CSV path for the result (renamed from the
                profiler's default auto-timestamped path). If omitted, the
                default output_dir/timestamped path is used.
            feed_rate_mm_s: Scan speed in mm/s. Required — deliberately no
                default, since this drives a real hardware move.

        Returns:
            ProfileResult (path, metadata, DataFrame) — see
            laguna.robot.macron.profiler.ProfileResult.

        Raises:
            RuntimeError: If no 'gantry' subsystem is registered.
            ValueError: If `end` or `feed_rate_mm_s` is missing, `start`/
                `end` don't match the configured axis count, or they don't
                differ on exactly one axis.
            KeyError: If no config section exists for `instrument`.
        """
        from laguna.robot.macron.profiler import TopographicProfiler
        from laguna.robot.motion_arbiter import DEFAULT_ARBITER

        gantry = self._subsystems.get("gantry")
        if gantry is None:
            raise RuntimeError("acquire_scan() requires a 'gantry' subsystem — lab.add(GantryController(...))")
        if end is None:
            raise ValueError("acquire_scan() requires end=[...]")
        if feed_rate_mm_s is None:
            raise ValueError("acquire_scan() requires feed_rate_mm_s — no default for a hardware move")

        # Held across the pre-position move() and the scan pass itself, not
        # just move_to() individually — a scheduled Gocator scan (which holds
        # the same arbiter via scan_with_gantry()) must not be able to
        # command the gantry in between the two.
        arbiter = getattr(gantry, "arbiter", DEFAULT_ARBITER)
        with arbiter.hold(f"acquire_scan {instrument} -> {end}"):
            if start is not None:
                self.move_to(start)

            axis_names = [axis.name for axis in gantry._axes]
            if start is None:
                start = [gantry.cmd.get_actual_position(axis) for axis in gantry._axes]
            if len(start) != len(axis_names) or len(end) != len(axis_names):
                raise ValueError(
                    f"start/end must have {len(axis_names)} values (one per configured axis: {axis_names})"
                )

            differing = [name for name, s, e in zip(axis_names, start, end) if abs(e - s) > 1e-9]
            if len(differing) != 1:
                raise ValueError(
                    "acquire_scan() infers the scan axis as the single component where "
                    f"start and end differ; got {len(differing)} differing axes: {differing}"
                )
            scan_axis = next(axis for axis in gantry._axes if axis.name == differing[0])
            end_mm = end[axis_names.index(differing[0])]

            sensor = "wtt12l_powerprox" if instrument in ("wtt12l", "wtt12l_powerprox") else instrument
            rf_config = self.config.get(instrument)
            gantry_config = self.config.get("gantry")

            gantry_host = gantry_config.get("host")
            if not isinstance(gantry_host, str):
                raise ValueError("gantry.host must be configured as a string")

            gantry_ssh_user = gantry_config.get("ssh_user", "oak")
            if not isinstance(gantry_ssh_user, str):
                raise ValueError("gantry.ssh_user must be configured as a string")

            gantry_ssh_key = gantry_config.get("ssh_key")
            if gantry_ssh_key is not None and not isinstance(gantry_ssh_key, str):
                raise ValueError("gantry.ssh_key must be configured as a string when provided")

            al1342_host = rf_config.get("al1342_host")
            if not isinstance(al1342_host, str):
                raise ValueError("instrument.al1342_host must be configured as a string")

            # profiler.scan() is the actual gantry-touching operation and
            # must stay inside the arbiter hold — see the comment above.
            profiler = TopographicProfiler(
                gantry=gantry,
                pi_host=gantry_host,
                pi_user=gantry_ssh_user,
                pi_key=gantry_ssh_key,
                pdin_port=rf_config.get("pdin_port", 1),
                al1342_host=al1342_host,
                output_dir=str(Path(output).parent) if output else "/tmp",
                sensor=sensor,
            )
            result = profiler.scan(axis=scan_axis.token(), end_mm=end_mm, feed_rate_mm_s=feed_rate_mm_s)

        if output:
            output_path = Path(output)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            old_meta = Path(str(result.path).replace(".csv", "_meta.json"))
            result.path.replace(output_path)
            if old_meta.exists():
                old_meta.replace(Path(str(output_path).replace(".csv", "_meta.json")))
            result.path = output_path

        return result

    # ------------------------------------------------------------------
    # Safety verbs — see laguna.safety for the three tiers and why they
    # exist. Every loop here is guarded per subsystem: one failing must
    # never stop the rest from being brought to a safe state.
    # ------------------------------------------------------------------

    #: Order for estop: motion first (the thing that can hit someone), then
    #: hydraulics, then everything else. Subsystems not listed follow.
    _ESTOP_ORDER = ("gantry", "weir", "flow")

    @property
    def safety_state(self) -> SafetyState:
        """Where the rig is in the pause/stop/estop lifecycle."""
        return self._safety_state

    def _for_each_subsystem(self, verb: str, order: tuple = ()) -> List[str]:
        """Call `verb` on every subsystem that has it, guarded individually.

        Notes returned by a subsystem go to the **event log**, not just the
        Python logger — above all when data was discarded. Silently missing
        scan data can invalidate an experiment as thoroughly as bad data can,
        so it has to land in the one record an analyst reads afterwards.

        Returns the names that raised. Guarding each call individually is the
        fix for the defect where emergency_stop() called stop() unguarded in
        a loop: GocatorScanner.stop() raised when disconnected, aborting the
        loop, so every subsystem registered after it was never stopped.
        """
        names = list(order) + [n for n in self._subsystems if n not in order]
        failed: List[str] = []
        for name in names:
            subsystem = self._subsystems.get(name)
            if subsystem is None:
                continue
            action = getattr(subsystem, verb, None)
            if action is None:
                continue
            try:
                note = action()
            except Exception as exc:
                failed.append(name)
                logger.error("%s.%s() failed: %s", name, verb, exc)
                self.event_log.log(
                    self.clock.elapsed(), name, verb, result=f"error: {exc}"
                )
                continue
            if note:
                logger.warning("%s.%s(): %s", name, verb, note)
                self.event_log.log(
                    self.clock.elapsed(), name, verb, result="ok", notes=str(note)
                )
        return failed

    def _on_safety_trigger(self, tier: SafetyTier, name: str) -> None:
        """Dispatch a monitor trip to the matching verb."""
        {
            SafetyTier.PAUSE: lambda: self.pause(reason=f"trigger:{name}"),
            SafetyTier.STOP: lambda: self.end_run(reason=f"trigger:{name}"),
            SafetyTier.ESTOP: lambda: self.estop(reason=f"trigger:{name}"),
        }[tier]()

    def log_note(self, text: str, refers_to: Optional[int] = None) -> int:
        """Add a free-text note to the event log — for a human to explain
        what happened, alongside the automatic narrative every other
        event-log row records.

        The event log ships as metadata alongside published experiment
        data, so this is the place to record anything an automated
        subsystem action can't say for itself: why a run was paused, what
        a sensor glitch looked like, a decision made mid-experiment. Call
        it any time — pausing/stopping first is not required.

        Args:
            text: The note itself.
            refers_to: Optional event_id (returned by this method or any
                subsystem's log_event()) this note explains — e.g. the
                event_id of a failed scan you're annotating with the cause.

        Returns:
            This note's own event_id, so a later note can refer back to it.
        """
        return self.event_log.log(
            self.clock.elapsed(), "operator", "note", notes=text, refers_to=refers_to
        )

    def pause(self, reason: str = "manual") -> None:
        """Pause the experiment — everything quiesces, nothing disconnects.

        Halts the scheduler, pauses every subsystem, and **pauses the
        experiment clock**, so runtime measures time under experimental
        conditions rather than wall time. Recover with :meth:`resume_from_pause`.

        Anything a subsystem discarded to get here — a part-finished scan,
        most importantly — is written to the event log by
        :meth:`_for_each_subsystem`.

        Refuses to downgrade an active ESTOPPED state — a pause is milder
        than an estop, and something already decided the rig needed the
        harder stop. Call :meth:`rearm` first.
        """
        if self._safety_state is SafetyState.ESTOPPED:
            logger.error(
                "Refusing to pause: the rig is ESTOPPED. Call rearm() first."
            )
            return
        logger.info("Pausing experiment (%s)...", reason)
        self.event_log.log(
            self.clock.elapsed(), "flume_lab", "experiment_pause", notes=reason
        )
        self.scheduler.stop()
        self._for_each_subsystem("pause")
        if self.clock.is_running and not self.clock.is_paused:
            self.clock.pause()
        self._safety_state = SafetyState.PAUSED

    def resume_from_pause(self) -> bool:
        """Undo :meth:`pause` — restart the clock and restore setpoints.

        Refuses while a pause trigger is still asserted, so the rig cannot be
        resumed straight back into whatever tripped it. Clear the sentinel
        (or fix the failing health check) first.

        Named to avoid colliding with :meth:`resume`, which resumes the
        *scheduler* for the remainder of a run and predates this.
        """
        if self._safety_state is SafetyState.ESTOPPED:
            raise RuntimeError(
                "Cannot resume from an estop — call rearm() instead, once the "
                "condition that tripped it has been cleared."
            )
        still = self.safety_monitor.tripped_by()
        if still is not None:
            logger.error(
                "Refusing to resume: safety trigger %r is still asserted. "
                "Clear it first (%s).", still, self.safety_monitor.hint_for(still),
            )
            return False

        logger.info("Resuming from pause...")
        if self.clock.is_paused:
            self.clock.resume()
        self._for_each_subsystem("resume")
        self.safety_monitor.rearm()
        self.event_log.log(self.clock.elapsed(), "flume_lab", "experiment_resume")
        self._safety_state = SafetyState.RUNNING
        return True

    def end_run(self, reason: str = "manual") -> None:
        """Stop cleanly — quiesce everything into a disconnectable state.

        The middle tier: harder than a pause (not resumable), gentler than an
        estop (controlled deceleration, no stalling against brakes). Named
        end_run() because ``FlumeLab.stop()`` has always meant "pause" to
        existing scripts and to run_blocking()'s signal handlers.

        Refuses to downgrade an active ESTOPPED state, for the same reason
        as :meth:`pause`: an estop is the more severe tier, and something
        already decided the rig needed it. Call :meth:`rearm` first.
        """
        if self._safety_state is SafetyState.ESTOPPED:
            logger.error(
                "Refusing to end the run: the rig is ESTOPPED. Call rearm() first."
            )
            return
        logger.info("Stopping run (%s)...", reason)
        self.event_log.log(
            self.clock.elapsed(), "flume_lab", "experiment_stop_requested", notes=reason
        )
        self.scheduler.stop()
        self._for_each_subsystem("stop", order=self._ESTOP_ORDER)
        if self.clock.is_running and not self.clock.is_paused:
            self.clock.pause()
        self._safety_state = SafetyState.STOPPED

    def estop(self, reason: str = "manual") -> None:
        """Emergency stop — bring everything to a halt as fast as possible.

        Gantry zero-decel abort with brakes engaged and motors disabled, pump
        off, both valves closed, acquisition aborted and any partial scan
        discarded. Motion is stopped first, then hydraulics.

        Never raises and never stops early: each subsystem is guarded
        individually, so one that fails or is disconnected cannot prevent the
        others from being made safe.

        Leaves the rig in :attr:`SafetyState.ESTOPPED`. Recover with
        :meth:`rearm`, which refuses while a trigger is still asserted.
        """
        logger.critical("EMERGENCY STOP (%s)", reason)
        self.event_log.log(
            self.clock.elapsed(), "flume_lab", "emergency_stop", notes=reason
        )
        try:
            self.scheduler.stop()
        except Exception as exc:
            logger.error("Could not stop the scheduler during estop: %s", exc)

        failed = self._for_each_subsystem("estop", order=self._ESTOP_ORDER)
        if self.clock.is_running and not self.clock.is_paused:
            self.clock.pause()
        self._safety_state = SafetyState.ESTOPPED
        if failed:
            logger.error(
                "ESTOP completed, but these subsystems reported errors: %s. "
                "Verify the hardware physically before re-arming.",
                ", ".join(failed),
            )

    def rearm(self) -> bool:
        """Return from ESTOPPED to RUNNING, if it is safe to do so.

        Refuses while any trigger is still asserted — a sentinel still on
        disk, the VFD's hardware e-stop still latched — so the rig cannot be
        brought back up into a live emergency. Clear the cause first.

        Re-enables motors and releases brakes via the gantry's own
        ``set_safe_mode(False)`` path, which does motor-on then brake-release
        in that order and never the reverse.

        If this cannot clear the controller — a PLC that needed a power cycle
        — fall back to a full ``disconnect_all()``/``connect_all()``, noting
        that a reconnect loses the gantry's position reference unless
        ``restore_last_position()`` is used (see issue #23).
        """
        tripped = self.safety_monitor.tripped_by()
        if tripped is not None:
            logger.error(
                "Refusing to re-arm: safety trigger %r is still asserted. "
                "Clear it first (%s).", tripped, self.safety_monitor.hint_for(tripped),
            )
            return False

        logger.warning("Re-arming after emergency stop...")
        gantry = self._subsystems.get("gantry")
        ok = True
        if gantry is not None and hasattr(gantry, "set_safe_mode"):
            try:
                ok = bool(gantry.set_safe_mode(False))
            except Exception as exc:
                logger.error("Could not re-arm the gantry: %s", exc)
                ok = False

        self.safety_monitor.rearm()
        self._safety_state = SafetyState.RUNNING if ok else SafetyState.ESTOPPED
        self.event_log.log(
            self.clock.elapsed(), "flume_lab", "rearm",
            result="ok" if ok else "failed",
        )
        if not ok:
            logger.error(
                "Re-arm incomplete — the rig is still ESTOPPED. Try a full "
                "disconnect_all()/connect_all(); if the gantry lost its "
                "position reference, see restore_last_position()."
            )
        return ok

    def escalate(self, problem: str, tier: SafetyTier = SafetyTier.PAUSE) -> None:
        """Bring the experiment down because something went wrong elsewhere.

        The hook for "this is worse than one failed action". A scan that could
        not run, an instrument that has gone unreachable, or a scheduled move
        that found the gantry already busy all mean the scripted plan is no
        longer being followed — and letting the run continue collects
        perishable data under conditions nobody recorded.

        Defaults to a pause, which is recoverable once the cause is fixed.
        """
        logger.error("Escalating to %s: %s", tier.name, problem)
        self._on_safety_trigger(tier, problem)

    def watch_for_safety(
        self,
        sentinels: Optional[dict] = None,
        extra_triggers: Optional[list] = None,
    ) -> "FlumeLab":
        """Start watching for externally-demanded pause / stop / estop.

        All three tiers, not just estop: a health check that notices the
        scanner has stopped returning surfaces, or a camera has stopped
        producing frames, can ``touch PAUSE`` and halt the run gracefully
        before more perishable data is lost — no code changes needed.

        Polls on a background thread rather than using a signal handler:
        Python delivers signals only in the main thread between bytecodes, so
        a signal cannot interrupt a blocking serial read or SDK call.

        Args:
            sentinels: ``{tier_name: path}``, defaulting to
                ``./PAUSE``, ``./STOP``, ``./ESTOP``. Pass ``{}`` to skip.
            extra_triggers: Additional trigger objects (see laguna.safety).

        Returns:
            self, so this chains off the constructor.
        """
        paths = DEFAULT_SENTINELS if sentinels is None else sentinels
        for tier_name, path in (paths or {}).items():
            self.safety_monitor.add(
                SentinelFileTrigger(path, tier=SafetyTier(tier_name))
            )
        for trigger in extra_triggers or []:
            self.safety_monitor.add(trigger)
        # The flow controller's VFD reports a real hardware e-stop circuit;
        # follow it rather than pretending software is the only authority.
        flow = self._subsystems.get("flow")
        if flow is not None:
            self.safety_monitor.add(
                CallableTrigger(
                    lambda: bool(flow.get_status().get("vfd_estop", False)),
                    tier=SafetyTier.ESTOP,
                    name="vfd_hardware_estop",
                )
            )
        self.safety_monitor.start()
        return self

    def emergency_stop(self) -> None:
        """Deprecated alias for :meth:`estop`, kept for existing scripts.

        The old behaviour also disconnected everything; estop() deliberately
        does not, so the rig can be inspected and re-armed without losing the
        gantry's position reference.
        """
        self.estop(reason="emergency_stop() alias")

    def open_ocean_control_gui(self, gui_script_path: Optional[str] = None) -> None:
        """Launch the OceanControl GUI as a subprocess."""
        import subprocess
        import sys

        configured_path = self.config.get_value("ocean_control.gui_path")
        if gui_script_path is None:
            gui_script_path = configured_path if isinstance(configured_path, str) else None

        path = gui_script_path or (
            "/home/eric/Desktop/safl-ocean-control/OceanControl/Python Controls/"
            "SAFL_OceanControl.py"
        )
        path = Path(path)
        if not path.exists():
            logger.warning(f"OceanControl GUI not found at {path} — skipping launch")
            return
        logger.info(f"Launching OceanControl GUI: {path}")
        subprocess.Popen([sys.executable, str(path)], cwd=str(path.parent))
