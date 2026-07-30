"""Core orchestrator module for FlumeLab.

FlumeLab uses an opt-in model: instantiate subsystems separately and attach them
with lab.add(subsystem). This avoids hardcoding hardware assumptions in the core.
"""
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict, Iterator, Optional
import logging
import threading
import time

from .config import Config
from .timing import CheckpointStore, EventLog, ExperimentClock, Scheduler

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class FlumeLab:
    """Main orchestrator for the flume lab robotic system.

    Creates a timing backbone immediately; hardware subsystems are registered
    explicitly via ``lab.add(subsystem)``.

    Example::

        lab = FlumeLab("config.yaml")
        lab.add(CameraManager(configs)).add(GantryController.from_config(lab.config.get("gantry")))
        lab.connect_all()
        with lab.experiment() as clock:
            lab.scheduler.run(duration=300)

    Attributes:
        clock:      Experiment clock (wall time + runtime)
        scheduler:  Action scheduler tied to the experiment clock
        event_log:  Append-only CSV event log
    """

    def __init__(self, config_file: Optional[str] = None) -> None:
        logger.info("Initializing FlumeLab system...")

        self.config = Config(config_file=config_file)

        # Timing subsystem — always present
        self.clock = ExperimentClock()
        self.event_log = EventLog(
            self.config.get_value("timing.event_log", "./experiment_events.csv")
        )
        self.scheduler = Scheduler(clock=self.clock, event_log=self.event_log)

        # Registry for opt-in hardware subsystems
        self._subsystems: Dict[str, Any] = {}

        self._duration: Optional[float] = None   # set by start(), used by resume()
        self._start_wall: Optional[float] = None  # wall time of lab.start()
        self.is_running = False
        logger.info("FlumeLab timing backbone ready — add subsystems via lab.add()")

    # ------------------------------------------------------------------
    # Opt-in subsystem registration
    # ------------------------------------------------------------------

    def add(self, subsystem: Any) -> "FlumeLab":
        """Register a hardware subsystem by its ``subsystem_name`` attribute.

        The subsystem is stored both in ``self._subsystems`` (keyed by name)
        and as a direct attribute (``self.<subsystem_name>``), making
        ``lab.cameras``, ``lab.gantry``, etc. work naturally.

        Args:
            subsystem: Any object with a ``subsystem_name`` class or instance
                       attribute (e.g. CameraManager, GantryController).

        Returns:
            self — so calls can be chained: ``lab.add(cam).add(robot)``
        """
        name = getattr(subsystem, "subsystem_name", None)
        if not name:
            raise ValueError(
                f"{type(subsystem).__name__} has no 'subsystem_name' attribute; "
                "cannot register as a FlumeLab subsystem."
            )
        self._subsystems[name] = subsystem
        setattr(self, name, subsystem)
        logger.info("Registered subsystem '%s' (%s)", name, type(subsystem).__name__)
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
        """Disconnect all registered subsystems."""
        logger.info("Disconnecting all subsystems...")
        for name, subsystem in self._subsystems.items():
            disconnector = getattr(subsystem, "disconnect", None) or getattr(subsystem, "stop", None)
            if disconnector:
                disconnector()
        self.is_running = False
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
        self.event_log.log(0.0, "flume_lab", "experiment_start")
        try:
            yield self.clock
        finally:
            self.clock.stop()
            self.event_log.log(self.clock.elapsed(), "flume_lab", "experiment_stop")
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
        self.event_log.log(0.0, "flume_lab", "experiment_start")
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
        """Pause the experiment: stop scheduler loop, pause clock, and stop weir.

        Unlike disconnect_all(), this does NOT close the event log or disconnect
        hardware — the experiment can be resumed with resume() or a fresh run().
        """
        logger.info("Stopping experiment (pausing clock and scheduler)...")
        self.event_log.log(self.clock.elapsed(), "flume_lab", "experiment_pause")
        self.scheduler.stop()
        weir = self._subsystems.get("weir")
        if weir and hasattr(weir, "stop"):
            weir.stop()

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

        gantry = self._subsystems.get("gantry")
        if gantry is None:
            raise RuntimeError("acquire_scan() requires a 'gantry' subsystem — lab.add(GantryController(...))")
        if end is None:
            raise ValueError("acquire_scan() requires end=[...]")
        if feed_rate_mm_s is None:
            raise ValueError("acquire_scan() requires feed_rate_mm_s — no default for a hardware move")

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

        profiler = TopographicProfiler(
            gantry=gantry,
            pi_host=gantry_config.get("host"),
            pi_user=gantry_config.get("ssh_user", "oak"),
            pi_key=gantry_config.get("ssh_key"),
            pdin_port=rf_config.get("pdin_port", 1),
            al1342_host=rf_config.get("al1342_host"),
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

    def emergency_stop(self) -> None:
        """Emergency stop — immediately shut down all registered systems."""
        logger.warning("EMERGENCY STOP activated!")
        for subsystem in self._subsystems.values():
            stopper = getattr(subsystem, "stop", None) or getattr(subsystem, "disconnect", None)
            if stopper:
                stopper()
        if self.clock.is_running and not self.clock.is_paused:
            self.clock.pause()
        self.disconnect_all()

    def open_ocean_control_gui(self, gui_script_path: str = None) -> None:
        """Launch the OceanControl GUI as a subprocess."""
        import subprocess
        import sys

        path = gui_script_path or self.config.get("ocean_control", {}).get(
            "gui_path",
            "/home/eric/Desktop/safl-ocean-control/OceanControl/Python Controls/SAFL_OceanControl.py",
        )
        path = Path(path)
        if not path.exists():
            logger.warning(f"OceanControl GUI not found at {path} — skipping launch")
            return
        logger.info(f"Launching OceanControl GUI: {path}")
        subprocess.Popen([sys.executable, str(path)], cwd=str(path.parent))
