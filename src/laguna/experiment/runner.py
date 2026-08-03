"""Experiment setup and blocking-run utilities."""

import logging
import os
import signal
import threading
from pathlib import Path
from typing import Callable, Optional

import yaml

from laguna import FlumeLab
from laguna.schedule import ExperimentSchedule

logger = logging.getLogger(__name__)


# ------------------------------------------------------------------ #
# Internal helpers                                                     #
# ------------------------------------------------------------------ #

def _load_yaml(config_path: str) -> dict:
    try:
        with open(config_path) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        logger.warning("Config file %s not found", config_path)
        return {}
    except Exception as e:
        logger.warning("Could not load config %s: %s", config_path, e)
        return {}


def _validate_trigger_config(section_name: str, cfg: dict) -> None:
    """Raise ValueError if conflicting scheduling options are specified."""
    use = cfg.get("use_schedule", False)
    has_interval = "interval_s" in cfg
    has_at = "trigger_at" in cfg
    if use and has_interval:
        raise ValueError(f"[{section_name}]: 'use_schedule' and 'interval_s' are mutually exclusive")
    if use and has_at:
        raise ValueError(f"[{section_name}]: 'use_schedule' and 'trigger_at' are mutually exclusive")
    if has_interval and has_at:
        raise ValueError(f"[{section_name}]: 'interval_s' and 'trigger_at' are mutually exclusive")


def _register_action(
    lab: FlumeLab,
    cfg: dict,
    subsystem: str,
    name: str,
    action: Optional[Callable] = None,
    action_factory: Optional[Callable] = None,
    exp_schedule: Optional[ExperimentSchedule] = None,
    schedule_col: Optional[str] = None,
) -> None:
    """Register a scheduled action via interval_s, trigger_at, or use_schedule.

    Works for any subsystem — cameras, sensors, actuators, or future additions.

    Scheduling modes (cfg keys, mutually exclusive):
      interval_s: N    — action fires every N runtime seconds
      trigger_at: [t]  — action fires once at each listed runtime second
      use_schedule: true — action fires at schedule CSV time points

    For use_schedule with a fixed action (cameras, sensors):
      - If schedule_col is present in the CSV: fire only at truthy rows.
      - If schedule_col is absent: fire at every time_s row.

    For use_schedule with an action that depends on the scheduled value (actuators):
      - Pass action_factory(t_s: float) -> Callable instead of action.
      - Fires at every time_s row; each call uses action_factory to build a
        closure capturing the target value at that time.
    """
    use_sched = cfg.get("use_schedule", False)

    if use_sched:
        if exp_schedule is None:
            logger.warning("[%s] use_schedule=true but no schedule provided — skipping", subsystem)
            return
        df = exp_schedule._df
        if schedule_col and schedule_col in df.columns:
            times = df.loc[df[schedule_col].astype(bool), "time_s"].tolist()
        else:
            times = df["time_s"].tolist()
        make = action_factory or (lambda t: action)
        for t in times:
            lab.scheduler.at(float(t), make(float(t)), subsystem=subsystem, name=name)

    elif "interval_s" in cfg:
        lab.scheduler.repeat(
            every=cfg["interval_s"], action=action, subsystem=subsystem, name=name
        )

    elif "trigger_at" in cfg:
        for t in cfg["trigger_at"]:
            lab.scheduler.at(float(t), action, subsystem=subsystem, name=name)


# ------------------------------------------------------------------ #
# Public API                                                           #
# ------------------------------------------------------------------ #

def setup_run(
    lab_config: str,
    schedule: Optional[str] = None,
    verbose_cameras: bool = False,
    simulate: bool = False,
    speed_factor: Optional[float] = None,
) -> FlumeLab:
    """Set up a FlumeLab from experiment_config.yaml.

    Instantiates only the subsystems whose sections are present in the YAML.
    Registers scheduled actions with intervals or trigger times from the YAML.

    Scheduling options per section (mutually exclusive):
      interval_s: N         — fire every N seconds
      trigger_at: [t1, t2]  — fire at these runtime seconds
      use_schedule: true    — fire at schedule CSV time points; actuator sections
                              read their setpoint from the matching interpolated
                              column; camera sections filter by an optional column
                              named after the YAML key (e.g. pi_cameras: 1/0)

    Args:
        lab_config: Path to experiment_config.yaml.
        schedule: Optional path to schedule CSV. Required if any section has
                  use_schedule: true. Only time_s is required; actuator columns
                  (weir_elevation_mm, pump_flow_lpm, qin_open, qaux_open) and
                  camera trigger columns (pi_cameras, dslr_cameras) are optional.
        verbose_cameras: Show full paramiko / SSH progress during Pi captures.
        simulate: Rehearse with no hardware attached — see laguna.simulation.
                  Only gantry/gocator have a simulated backend; every other
                  section (weir, flow, gauge, cameras, rangefinders) is
                  dropped rather than connecting to real hardware.
        speed_factor: Experiment seconds per real second. Only valid with
                      simulate=True (see FlumeLab.__init__).

    Returns:
        Configured FlumeLab. Call lab.start(duration) to begin.
    """
    cfg = _load_yaml(lab_config)
    lab = FlumeLab(lab_config, simulate=simulate, speed_factor=speed_factor)
    if simulate:
        # setup_run() reads its own local `cfg` (raw YAML, not merged with
        # Config's defaults) to decide which subsystems to construct — see
        # the `if "weir" in cfg` pattern below. FlumeLab.__init__ already
        # simulated lab.config.config_dict for lab.config.get(...) callers
        # (acquire_scan(), survey passes), but that is a different dict; this
        # `cfg` needs the same rewrite or the sections dropped there would
        # still get built here, real hardware and all.
        from laguna.simulation import simulate_config

        cfg = simulate_config(cfg)

    if not verbose_cameras:
        logging.getLogger("paramiko").setLevel(logging.WARNING)
        logging.getLogger("paramiko.transport").setLevel(logging.WARNING)
        logging.getLogger("laguna.camera.network").setLevel(logging.INFO)

    # Load schedule
    exp_schedule: Optional[ExperimentSchedule] = None
    if schedule is not None:
        exp_schedule = ExperimentSchedule.from_csv(schedule)
        logger.info("Loaded schedule from %s (%d time points)", schedule,
                    len(exp_schedule._df))

    # Validate scheduling config for all present sections. "gantry" has no
    # scheduled action of its own — it only ever moves as part of a scan —
    # but interval_s/trigger_at/use_schedule under a gantry: block would be
    # silently ignored without this, so it's validated here too rather than
    # left to fail confusingly later.
    for section in ("gauge", "weir", "flow", "pi_cameras", "dslr_cameras",
                    "gantry", "gocator"):
        if section in cfg:
            _validate_trigger_config(section, cfg[section])

    # Warn if use_schedule sections lack a schedule CSV
    for section in ("weir", "flow"):
        if cfg.get(section, {}).get("use_schedule") and exp_schedule is None:
            logger.warning(
                "[%s] use_schedule=true but no schedule CSV provided — "
                "will be connected in read-only mode", section,
            )

    # ------------------------------------------------------------------ #
    # Instantiate subsystems (only those present in YAML)                 #
    # ------------------------------------------------------------------ #

    gauge = None
    weir = None
    flow = None
    pi_array = None
    pi_output_dir = None
    dslr = None

    if "gauge" in cfg:
        from laguna.gauge import SaflWaterLevelSensor
        gauge = SaflWaterLevelSensor(cfg["gauge"])
        lab.add(gauge)

    if "weir" in cfg:
        from laguna.weir import SaflWeirController
        weir = SaflWeirController(cfg["weir"])
        lab.add(weir)

    if "flow" in cfg:
        from laguna.flow import SaflFlowController
        flow = SaflFlowController(cfg["flow"])
        lab.add(flow)

    if "pi_cameras" in cfg:
        pi_cfg = cfg["pi_cameras"]
        from laguna.camera.network import CameraArray
        pi_array = CameraArray(
            hosts=pi_cfg.get("hosts", []),
            ssh_user=pi_cfg.get("ssh_user", "ucrs"),
            ssh_key=pi_cfg.get("ssh_key"),
        )
        pi_output_dir = Path(pi_cfg.get("output_dir", "./captures/pi"))
        pi_output_dir.mkdir(parents=True, exist_ok=True)
        lab.add(pi_array)

    if "dslr_cameras" in cfg:
        from laguna.camera import DslrCameraSubsystem
        dslr = DslrCameraSubsystem.from_dict(
            config=cfg["dslr_cameras"],
            main_yaml_path=lab_config,
        )
        lab.add(dslr)

    # The survey half of the rig. Until now gantry/gocator were hand-scripted
    # in examples only — they were absent from this function entirely, so a
    # scan could not be part of a scheduled experiment. See
    # docs/COSCRIPTING_ROADMAP.md.
    gantry = None
    if "gantry" in cfg:
        from laguna.robot.macron.controller import GantryController
        gantry = GantryController.from_config(cfg["gantry"])
        lab.add(gantry)

    gocator = None
    if "gocator" in cfg:
        from laguna.scanner import GocatorScanner
        gocator = GocatorScanner.from_config(cfg["gocator"])
        if lab.run.root is not None:
            # Route scans under the run directory rather than the bare
            # configured output_dir, so a run's whole output tree is one
            # self-contained artifact — see laguna.run_context.
            gocator._output_dir = lab.run.path_for("gocator", str(gocator._output_dir))
        lab.add(gocator)

    # ------------------------------------------------------------------ #
    # Connect                                                              #
    # ------------------------------------------------------------------ #

    logger.info("Connecting subsystems...")
    for sub_name, subsystem in lab._subsystems.items():
        ok = subsystem.connect()
        if ok:
            logger.info("  %-14s connected", sub_name)
        else:
            logger.warning("  %-14s FAILED — continuing without it", sub_name)

    if gauge is not None and gauge._is_connected:
        try:
            gauge.read_mm()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Define action closures                                               #
    # ------------------------------------------------------------------ #

    def _log_gauge():
        try:
            gauge.connect()
            elev_mm = gauge.read_mm()
            logger.info("Water level: %.2f mm", elev_mm)
            lab.event_log.log(lab.clock.elapsed(), "gauge", "read_mm",
                              f"elevation_mm={elev_mm:.2f}")
        except Exception as e:
            logger.error("Gauge read failed: %s", e)
            raise

    def _log_weir_status():
        try:
            weir.connect()
            status = weir.get_status()
            motor = status.get("motor", {})
            elev = status.get("elevation_mm")
            logger.info("Weir: elevation=%.2f mm  enabled=%s  fault=%s  steps=%s",
                        elev if elev is not None else float("nan"),
                        motor.get("Enabled"), motor.get("MotorInFault"), motor.get("StepsActive"))
            lab.event_log.log(lab.clock.elapsed(), "weir", "get_status",
                              f"elevation_mm={elev} enabled={motor.get('Enabled')} "
                              f"fault={motor.get('MotorInFault')}")
        except Exception as e:
            logger.error("Weir status query failed: %s", e)
            raise

    def _make_update_weir(t_s: float) -> Callable:
        def _update_weir():
            try:
                weir.connect()
                target_mm = exp_schedule.weir_elevation(t_s)
                logger.info("Weir -> %.2f mm  (t=%.0f s)", target_mm, t_s)
                weir.go_to_elevation(target_mm)
                lab.event_log.log(lab.clock.elapsed(), "weir", "update_elevation",
                                  f"target_mm={target_mm:.2f}")
            except Exception as e:
                logger.error("Weir update failed: %s", e)
                raise
        return _update_weir

    def _log_flow_status():
        try:
            status = flow.get_status()
            logger.info("Flow: %.2f lpm  qin=%s qaux=%s",
                        status.get("flowrate_lpm", 0),
                        status.get("qin_open"), status.get("qaux_open"))
            lab.event_log.log(lab.clock.elapsed(), "flow", "get_status",
                              f"flowrate_lpm={status.get('flowrate_lpm', 0):.2f}")
        except Exception as e:
            logger.error("Flow status query failed: %s", e)
            raise

    def _make_update_flow(t_s: float) -> Callable:
        def _update_flow():
            try:
                df = exp_schedule._df
                lpm = float(exp_schedule.pump_flow(t_s)) if "pump_flow_lpm" in df.columns else 0.0
                qin = bool(exp_schedule.qin_open(t_s)) if "qin_open" in df.columns else False
                qaux = bool(exp_schedule.qaux_open(t_s)) if "qaux_open" in df.columns else False
                flow.set_flowrate(lpm)
                flow.start() if lpm > 0 else flow.stop()
                flow.qin = qin
                flow.qaux = qaux
                logger.info("Flow: %.2f lpm  qin=%s qaux=%s  (t=%.0f s)", lpm, qin, qaux, t_s)
                lab.event_log.log(lab.clock.elapsed(), "flow", "update",
                                  f"flowrate_lpm={lpm:.2f} qin={qin} qaux={qaux}")
            except Exception as e:
                logger.error("Flow update failed: %s", e)
                raise
        return _update_flow

    def _capture_pi():
        try:
            from laguna.camera.network import assess_spread
            logger.info("Pi cameras: triggering capture...")
            results = pi_array.trigger_capture()
            n_ok = sum(1 for r in results if r.success)
            times_mid = [r.capture_time_mid_pc for r in results
                         if r.success and r.capture_time_mid_pc is not None]
            if len(times_mid) >= 2:
                spread_ms = (max(times_mid) - min(times_mid)) * 1000
                logger.info("Pi cameras: %d/%d  spread=%.1f ms — %s",
                            n_ok, len(results), spread_ms, assess_spread(spread_ms))
            else:
                logger.info("Pi cameras: %d/%d succeeded", n_ok, len(results))
            for r in results:
                if not r.success:
                    logger.warning("  %s FAILED: %s", r.hostname, r.error)
            fetched = pi_array.fetch_images(results, pi_output_dir)
            for host, path in fetched.items():
                logger.info("  %s -> %s", host, Path(path).name)
                lab.event_log.log(lab.clock.elapsed(), "pi_cameras", "capture",
                                  f"host={host} file={path}")
            if n_ok < len(results):
                failed = [r.hostname for r in results if not r.success]
                lab.event_log.log(lab.clock.elapsed(), "pi_cameras", "capture_partial_failure",
                                  f"failed={failed}")
        except Exception as e:
            logger.error("Pi camera capture failed: %s", e)
            raise

    def _capture_dslr():
        try:
            logger.info("DSLR cameras: triggering capture...")
            results = dslr.capture_all()
            for cam_name, path in results.items():
                if path:
                    logger.info("  %s -> %s", cam_name, Path(path).name)
                    lab.event_log.log(lab.clock.elapsed(), "dslr_cameras", "capture",
                                      f"camera={cam_name} file={path}")
                else:
                    logger.warning("  %s FAILED", cam_name)
                    lab.event_log.log(lab.clock.elapsed(), "dslr_cameras", "capture_failed",
                                      f"camera={cam_name}")
        except Exception as e:
            logger.error("DSLR capture failed: %s", e)
            raise

    # ------------------------------------------------------------------ #
    # Register scheduled actions                                           #
    # ------------------------------------------------------------------ #

    if gauge is not None:
        _register_action(lab, cfg.get("gauge", {}), "gauge", "read_mm", action=_log_gauge)

    if weir is not None:
        weir_cfg = cfg.get("weir", {})
        schedule_has_weir = (exp_schedule is not None
                             and "weir_elevation_mm" in exp_schedule._df.columns)
        if weir_cfg.get("use_schedule"):
            if not schedule_has_weir:
                logger.warning("weir.use_schedule=true but CSV has no weir_elevation_mm — read-only")
                _register_action(lab, weir_cfg, "weir", "get_status", action=_log_weir_status)
            else:
                _register_action(lab, weir_cfg, "weir", "update_elevation",
                                 action_factory=_make_update_weir, exp_schedule=exp_schedule)
        else:
            _register_action(lab, weir_cfg, "weir", "get_status", action=_log_weir_status)

    if flow is not None:
        flow_cfg = cfg.get("flow", {})
        schedule_has_flow = (exp_schedule is not None and any(
            c in exp_schedule._df.columns for c in ("pump_flow_lpm", "qin_open", "qaux_open")
        ))
        if flow_cfg.get("use_schedule"):
            if not schedule_has_flow:
                logger.warning("flow.use_schedule=true but CSV has no flow columns — read-only")
                _register_action(lab, flow_cfg, "flow", "get_status", action=_log_flow_status)
            else:
                _register_action(lab, flow_cfg, "flow", "update_flow",
                                 action_factory=_make_update_flow, exp_schedule=exp_schedule)
        else:
            _register_action(lab, flow_cfg, "flow", "get_status", action=_log_flow_status)

    if pi_array is not None:
        _register_action(lab, cfg.get("pi_cameras", {}), "pi_cameras", "capture",
                         action=_capture_pi, exp_schedule=exp_schedule, schedule_col="pi_cameras")

    if dslr is not None:
        _register_action(lab, cfg.get("dslr_cameras", {}), "dslr_cameras", "capture",
                         action=_capture_dslr, exp_schedule=exp_schedule, schedule_col="dslr_cameras")

    def _scan_gocator():
        """One coordinated Gocator pass, logged to the event log.

        Never raises: a failed scan must not take down a running experiment
        that is also driving hydraulics and cameras. The scheduler would log
        the exception anyway, but then the event log would carry no record of
        what was attempted.
        """
        from laguna.robot.motion_arbiter import MotionBusyError
        from laguna.scanner import ScanNotPossibleError

        # One runtime_s for this whole scan, taken at the start — a pass
        # takes real time, so "start" and "completion" would otherwise be
        # two different numbers describing the same scan depending which
        # artifact you read (scan.metadata's stamp vs. run.json's outputs
        # vs. the event log). This has to be the start time specifically
        # because save_scan() reads gocator._run_stamp mid-acquire() to
        # build the filename and embed it in the surface's own metadata —
        # it cannot be computed after the fact.
        runtime_s = lab.clock.elapsed()
        gocator._run_stamp = lab.run.stamp(runtime_s)
        try:
            scan = gocator.acquire(gantry=gantry)
        except (ScanNotPossibleError, MotionBusyError) as exc:
            # Not a skippable hiccup. Either the scanner cannot collect at
            # all, or something moved the gantry outside the scripted plan —
            # both mean the run is no longer doing what it was told, and
            # continuing just accumulates data under unrecorded conditions.
            lab.event_log.log(runtime_s, "gocator", "scan", result=f"error: {exc}")
            lab.escalate(f"gocator scan could not run: {exc}")
            return
        except Exception as exc:
            lab.event_log.log(runtime_s, "gocator", "scan", result=f"error: {exc}")
            lab.escalate(f"gocator scan failed unexpectedly: {exc}")
            return
        if scan is None:
            return
        path = getattr(gocator, "_last_saved_path", None)
        if path:
            lab.run.record_output("gocator", path, runtime_s, points=scan.valid_count)
        lab.event_log.log(
            runtime_s, "gocator", "scan",
            result=f"points={scan.valid_count}",
            notes=f"file={path}" if path else "",
        )

    if gocator is not None:
        _register_action(lab, cfg.get("gocator", {}), "gocator", "scan",
                         action=_scan_gocator, exp_schedule=exp_schedule,
                         schedule_col="gocator")

    return lab


def run_blocking(lab: FlumeLab, duration: float) -> None:
    """Run lab.start(duration) with signal-based pause/resume control.

    Intended for non-interactive CLI use. Prints PID at startup so the user
    can send signals from another terminal. Writes .experiment.pid to the
    current directory (removed on exit).

    Control:
      First Ctrl+C          — pause (hardware stays connected)
      Second Ctrl+C         — stop and disconnect
      kill -USR1 <pid>      — pause from another terminal
      kill -USR2 <pid>      — resume from another terminal
      kill <pid> (SIGTERM)  — stop and disconnect
      tail -f experiment_events.csv — live status from another terminal
    """
    pid = os.getpid()
    pid_file = Path(".experiment.pid")
    pid_file.write_text(str(pid))

    print(f"\nExperiment running — PID {pid}")
    print(f"  Ctrl+C once              — pause (hardware stays connected)")
    print(f"  Ctrl+C twice             — stop and disconnect")
    print(f"  kill -USR1 {pid}  — pause from another terminal")
    print(f"  kill -USR2 {pid}  — resume from another terminal")
    print(f"  tail -f experiment_events.csv — live status\n")

    _paused = False
    _sigint_count = 0
    _resume_event = threading.Event()

    def _on_sigint(sig, frame):
        nonlocal _paused, _sigint_count
        _sigint_count += 1
        if _sigint_count == 1:
            _paused = True
            lab.stop()
            print(f"\nPaused (Ctrl+C again to exit | kill -USR2 {pid} to resume)")
        else:
            raise SystemExit(0)

    def _on_pause(sig, frame):
        nonlocal _paused
        _paused = True
        lab.stop()
        print(f"\nPaused (kill -USR2 {pid} to resume)")

    def _on_resume(sig, frame):
        _resume_event.set()

    def _on_term(sig, frame):
        raise SystemExit(0)

    signal.signal(signal.SIGINT,  _on_sigint)
    signal.signal(signal.SIGUSR1, _on_pause)
    signal.signal(signal.SIGUSR2, _on_resume)
    signal.signal(signal.SIGTERM, _on_term)

    try:
        thread = lab.start(duration)
        while True:
            thread.join(timeout=0.5)
            if not thread.is_alive():
                if _paused:
                    _resume_event.wait()        # Ctrl+C (second) raises SystemExit here
                    _resume_event.clear()
                    _paused = False
                    _sigint_count = 0
                    print("Resuming...")
                    thread = lab.resume()       # uses stored _duration - elapsed
                else:
                    break
    finally:
        pid_file.unlink(missing_ok=True)
        lab.disconnect_all()
