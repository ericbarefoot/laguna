"""Tests for the config-driven subsystem registry: Config.explicit_sections,
FlumeLab.add(name)/add_all(), the attach_event_log() auto-wiring, and the
SubsystemLogging two-knob log_event() contract (see docs/CONTRIBUTING.md's
"Adding a New Subsystem" section for the from_config(config: Config) shape
this all rests on).
"""

import yaml
import pytest

from laguna import FlumeLab
from laguna.config import Config
from laguna.registry import SUBSYSTEM_REGISTRY
from laguna.subsystem_logging import SubsystemLogging


class TestConfigExplicitSections:
    def test_matches_yaml_top_level_keys(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text("weir:\n  port: /dev/ttyUSB0\ntiming:\n  event_log: ./e.csv\n")
        config = Config(config_file=str(path))
        assert config.explicit_sections == {"weir", "timing"}

    def test_defaults_only_config_has_no_explicit_sections(self):
        assert Config().explicit_sections == set()

    def test_config_file_path_is_recorded(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text("timing: {}\n")
        config = Config(config_file=str(path))
        assert config.config_file == str(path.resolve())

    def test_config_file_is_none_without_a_file(self):
        assert Config().config_file is None


class TestSimulateFiltersExplicitSections:
    def test_non_simulated_sections_are_dropped(self, tmp_path):
        """The specific bug this guards: simulate_config() drops sections
        with no simulated backend (rangefinders) from config_dict, but
        explicit_sections is a separate attribute — if FlumeLab.__init__
        didn't also filter it, add_all() would still try to build a real
        OD2000Rangefinder against real hardware under simulate=True (see
        laguna.simulation's module docstring). weir/flow/gauge/pi_cameras/
        dslr_cameras do have simulated backends now, so they're deliberately
        not the example here — see TestSimulateConfig in test_simulation.py
        for that coverage."""
        path = tmp_path / "cfg.yaml"
        path.write_text(
            "gantry:\n  axes: [{name: X, index: 1}]\n"
            "od2000:\n  topic: laguna/od2000\n"
        )
        lab = FlumeLab(str(path), simulate=True)
        assert lab.config.explicit_sections == {"gantry"}

    def test_add_all_under_simulate_never_touches_dropped_sections(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text(
            "gantry:\n  axes: [{name: X, index: 1}]\n"
            "od2000:\n  topic: laguna/od2000\n"
        )
        lab = FlumeLab(str(path), simulate=True)
        lab.add_all()
        assert "od2000" not in lab._subsystems
        assert "gantry" in lab._subsystems


class TestRegistry:
    def test_covers_the_documented_subsystems(self):
        assert set(SUBSYSTEM_REGISTRY) == {
            "gantry", "weir", "gauge", "flow", "gocator",
            "od2000", "wtt12l", "pi_cameras", "dslr_cameras",
        }

    def test_mqtt_is_deliberately_not_registered(self):
        """Each rangefinder builds its own private MqttSubscriber (see
        RangefinderSubsystem.from_config()) — a standalone lab.mqtt would be
        redundant and, under simulate=True, a real broker connection nothing
        reads (see registry.py's module docstring)."""
        assert "mqtt" not in SUBSYSTEM_REGISTRY


class TestAddByName:
    def test_looks_up_registry_and_builds_from_config(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text("weir:\n  port: /dev/ttyUSB0\n")
        lab = FlumeLab(str(path))
        lab.add("weir")
        assert "weir" in lab._subsystems
        assert lab.weir._port == "/dev/ttyUSB0"

    def test_unknown_name_raises(self):
        lab = FlumeLab()
        with pytest.raises(ValueError, match="registry"):
            lab.add("not_a_real_subsystem")

    def test_returns_self_for_chaining(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text("weir:\n  port: /dev/ttyUSB0\ngauge:\n  port: /dev/ttyUSB2\n")
        lab = FlumeLab(str(path))
        assert lab.add("weir").add("gauge") is lab


class TestAddAll:
    def _config_path(self, tmp_path, sections):
        path = tmp_path / "cfg.yaml"
        path.write_text(sections)
        return str(path)

    def test_builds_every_explicit_section_with_a_registry_entry(self, tmp_path):
        lab = FlumeLab(self._config_path(
            tmp_path, "weir:\n  port: /dev/ttyUSB0\ngauge:\n  port: /dev/ttyUSB2\ntiming: {}\n"
        ))
        lab.add_all()
        assert set(lab._subsystems) == {"weir", "gauge"}

    def test_sections_without_a_registry_entry_are_skipped(self, tmp_path):
        """'timing' is a real explicit_sections member but has no
        SUBSYSTEM_REGISTRY entry — add_all() must not raise for it."""
        lab = FlumeLab(self._config_path(tmp_path, "timing: {}\n"))
        lab.add_all()
        assert lab._subsystems == {}

    def test_no_config_file_means_add_all_adds_nothing(self):
        lab = FlumeLab()
        lab.add_all()
        assert lab._subsystems == {}


class TestFromConfigAcrossEveryRegistrySubsystem:
    def _full_config_path(self, tmp_path):
        cfg = {
            "gauge": {"port": "/dev/ttyUSB2"},
            "weir": {"port": "/dev/ttyUSB0"},
            "flow": {"vfd_port": "/dev/ttyUSB1"},
            "pi_cameras": {"hosts": ["pi1.local"]},
            "dslr_cameras": {"cameras": {"Cam1": {"port": "usb:001,001", "output_dir": "./out"}}},
            "mqtt": {"broker_host": "red.lab"},
            "od2000": {"topic": "laguna/od2000", "al1342_host": "192.168.1.251"},
            "wtt12l": {"topic": "laguna/wtt12l", "al1342_host": "192.168.1.251"},
            "gantry": {"axes": [{"name": "X", "index": 1}]},
            "gocator": {"ip": "192.168.1.10"},
        }
        path = tmp_path / "cfg.yaml"
        path.write_text(yaml.safe_dump(cfg))
        return str(path)

    def test_add_all_constructs_every_registry_subsystem(self, tmp_path):
        lab = FlumeLab(self._full_config_path(tmp_path))
        lab.add_all()
        assert set(lab._subsystems) == set(SUBSYSTEM_REGISTRY)

    def test_rangefinder_from_config_builds_its_own_mqtt_subscriber(self, tmp_path):
        """Each rangefinder gets a private MqttSubscriber built from the
        shared 'mqtt:' section — see RangefinderSubsystem.from_config()."""
        lab = FlumeLab(self._full_config_path(tmp_path))
        lab.add("od2000")
        assert lab.od2000._mqtt.__class__.__name__ == "MqttSubscriber"
        assert lab.od2000._mqtt._host == "red.lab"

    def test_two_rangefinders_get_independent_mqtt_subscribers(self, tmp_path):
        lab = FlumeLab(self._full_config_path(tmp_path))
        lab.add("od2000").add("wtt12l")
        assert lab.od2000._mqtt is not lab.wtt12l._mqtt
        assert lab.od2000._mqtt._client_id != lab.wtt12l._mqtt._client_id

    def test_dslr_from_config_resolves_output_dir_relative_to_the_yaml(self, tmp_path):
        lab = FlumeLab(self._full_config_path(tmp_path))
        lab.add("dslr_cameras")
        out_dir = lab.dslr_cameras._config["cameras"]["Cam1"]["output_dir"]
        assert out_dir == str((tmp_path / "out").resolve())

    def test_pi_cameras_without_explicit_hosts_targets_nothing(self, tmp_path):
        """A pi_cameras: section with some other key set but no hosts: must
        not silently default to real named lab cameras (DEFAULT_CAMERAS)."""
        path = tmp_path / "cfg.yaml"
        path.write_text("pi_cameras:\n  ssh_key: ~/.ssh/id_ed25519\n")
        lab = FlumeLab(str(path))
        lab.add("pi_cameras")
        assert lab.pi_cameras.hosts == []

    def test_dslr_from_config_requires_a_config_file(self):
        """No config_file means no YAML location to resolve output_dir
        paths against — must raise a clear error, not an obscure TypeError
        from Path(None)."""
        from laguna.camera.dslr import DslrCameraSubsystem

        config = Config(defaults={"dslr_cameras": {"cameras": {}}})
        with pytest.raises(ValueError, match="config_file"):
            DslrCameraSubsystem.from_config(config)


class TestEventLogAutoWiring:
    def test_add_wires_attach_event_log_when_present(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text("weir:\n  port: /dev/ttyUSB0\n")
        lab = FlumeLab(str(path))
        lab.add("weir")
        assert lab.weir._event_log is lab.event_log
        assert lab.weir._clock is lab.clock

    def test_add_does_not_crash_for_subsystems_without_the_hook(self, tmp_path):
        path = tmp_path / "cfg.yaml"
        path.write_text("gocator:\n  ip: 192.168.1.10\n")
        lab = FlumeLab(str(path))
        lab.add("gocator")  # GocatorScanner has no attach_event_log — must not raise
        assert "gocator" in lab._subsystems


class TestSubsystemLoggingMixin:
    class _Fake(SubsystemLogging):
        subsystem_name = "fake"

    class _FakeEventLog:
        def __init__(self):
            self.rows = []

        def log(self, runtime_s, subsystem, action, result="ok", notes=""):
            self.rows.append((runtime_s, subsystem, action, result, notes))

    class _FakeClock:
        def elapsed(self):
            return 12.5

    def test_log_event_noops_before_attach(self):
        fake = self._Fake()
        fake.log_event("did_a_thing")  # must not raise, must not need an event_log

    def test_log_event_writes_after_attach(self):
        fake = self._Fake()
        event_log = self._FakeEventLog()
        fake.attach_event_log(event_log, self._FakeClock())
        fake.log_event("did_a_thing", target_mm="5.00")
        assert event_log.rows == [(12.5, "fake", "did_a_thing", "ok", "target_mm=5.00")]

    def test_event_log_verbosity_filters_below_threshold(self):
        fake = self._Fake()
        fake.event_log_verbosity = "WARNING"
        event_log = self._FakeEventLog()
        fake.attach_event_log(event_log, self._FakeClock())

        fake.log_event("routine", level="INFO")
        assert event_log.rows == []

        fake.log_event("important", level="ERROR")
        assert len(event_log.rows) == 1

    def test_attach_sets_the_module_logger_level(self):
        import logging

        fake = self._Fake()
        fake.log_level = "DEBUG"
        fake.attach_event_log(self._FakeEventLog(), self._FakeClock())
        assert logging.getLogger(type(fake).__module__).level == logging.DEBUG
