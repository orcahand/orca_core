"""Guards on the operator scripts and examples: shared CLI surface, no hardcoding.

The Feetech bring-up broke on scripts that pinned one developer's device path,
one hand model, or one motor client class. These tests keep them wired to the
shared autodetection helpers instead.
"""

import argparse
import ast
import importlib.util
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# Scripts and examples that take a hand and must expose the shared flags.
HAND_CLI_MODULES = [
    "scripts/check_motor.py",
    "scripts/setup.py",
    "scripts/calibrate.py",
    "scripts/stress_test.py",
    "scripts/neutral.py",
    "scripts/tension.py",
    "scripts/zero.py",
    "scripts/manual_control.py",
    "examples/main_demo.py",
    "examples/main_demo_abduction.py",
    "examples/record_angles.py",
    "examples/record_continuous.py",
    "examples/replay_angles.py",
    "examples/replay_continuous.py",
]

# Every front-end whose source must stay free of hardcoded hardware identity.
NO_HARDCODING_MODULES = HAND_CLI_MODULES + [
    "scripts/detect.py",
    "scripts/check_sensors.py",
    "scripts/configure_motor_chain.py",
    "scripts/monitor_sensors.py",
    "examples/demo_runner.py",
    "examples/taxel_frames.py",
    "tools/extract_urdf_kinematics.py",
]

# Motor client classes a front-end must never name: the family is resolved from
# the config and the bus, not written into a script.
FORBIDDEN_IMPORTS = {
    "DynamixelClient",
    "FeetechClient",
    "MockDynamixelClient",
    "MockFeetechClient",
}

# The shared flags are only honoured through create_hand_from_args(args), so a
# hand-taking front-end must build its hand with it and nothing else.
HAND_BUILT_DIRECTLY = re.compile(r"\b(load_hand|create_hand)\s*\(")
FEEDBACK_PINNED = re.compile(r"create_hand_from_args\([^)]*engage_feedback\s*=")

# Hand classes must come from load_hand()/create_hand(), never be constructed.
HAND_CONSTRUCTION = re.compile(r"\b(Mock)?OrcaHand(Touch|Full|JointFeedback)?\s*\(")

DEVICE_PATH = re.compile(r"^/dev/")
MODEL_PATH = re.compile(r"models/v\d")


def _load(rel_path: str):
    path = REPO_ROOT / rel_path
    name = f"_script_{path.stem}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    # Scripts import their sibling helpers the way the interpreter runs them.
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(path.parent))
    return module


def _tree(rel_path: str) -> ast.Module:
    return ast.parse((REPO_ROOT / rel_path).read_text(encoding="utf-8"))


def _code_strings(tree: ast.Module) -> list[str]:
    """String literals that are code, not docstrings or argparse help text."""
    skip = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
            skip.add(id(node.value))
        elif isinstance(node, ast.Call):
            for keyword in node.keywords:
                if keyword.arg in ("help", "description"):
                    skip.add(id(keyword.value))
    return [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and id(node) not in skip
    ]


def _help_text(module, argv0: str, capsys) -> str:
    sys.argv = [argv0, "--help"]
    with pytest.raises(SystemExit) as exc:
        module.main()
    assert exc.value.code == 0
    return capsys.readouterr().out


@pytest.mark.parametrize("rel_path", HAND_CLI_MODULES)
def test_shared_hand_arguments_are_exposed(rel_path, capsys, monkeypatch):
    monkeypatch.setattr(sys, "argv", [rel_path, "--help"])
    module = _load(rel_path)
    help_text = _help_text(module, rel_path, capsys)

    assert "[config_path]" in help_text, "config_path must stay optional (autodetect)"
    assert "--mock" in help_text
    assert "--model-name" in help_text
    if "--no-engage-feedback" not in help_text:
        # Omitting it is only allowed when the front-end decides for itself.
        assert FEEDBACK_PINNED.search((REPO_ROOT / rel_path).read_text()), (
            f"{rel_path} drops --no-engage-feedback without pinning engage_feedback"
        )


@pytest.mark.parametrize("rel_path", HAND_CLI_MODULES)
def test_hand_is_built_from_the_parsed_arguments(rel_path):
    """A front-end that calls create_hand()/load_hand() itself silently drops the
    flags add_hand_arguments() advertised for it."""
    source = (REPO_ROOT / rel_path).read_text(encoding="utf-8")
    assert "create_hand_from_args(" in source, f"{rel_path} must build its hand from args"
    offenders = [
        line for line in source.splitlines()
        if HAND_BUILT_DIRECTLY.search(line) and not line.lstrip().startswith(("#", "*", ":"))
    ]
    assert not offenders, f"{rel_path} bypasses create_hand_from_args(): {offenders}"


@pytest.mark.parametrize("rel_path", NO_HARDCODING_MODULES)
def test_no_motor_client_class_is_imported(rel_path):
    imported = set()
    for node in ast.walk(_tree(rel_path)):
        if isinstance(node, ast.ImportFrom):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[-1] for alias in node.names)
    assert not imported & FORBIDDEN_IMPORTS


@pytest.mark.parametrize("rel_path", NO_HARDCODING_MODULES)
def test_no_hand_class_is_constructed_directly(rel_path):
    source = (REPO_ROOT / rel_path).read_text(encoding="utf-8")
    offenders = [
        line for line in source.splitlines()
        if HAND_CONSTRUCTION.search(line) and "isinstance" not in line
    ]
    assert not offenders, f"{rel_path} must build hands via load_hand(): {offenders}"


@pytest.mark.parametrize("rel_path", NO_HARDCODING_MODULES)
def test_no_hardcoded_device_or_model_path(rel_path):
    offenders = [
        value for value in _code_strings(_tree(rel_path))
        if DEVICE_PATH.match(value) or MODEL_PATH.search(value)
    ]
    assert not offenders, f"{rel_path} hardcodes {offenders}"


class TestCreateHandFromArgs:
    """orca_core.utils.cli: every advertised flag reaches load_hand()."""

    @staticmethod
    def _recording_load_hand(monkeypatch):
        import orca_core.utils.cli as cli

        calls = []

        def fake_load_hand(**kwargs):
            calls.append(kwargs)
            return SimpleNamespace(config=SimpleNamespace(config_path="fake.yaml"))

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        return cli, calls

    def _args(self, cli, argv):
        parser = argparse.ArgumentParser()
        cli.add_hand_arguments(parser)
        return parser.parse_args(argv)

    def test_flags_reach_load_hand(self, monkeypatch, capsys):
        cli, calls = self._recording_load_hand(monkeypatch)
        args = self._args(
            cli, ["cfg.yaml", "--mock", "--model-name", "orcahand-full-left",
                  "--no-engage-feedback"],
        )

        cli.create_hand_from_args(args)

        assert calls == [dict(
            config_path="cfg.yaml", mock=True,
            model_name="orcahand-full-left", engage_feedback=False,
        )]

    def test_front_end_override_wins_over_the_flag(self, monkeypatch, capsys):
        cli, calls = self._recording_load_hand(monkeypatch)
        args = self._args(cli, [])

        cli.create_hand_from_args(args, engage_feedback=False)

        assert calls[0]["engage_feedback"] is False

    def test_a_hidden_feedback_flag_defaults_to_engaged(self, monkeypatch, capsys):
        cli, calls = self._recording_load_hand(monkeypatch)
        parser = argparse.ArgumentParser()
        cli.add_hand_arguments(parser, feedback_flag=False)

        cli.create_hand_from_args(parser.parse_args([]))

        assert calls[0]["engage_feedback"] is True


class TestConfigureMotorChainFamily:
    """scripts/configure_motor_chain.py: the bus names the family, not the config.

    Every bundled config pins a family, so trusting it first would program a
    hand of the other family as this one on its very first setup step.
    """

    @staticmethod
    def _module(monkeypatch, detected):
        module = _load("scripts/configure_motor_chain.py")
        monkeypatch.setattr(module, "detect_motor_type", lambda *a, **k: detected)
        return module

    def test_a_detected_family_beats_the_configs_pin(self, monkeypatch, capsys):
        module = self._module(monkeypatch, "feetech")
        config = {"motor_type": "dynamixel", "baudrate": 1_000_000}

        assert module._resolve_motor_type(None, config, "/dev/fake") == "feetech"
        assert "dynamixel" in capsys.readouterr().out

    def test_the_config_names_the_family_of_an_already_configured_chain(
            self, monkeypatch, capsys):
        module = self._module(monkeypatch, None)

        assert module._resolve_motor_type(
            None, {"motor_type": "feetech"}, "/dev/fake") == "feetech"

    def test_a_silent_bus_and_an_unpinned_config_is_an_error(self, monkeypatch, capsys):
        module = self._module(monkeypatch, None)

        with pytest.raises(SystemExit):
            module._resolve_motor_type(None, {}, "/dev/fake")

    def test_explicit_flag_wins(self, monkeypatch):
        module = self._module(monkeypatch, "dynamixel")

        assert module._resolve_motor_type(
            "feetech", {"motor_type": "dynamixel"}, "/dev/fake") == "feetech"

    def test_the_other_familys_baudrate_is_dropped(self, monkeypatch):
        module = self._module(monkeypatch, "feetech")
        config = {"motor_type": "dynamixel", "baudrate": 3_000_000, "motor_ids": [1]}

        assert module._config_for_family(config, "feetech") == {
            "motor_type": "dynamixel", "motor_ids": [1]}
        assert module._config_for_family(config, "dynamixel") == config


class _FakeConfig:
    joint_ids = ["wrist", "index_mcp"]
    type = "right"


class _FakeHand:
    """Minimal stand-in for the recorder/replay scripts' hand."""

    def __init__(self, read_ok=None):
        self.config = _FakeConfig()
        self._read_ok = list(read_ok) if read_ok is not None else None
        self.last_read_ok = True
        self.commanded = []

    def init_joints(self, **kwargs):
        pass

    def disable_torque(self):
        pass

    def get_joint_position(self):
        if self._read_ok:
            self.last_read_ok = self._read_ok.pop(0)
        return _FakePose()

    def set_joint_positions(self, pose, **kwargs):
        self.commanded.append(pose)


class _FakePose:
    def as_list(self, joint_ids):
        return [0.0] * len(joint_ids)


def _patch_hand(monkeypatch, module, hand):
    monkeypatch.setattr(module, "create_hand_from_args", lambda *a, **k: hand)
    monkeypatch.setattr(module, "connect_hand", lambda *a, **k: None)
    monkeypatch.setattr(module, "shutdown_hand", lambda *a, **k: None)


class TestContinuousRecorder:
    """examples/record_continuous.py: honest rate, honest frames."""

    def _run(self, monkeypatch, tmp_path, hand, extra_argv=()):
        module = _load("examples/record_continuous.py")
        _patch_hand(monkeypatch, module, hand)
        monkeypatch.setattr("builtins.input", lambda *a: "")
        monkeypatch.setattr(
            sys, "argv",
            ["record_continuous.py", "--mock", "--duration", "0.2",
             "--frequency", "50", "--output-dir", str(tmp_path), *extra_argv],
        )
        assert module.main() == 0
        return sorted(tmp_path.glob("*.yaml"))

    def test_measured_rate_is_written_alongside_the_requested_one(
        self, monkeypatch, tmp_path, capsys,
    ):
        import yaml

        files = self._run(monkeypatch, tmp_path, _FakeHand())
        assert len(files) == 1
        metadata = yaml.safe_load(files[0].read_text())["metadata"]
        assert metadata["requested_frequency_hz"] == 50.0
        assert 0 < metadata["sampling_frequency_hz"] <= 50.0

    def test_stale_frames_are_dropped_and_a_bad_chain_is_not_saved(
        self, monkeypatch, tmp_path, capsys,
    ):
        hand = _FakeHand(read_ok=[True, False] * 500)
        files = self._run(monkeypatch, tmp_path, hand)
        out = capsys.readouterr().out
        assert files == []
        assert "dropped on failed motor reads" in out
        assert "Refusing to save" in out


class TestReplayAnglesSideGuard:
    """examples/replay_angles.py: a left-hand recording must not replay on a right hand."""

    def _write_sequence(self, tmp_path, hand_type):
        import yaml

        path = tmp_path / "seq.yaml"
        path.write_text(yaml.safe_dump({
            "metadata": {
                "type": "discrete_waypoints",
                "joint_ids": _FakeConfig.joint_ids,
                "hand_type": hand_type,
            },
            "waypoints": [[0.0, 0.0], [1.0, 1.0]],
        }))
        return path

    def _run(self, monkeypatch, path, extra_argv=()):
        module = _load("examples/replay_angles.py")
        hand = _FakeHand()
        _patch_hand(monkeypatch, module, hand)
        monkeypatch.setattr(
            sys, "argv",
            ["replay_angles.py", "--mock", "--replay-file", str(path), *extra_argv],
        )
        return module, hand

    def test_mirrored_recording_is_refused(self, monkeypatch, tmp_path):
        module, _ = self._run(monkeypatch, self._write_sequence(tmp_path, "left"))
        with pytest.raises(ValueError, match="hand_type=left"):
            module.main()

    def test_force_replays_the_mirrored_recording(self, monkeypatch, tmp_path):
        module, hand = self._run(
            monkeypatch, self._write_sequence(tmp_path, "left"), extra_argv=("--force",),
        )
        assert module.main() == 0
        assert hand.commanded

    def test_matching_side_replays(self, monkeypatch, tmp_path):
        module, hand = self._run(monkeypatch, self._write_sequence(tmp_path, "right"))
        assert module.main() == 0
        assert hand.commanded


class TestManualControlMotorSpace:
    """scripts/manual_control.py: sliders bounded by what the motor can reach."""

    @staticmethod
    def _module():
        pytest.importorskip("tkinter")
        return _load("scripts/manual_control.py")

    @staticmethod
    def _hand(module, limits, position_range):
        class _Client:
            position_range_rad = position_range

        class _Hand:
            motor_limits_dict = limits
            motor_client = _Client()

        return _Hand()

    def test_calibrated_limits_clamp_and_mark_the_slider(self):
        module = self._module()
        hand = self._hand(module, {1: [0.0, 0.5]}, None)
        low, high, clamped = module._motor_slider_range(hand, 1, 0.4)
        assert (low, high) == (0.0, 0.5)
        assert clamped

    def test_single_turn_range_bounds_an_uncalibrated_motor(self):
        module = self._module()
        hand = self._hand(module, {1: [None, None]}, (-6.28, 0.0))
        low, high, clamped = module._motor_slider_range(hand, 1, -0.2)
        assert (low, high) == (-1.2, 0.0)
        assert clamped

    def test_unbounded_motor_keeps_the_full_window(self):
        module = self._module()
        hand = self._hand(module, {}, None)
        low, high, clamped = module._motor_slider_range(hand, 1, 2.0)
        assert (low, high) == (1.0, 3.0)
        assert not clamped

    def test_feedback_only_flags_are_reported_when_set(self):
        module = self._module()
        parser = module.build_parser()
        assert module._feedback_only_overrides(parser, parser.parse_args([])) == []
        args = parser.parse_args(["--Kp", "7", "--correction-max-deg", "3"])
        assert module._feedback_only_overrides(parser, args) == [
            "--Kp", "--correction-max-deg",
        ]


class TestDetectScript:
    """scripts/detect.py: the 'what is plugged in?' command."""

    @staticmethod
    def _detection(**overrides):
        from orca_core import HandDetection

        fields = dict(
            model_name="orcahand-full-left",
            side="left",
            has_tactile=True,
            has_encoders=True,
            motor_port="/dev/cu.usbmodem-motor",
            sensing_port="/dev/cu.usbmodem-sense",
            tactile_port=None,
            identity=None,
            motor_type="feetech",
            motor_baudrate=1000000,
        )
        fields.update(overrides)
        return HandDetection(**fields)

    def test_prints_every_detected_field_and_the_hand_class(self, capsys, monkeypatch):
        module = _load("scripts/detect.py")
        monkeypatch.setattr(module, "detect_hand", lambda: self._detection())
        monkeypatch.setattr(sys, "argv", ["detect.py"])

        assert module.main() == 0

        out = capsys.readouterr().out
        for expected in (
            "orcahand-full-left", "OrcaHandFull", "left", "feetech", "1000000",
            "/dev/cu.usbmodem-motor", "/dev/cu.usbmodem-sense",
        ):
            assert expected in out

    def test_reports_a_silent_motor_bus(self, capsys, monkeypatch):
        module = _load("scripts/detect.py")
        monkeypatch.setattr(
            module, "detect_hand",
            lambda: self._detection(motor_type=None, motor_baudrate=None),
        )
        monkeypatch.setattr(sys, "argv", ["detect.py"])

        assert module.main() == 0
        assert "no motor answered" in capsys.readouterr().out

    def test_reports_a_missing_motor_bus(self, capsys, monkeypatch):
        module = _load("scripts/detect.py")
        monkeypatch.setattr(
            module, "detect_hand",
            lambda: self._detection(motor_port=None, motor_type=None, motor_baudrate=None),
        )
        monkeypatch.setattr(sys, "argv", ["detect.py"])

        assert module.main() == 0
        assert "No motor bus found" in capsys.readouterr().out
