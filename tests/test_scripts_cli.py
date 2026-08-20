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
    # Shipped front-ends, held to the same rule as the scripts.
    "orca_core/api/api.py",
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
FEEDBACK_PINNED = re.compile(
    r"create_(hand|fleet)_from_args\([^)]*engage_feedback\s*="
)

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
    assert "--hand" in help_text, "no way to name a hand on a two-hand bench"
    assert "--list-hands" in help_text
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
    assert "create_hand_from_args(" in source or "create_fleet_from_args(" in source, (
        f"{rel_path} must build its hand from args"
    )
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
            engage_sensors=True, select=None,
        )]

    def test_front_end_override_wins_over_the_flag(self, monkeypatch, capsys):
        cli, calls = self._recording_load_hand(monkeypatch)
        args = self._args(cli, [])

        cli.create_hand_from_args(args, engage_feedback=False, engage_sensors=False)

        assert calls[0]["engage_feedback"] is False
        assert calls[0]["engage_sensors"] is False

    def test_nothing_selecting_a_model_leaves_load_hand_to_autodetect(
        self, monkeypatch, capsys,
    ):
        """The bare invocation must reach load_hand with no model pinned —
        anything else silently loads the packaged default against real hardware."""
        cli, calls = self._recording_load_hand(monkeypatch)

        cli.create_hand_from_args(self._args(cli, []))

        assert calls[0]["config_path"] is None
        assert calls[0]["model_name"] is None
        assert calls[0]["mock"] is False

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


class TestEngageFlagsSelectTheClass:
    """The two engage_* flags are what a front-end uses to withhold a link it
    would otherwise open; they must actually change the class that is built."""

    MODEL = "orcahand-full-right"

    def _hand(self, **kwargs):
        from orca_core.utils.cli import create_hand

        return create_hand(None, use_mock=True, model_name=self.MODEL, **kwargs)

    def test_everything_engaged_is_the_full_hand(self):
        from orca_core import MockOrcaHandFull

        assert isinstance(self._hand(), MockOrcaHandFull)

    def test_disengaging_feedback_keeps_tactile(self):
        from orca_core import MockOrcaHandTouch

        assert isinstance(self._hand(engage_feedback=False), MockOrcaHandTouch)

    def test_disengaging_both_is_motor_only(self):
        from orca_core import MockOrcaHand

        hand = self._hand(engage_feedback=False, engage_sensors=False)
        assert type(hand) is MockOrcaHand

    def test_the_config_keeps_its_sensor_declaration_either_way(self):
        """Withholding the class must not strip the config: calibration's
        encoder pass still reads the declaration off it."""
        hand = self._hand(engage_feedback=False, engage_sensors=False)
        assert hand.config.joint_feedback_enabled
        assert hand.config.sensor_port


HAND_SELECTION_MODULES = HAND_CLI_MODULES + [
    "scripts/check_sensors.py",
    "scripts/check_encoder_signs.py",
]


@pytest.mark.parametrize("rel_path", HAND_SELECTION_MODULES)
def test_every_front_end_can_name_a_hand(rel_path, capsys, monkeypatch):
    """With two hands attached, a front-end that cannot be told which one
    either tracebacks or silently picks an arm."""
    monkeypatch.setattr(sys, "argv", [rel_path, "--help"])
    module = _load(rel_path)
    help_text = _help_text(module, rel_path, capsys)

    assert "--hand" in help_text
    assert "--list-hands" in help_text


def _detection(hand_id, board_id=None, side="right", model="orcahand-right"):
    from orca_core.hand_factory import HandDetection

    return HandDetection(
        model_name=model, side=side, has_tactile=False, has_encoders=False,
        hand_id=hand_id, board_id=board_id or f"BID-{hand_id}",
    )


class TestQuietConstruction:
    def test_quiet_drops_the_uncalibrated_banner_but_keeps_other_output(
        self, monkeypatch, capsys
    ):
        import orca_core.utils.cli as cli

        def fake_load_hand(**kwargs):
            print("Using model path: /fake/model")
            print("\x1b[93mWarning: Motor ID 1 (Joint: wrist) has not been "
                  "fully calibrated (missing motor limits).\x1b[0m")
            print("\x1b[93mWarning: Joint wrist is missing a joint-encoder "
                  "calibration entry.\x1b[0m")
            return SimpleNamespace(config=SimpleNamespace(config_path="fake.yaml"))

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: [_detection("ser-0001")])

        cli.create_hand(None, use_mock=False, quiet=True)

        out = capsys.readouterr().out
        assert "Using model path" in out
        assert "has not been fully calibrated" not in out
        assert "missing a joint-encoder calibration entry" not in out

    def test_without_quiet_the_banner_still_shows(self, monkeypatch, capsys):
        import orca_core.utils.cli as cli

        def fake_load_hand(**kwargs):
            print("Warning: Motor ID 1 (Joint: wrist) has not been fully "
                  "calibrated (missing motor limits).")
            return SimpleNamespace(config=SimpleNamespace(config_path="fake.yaml"))

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: [_detection("ser-0001")])

        cli.create_hand(None, use_mock=False)

        assert "has not been fully calibrated" in capsys.readouterr().out

    def test_quiet_does_not_wrap_the_interactive_picker(self, monkeypatch, capsys):
        """The bug this guards against: buffering the whole selection call
        (rather than just load_hand()) would swallow the picker's prompt,
        since input() writes its prompt through sys.stdout too."""
        from orca_core.hand_factory import select_hand

        detections = [_detection("ser-0001"), _detection("ser-0002")]
        cli, calls = TestHandSelectionFromTheCommandLine._cli_with_real_ambiguity(
            monkeypatch, detections
        )
        prompted = []

        def fake_picker(dets, **kw):
            prompted.append(dets)
            return cli.HandSelector(hand_id="ser-0002")

        monkeypatch.setattr(cli, "_choose_hand_interactively", fake_picker)

        hand = cli.create_hand(None, use_mock=False, quiet=True)

        assert len(prompted) == 1  # the picker really ran, not skipped
        assert hand.config.config_path == "ser-0002.yaml"

    def test_sensors_mode_drops_motor_limits_but_keeps_encoder_anchors_when_feedback_enabled(
        self, monkeypatch, capsys
    ):
        import orca_core.utils.cli as cli

        def fake_load_hand(**kwargs):
            print("Warning: Motor ID 1 (Joint: wrist) has not been fully "
                  "calibrated (missing motor limits).")
            print("Warning: Joint wrist is missing a joint-encoder "
                  "calibration entry.")
            return SimpleNamespace(
                config=SimpleNamespace(
                    config_path="fake.yaml", joint_feedback_enabled=True
                )
            )

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: [_detection("ser-0001")])

        cli.create_hand(None, use_mock=False, quiet="sensors")

        out = capsys.readouterr().out
        assert "has not been fully calibrated" not in out
        assert "missing a joint-encoder calibration entry" in out

    def test_sensors_mode_drops_encoder_anchors_too_without_feedback(
        self, monkeypatch, capsys
    ):
        import orca_core.utils.cli as cli

        def fake_load_hand(**kwargs):
            print("Warning: Motor ID 1 (Joint: wrist) has not been fully "
                  "calibrated (missing motor limits).")
            return SimpleNamespace(
                config=SimpleNamespace(
                    config_path="fake.yaml", joint_feedback_enabled=False
                )
            )

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: [_detection("ser-0001")])

        cli.create_hand(None, use_mock=False, quiet="sensors")

        assert "has not been fully calibrated" not in capsys.readouterr().out


class TestQuietUncalibratedWarnings:
    def test_suppresses_the_hardware_hand_logger_and_restores_it(self):
        import logging

        import orca_core.utils.cli as cli

        target = logging.getLogger("orca_core.hardware_hand")
        target.setLevel(logging.WARNING)

        with cli.quiet_uncalibrated_warnings():
            assert target.level == logging.ERROR

        assert target.level == logging.WARNING

    def test_restores_the_level_even_on_exception(self):
        import logging

        import orca_core.utils.cli as cli

        target = logging.getLogger("orca_core.hardware_hand")
        target.setLevel(logging.WARNING)

        with pytest.raises(ValueError):
            with cli.quiet_uncalibrated_warnings():
                raise ValueError("boom")

        assert target.level == logging.WARNING


class TestChooseHandInteractively:
    @staticmethod
    def _interactive(monkeypatch, answer):
        import orca_core.utils.cli as cli

        monkeypatch.setattr(
            cli, "sys", SimpleNamespace(stdin=SimpleNamespace(isatty=lambda: True))
        )
        monkeypatch.setattr("builtins.input", lambda prompt="": answer)
        return cli

    def test_not_a_tty_returns_none_without_prompting(self, monkeypatch):
        import orca_core.utils.cli as cli

        monkeypatch.setattr(
            cli, "sys", SimpleNamespace(stdin=SimpleNamespace(isatty=lambda: False))
        )
        monkeypatch.setattr(
            "builtins.input", lambda prompt="": (_ for _ in ()).throw(AssertionError("must not prompt"))
        )
        assert cli._choose_hand_interactively([_detection("ser-0001")]) is None

    def test_picks_a_hand_by_number(self, monkeypatch, capsys):
        from orca_core import HandSelector

        cli = self._interactive(monkeypatch, "2")
        detections = [_detection("ser-0001"), _detection("ser-0002")]

        result = cli._choose_hand_interactively(detections)

        assert result == HandSelector(hand_id="ser-0002")

    def test_all_is_offered_only_when_allowed(self, monkeypatch, capsys):
        cli = self._interactive(monkeypatch, "a")
        detections = [_detection("ser-0001"), _detection("ser-0002")]

        assert cli._choose_hand_interactively(detections, allow_all=True) == "all"
        assert "All 2 hands" in capsys.readouterr().out

    def test_quitting_returns_none(self, monkeypatch):
        cli = self._interactive(monkeypatch, "q")
        assert cli._choose_hand_interactively([_detection("ser-0001")]) is None


class TestHandSelectionFromTheCommandLine:
    @staticmethod
    def _cli(monkeypatch, detections):
        import orca_core.utils.cli as cli

        calls = []

        def fake_load_hand(**kwargs):
            calls.append(kwargs)
            return SimpleNamespace(config=SimpleNamespace(config_path="fake.yaml"))

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: list(detections))
        return cli, calls

    def _args(self, cli, argv):
        parser = argparse.ArgumentParser()
        cli.add_hand_arguments(parser)
        return parser.parse_args(argv)

    def test_hand_flag_reaches_load_hand_as_a_selector(self, monkeypatch, capsys):
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001"), _detection("ser-0002")])

        cli.create_hand_from_args(self._args(cli, ["--hand", "ser-0002"]))

        assert calls[0]["select"].hand_id == "ser-0002"

    def test_a_board_id_is_accepted_where_a_hand_id_does_not_match(
            self, monkeypatch, capsys):
        """An unprovisioned board answers with its board id; the operator
        should not have to know which one their hand reports."""
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001", board_id="BID-X")])

        cli.create_hand_from_args(self._args(cli, ["--hand", "BID-X"]))

        assert calls[0]["select"].board_id == "BID-X"
        assert calls[0]["select"].hand_id is None

    def test_two_hands_exit_non_zero_naming_both_and_the_flag(
            self, monkeypatch, capsys):
        import orca_core.utils.cli as cli
        from orca_core import AmbiguousHandError

        detections = [_detection("ser-0001"), _detection("ser-0002")]
        monkeypatch.setattr(cli, "detect_hands", lambda: list(detections))

        def raising(**kwargs):
            # Raised the way the library really raises it, advice and all —
            # a short stand-in message would hide the leak this pins.
            from orca_core.hand_factory import select_hand

            select_hand(detections, None)

        monkeypatch.setattr(cli, "load_hand", raising)

        with pytest.raises(SystemExit) as excinfo:
            cli.create_hand_from_args(self._args(cli, []))

        message = str(excinfo.value)
        assert "ser-0001" in message and "ser-0002" in message
        assert "--hand" in message
        assert "load_hand(" not in message, "advice the operator cannot act on"

    def test_list_hands_prints_the_attached_hands_and_stops(
            self, monkeypatch, capsys):
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001"), _detection("ser-0002")])

        with pytest.raises(SystemExit) as excinfo:
            cli.create_hand_from_args(self._args(cli, ["--list-hands"]))

        assert excinfo.value.code == 0
        assert calls == [], "listing hands must not build one"
        out = capsys.readouterr().out
        assert "ser-0001" in out and "ser-0002" in out

    def test_hand_is_refused_where_it_cannot_be_honoured(self, monkeypatch, capsys):
        """Recording rather than raising: a load_hand patched to raise routes
        into the ambiguity handler, whose message also mentions --hand."""
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001")])

        with pytest.raises(SystemExit) as excinfo:
            cli.create_hand_from_args(self._args(cli, ["--mock", "--hand", "ser-0001"]))

        assert "--mock" in str(excinfo.value)
        assert calls == []

    def test_a_board_reporting_no_id_is_still_listed(self, monkeypatch, capsys):
        cli, _ = self._cli(monkeypatch, [_detection(None, board_id=None)])

        with pytest.raises(SystemExit):
            cli.create_hand_from_args(self._args(cli, ["--list-hands"]))

        assert "unidentified" in capsys.readouterr().out

    @staticmethod
    def _cli_with_real_ambiguity(monkeypatch, detections):
        """Like _cli, but fake_load_hand actually resolves select_hand()
        against the fake detections, so an unselected two-hand call raises
        AmbiguousHandError the way the real load_hand() does."""
        import orca_core.utils.cli as cli
        from orca_core.hand_factory import select_hand

        calls = []

        def fake_load_hand(*, select=None, **kwargs):
            calls.append({"select": select, **kwargs})
            picked = select_hand(list(detections), select)
            return SimpleNamespace(config=SimpleNamespace(config_path=f"{picked.hand_id}.yaml"))

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: list(detections))
        return cli, calls

    def test_two_hands_with_no_flag_offers_the_interactive_picker(self, monkeypatch, capsys):
        """The bug this pins: --hand-less two-hand runs used to exit non-zero
        unconditionally, with no chance to choose — like the port picker
        offers when connect() can't resolve a port on its own."""
        detections = [_detection("ser-0001"), _detection("ser-0002")]
        cli, calls = self._cli_with_real_ambiguity(monkeypatch, detections)
        monkeypatch.setattr(
            cli, "_choose_hand_interactively",
            lambda dets, **kw: cli.HandSelector(hand_id="ser-0002"),
        )

        hand = cli.create_hand_from_args(self._args(cli, []))

        assert hand.config.config_path == "ser-0002.yaml"
        assert calls[-1]["select"].hand_id == "ser-0002"

    def test_quitting_the_picker_still_exits_non_zero(self, monkeypatch, capsys):
        detections = [_detection("ser-0001"), _detection("ser-0002")]
        cli, calls = self._cli_with_real_ambiguity(monkeypatch, detections)
        monkeypatch.setattr(cli, "_choose_hand_interactively", lambda dets, **kw: None)

        with pytest.raises(SystemExit) as excinfo:
            cli.create_hand_from_args(self._args(cli, []))

        assert "ser-0001" in str(excinfo.value) and "ser-0002" in str(excinfo.value)


class TestFleetSelectionFromTheCommandLine:
    @staticmethod
    def _cli(monkeypatch, detections, hands=None):
        import orca_core.utils.cli as cli
        from orca_core import HandFleet

        calls = []

        def fake_load_hands(**kwargs):
            calls.append(kwargs)
            built = hands if hands is not None else [
                SimpleNamespace(config=SimpleNamespace(config_path=f"{d.hand_id}.yaml"))
                for d in detections
            ]
            return HandFleet(tuple(built), tuple(d.hand_id for d in detections))

        monkeypatch.setattr(cli, "load_hands", fake_load_hands)
        monkeypatch.setattr(cli, "detect_hands", lambda: list(detections))
        return cli, calls

    def _args(self, cli, argv):
        parser = argparse.ArgumentParser()
        cli.add_hand_arguments(parser, all_flag=True)
        return parser.parse_args(argv)

    def test_no_all_flag_wraps_a_single_hand_in_a_one_hand_fleet(self, monkeypatch, capsys):
        import orca_core.utils.cli as cli

        hand = SimpleNamespace(config=SimpleNamespace(config_path="fake.yaml"))
        monkeypatch.setattr(cli, "load_hand", lambda **kwargs: hand)
        monkeypatch.setattr(cli, "detect_hands", lambda: [_detection("ser-0001")])

        fleet = cli.create_fleet_from_args(self._args(cli, []))

        assert len(fleet) == 1
        assert fleet.only() is hand

    def test_all_flag_builds_every_attached_hand(self, monkeypatch, capsys):
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001"), _detection("ser-0002")])

        fleet = cli.create_fleet_from_args(self._args(cli, ["--all"]))

        assert len(fleet) == 2
        assert calls[0].get("select") is None

    def test_no_flags_offers_all_in_the_interactive_picker(self, monkeypatch, capsys):
        """The other half of the bug report: a fleet-aware script's picker
        must offer "all", not just one hand at a time."""
        from orca_core.hand_factory import select_hand

        detections = [_detection("ser-0001"), _detection("ser-0002")]
        cli, calls = self._cli(monkeypatch, detections)

        def fake_load_hand(*, select=None, **kwargs):
            select_hand(list(detections), select)  # raises when select is None

        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        monkeypatch.setattr(
            cli, "_choose_hand_interactively", lambda dets, **kw: "all"
        )

        fleet = cli.create_fleet_from_args(self._args(cli, []))

        assert len(fleet) == 2
        assert calls[0].get("select") is None

    def test_picker_offers_all_flag_to_choose_hand_interactively(self, monkeypatch, capsys):
        """_choose_hand_interactively is called with allow_all=True when the
        parser supports --all and nothing else narrowed the choice."""
        import orca_core.utils.cli as cli
        from orca_core.hand_factory import select_hand

        detections = [_detection("ser-0001"), _detection("ser-0002")]

        def fake_load_hand(*, select=None, **kwargs):
            select_hand(list(detections), select)

        cli, calls = self._cli(monkeypatch, detections)
        monkeypatch.setattr(cli, "load_hand", fake_load_hand)
        seen_kwargs = {}

        def spy(dets, **kw):
            seen_kwargs.update(kw)
            return None

        monkeypatch.setattr(cli, "_choose_hand_interactively", spy)

        with pytest.raises(SystemExit):
            cli.create_fleet_from_args(self._args(cli, []))

        assert seen_kwargs.get("allow_all") is True

    def test_all_is_refused_with_a_config_path(self, monkeypatch, capsys):
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001"), _detection("ser-0002")])

        with pytest.raises(SystemExit):
            cli.create_fleet_from_args(self._args(cli, ["fake.yaml", "--all"]))

        assert calls == []

    def test_all_is_refused_with_hand(self, monkeypatch, capsys):
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001"), _detection("ser-0002")])

        with pytest.raises(SystemExit):
            cli.create_fleet_from_args(self._args(cli, ["--all", "--hand", "ser-0001"]))

        assert calls == []

    def test_all_with_no_hand_attached_exits_non_zero(self, monkeypatch, capsys):
        cli, _ = self._cli(monkeypatch, [], hands=[])

        with pytest.raises(SystemExit):
            cli.create_fleet_from_args(self._args(cli, ["--all"]))

    def test_overrides_reach_load_hands(self, monkeypatch, capsys):
        cli, calls = self._cli(monkeypatch, [_detection("ser-0001")])

        cli.create_fleet_from_args(
            self._args(cli, ["--all"]), engage_feedback=False, engage_sensors=False
        )

        assert calls[0]["engage_feedback"] is False
        assert calls[0]["engage_sensors"] is False


class TestDetectReportsCalibrationForTheHandItDescribes:
    """load_hand() now probes the bus regardless of model_name, so an
    unselected call is ambiguous between several attached hands — reproduced
    on real hardware 2026-08-14: with two hands attached, detect.py's
    per-hand calibration report failed with AmbiguousHandError for both."""

    def test_print_calibration_selects_the_hand_it_is_reporting_on(
            self, monkeypatch, capsys):
        from orca_core import AmbiguousHandError

        detect = _load("scripts/detect.py")
        detections = [_detection("ser-0001"), _detection("ser-0002")]
        calls = []

        def fake_load_hand(*, model_name, engage_feedback, engage_sensors, select=None):
            calls.append(select)
            if select is None:
                raise AmbiguousHandError("ambiguous", tuple(detections), "ambiguous")
            matches = [d for d in detections if d.hand_id == select.hand_id]
            assert len(matches) == 1, "selector did not resolve to exactly one hand"
            return SimpleNamespace(
                config=SimpleNamespace(model_path="fake", joint_feedback_enabled=False),
                calibrated=False,
                wrist_calibrated=False,
            )

        monkeypatch.setattr(detect, "load_hand", fake_load_hand)

        detect._print_calibration(detections[1])

        assert calls[-1] is not None and calls[-1].hand_id == "ser-0002"
        assert "NOT calibrated" in capsys.readouterr().out


class TestManualControlHandCap:
    """manual_control.py's non-GUI logic: the two-hand cap and column
    labels. The widget classes themselves need a real Tk display (none of
    the existing scripts/examples tests instantiate one either) so those
    stay verified by hand, in a real terminal."""

    def test_refuses_more_than_two_hands(self):
        from orca_core import HandFleet

        mc = _load("scripts/manual_control.py")
        fleet = HandFleet((object(), object(), object()), ("a", "b", "c"))

        with pytest.raises(SystemExit) as excinfo:
            mc._refuse_if_too_many(fleet)

        assert "a" in str(excinfo.value) and "c" in str(excinfo.value)

    def test_allows_up_to_two_hands(self):
        from orca_core import HandFleet

        mc = _load("scripts/manual_control.py")
        mc._refuse_if_too_many(HandFleet((object(),), ("a",)))
        mc._refuse_if_too_many(HandFleet((object(), object()), ("a", "b")))

    def test_hand_label_names_the_hand_and_its_side(self):
        mc = _load("scripts/manual_control.py")
        hand = SimpleNamespace(config=SimpleNamespace(type="right"))

        assert mc._hand_label("ser-8316", hand) == "ser-8316  (right)"


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

    def _run(self, monkeypatch, capsys, **overrides):
        module = _load("scripts/detect.py")
        monkeypatch.setattr(module, "detect_hand", lambda: self._detection(**overrides))
        # Reading calibration would build a real hand off the bundled model;
        # this script's port and family reporting is what is under test.
        monkeypatch.setattr(
            module, "_print_calibration", lambda detection: None
        )
        monkeypatch.setattr(sys, "argv", ["detect.py"])
        module.main()
        return capsys.readouterr().out

    def test_prints_the_model_ports_and_motor_family(self, capsys, monkeypatch):
        out = self._run(monkeypatch, capsys)
        for expected in (
            "orcahand-full-left", "left", "tactile", "encoders",
            "feetech", "1000000",
            "/dev/cu.usbmodem-motor", "/dev/cu.usbmodem-sense",
        ):
            assert expected in out

    def test_reports_a_bus_no_motor_answered_on(self, capsys, monkeypatch):
        out = self._run(monkeypatch, capsys, motor_type=None, motor_baudrate=None)
        assert "no motor answered" in out

    def test_reports_a_missing_motor_bus(self, capsys, monkeypatch):
        out = self._run(
            monkeypatch, capsys,
            motor_port=None, motor_type=None, motor_baudrate=None,
        )
        assert "Motor bus:   not detected" in out
        assert "no motor answered" not in out
