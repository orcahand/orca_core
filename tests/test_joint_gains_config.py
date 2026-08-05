"""Per-joint outer-loop gain configuration: parsing, resolution, and the
runtime ``set_pid_gains`` / ``get_pid_gains`` facade."""

import dataclasses
import os
import shutil

import pytest
import yaml

from orca_core import JointGains
from orca_core.control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_JOINT_GAIN_OVERRIDES,
    DEFAULT_KI,
    DEFAULT_KP,
)
from orca_core.hand_config import (
    HandConfigValidationError,
    OrcaHandConfig,
    _parse_joint_control_gains,
)

from tests._hand_feedback_helpers import make_calibrated_joint_feedback_hand

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def joint_feedback_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    return str(config_path)


# ----- JointGains ---------------------------------------------------------


def test_defaults_match_package_constants():
    gains = JointGains()
    assert gains.kp == DEFAULT_KP
    assert gains.ki == DEFAULT_KI
    assert gains.correction_max_deg == DEFAULT_CORRECTION_MAX_DEG


@pytest.mark.parametrize("field", ["kp", "ki", "correction_max_deg"])
def test_negative_gains_are_rejected(field):
    with pytest.raises(HandConfigValidationError, match=field):
        JointGains(**{field: -1.0})


def test_from_dict_inherits_unspecified_fields_from_base():
    """A per-joint override names only the gain it changes."""
    base = JointGains(kp=0.5, ki=5.0, correction_max_deg=15.0)
    tuned = JointGains.from_dict({"kp": 0.25}, base=base)
    assert tuned.kp == 0.25
    assert tuned.ki == 5.0
    assert tuned.correction_max_deg == 15.0


def test_from_dict_rejects_unknown_keys():
    with pytest.raises(HandConfigValidationError, match="i_clamp_deg"):
        JointGains.from_dict({"i_clamp_deg": 15.0})


@pytest.mark.parametrize(
    "bad", [float("nan"), float("inf"), True, None, "0.5"],
    ids=["nan", "inf", "bool", "none", "string"],
)
def test_non_numeric_gain_values_are_rejected(bad):
    """YAML can deliver .nan, booleans, nulls, and strings; none of them may
    reach the controller as a gain."""
    with pytest.raises(HandConfigValidationError):
        JointGains.from_dict({"kp": bad})


# ----- config.yaml block --------------------------------------------------


def test_parse_block_keeps_overrides_partial():
    """Layers stay partial so each one overrides only what it names."""
    gains_all, overrides = _parse_joint_control_gains({
        "all": {"kp": 0.4, "ki": 4.0},
        "joints": {"index_mcp": {"kp": 0.2}},
    })
    assert gains_all == {"kp": 0.4, "ki": 4.0}
    assert overrides == {"index_mcp": {"kp": 0.2}}


def test_parse_block_rejects_bad_values_at_load_time():
    with pytest.raises(HandConfigValidationError, match="non-negative"):
        _parse_joint_control_gains({"all": {"kp": -1.0}})
    with pytest.raises(HandConfigValidationError, match="unknown joint gain"):
        _parse_joint_control_gains({"joints": {"index_mcp": {"kd": 1.0}}})


def test_parse_block_tolerates_empty_and_absent_sections():
    assert _parse_joint_control_gains(None) == ({}, {})
    assert _parse_joint_control_gains({}) == ({}, {})
    assert _parse_joint_control_gains({"joints": None}) == ({}, {})
    assert _parse_joint_control_gains({"all": None}) == ({}, {})


def test_parse_block_rejects_unknown_section():
    with pytest.raises(HandConfigValidationError, match="default"):
        _parse_joint_control_gains({"default": {}})


def _write_config(tmp_path, gains_block):
    """A minimal valid config.yaml carrying a joint_control_gains block."""
    src = OrcaHandConfig.from_config_path(model_version="v2",
                                          model_name="orcahand-right")
    doc = yaml.safe_load(open(src.config_path))
    doc["joint_control_gains"] = gains_block
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    (model_dir / "config.yaml").write_text(yaml.safe_dump(doc))
    return str(model_dir / "config.yaml")


def test_config_resolves_per_joint_gains(tmp_path):
    path = _write_config(tmp_path, {
        "all": {"kp": 0.4},
        "joints": {"index_mcp": {"kp": 0.25}},
    })
    config = OrcaHandConfig.from_config_path(config_path=path)

    assert config.joint_gains_for("index_mcp").kp == 0.25
    assert config.joint_gains_for("index_mcp").ki == DEFAULT_KI
    assert config.joint_gains_for("middle_mcp").kp == 0.4
    assert config.joint_gains_dict["index_mcp"].kp == 0.25


def test_config_without_the_block_uses_package_defaults():
    config = OrcaHandConfig.from_config_path(model_version="v2",
                                             model_name="orcahand-right")
    assert config.joint_gains_for("index_mcp") == JointGains()
    assert config.joint_gains_baseline == JointGains()


def test_override_for_unknown_joint_is_rejected(tmp_path):
    """A typo'd joint name would otherwise silently never apply."""
    path = _write_config(tmp_path, {"joints": {"index_mcpp": {"kp": 0.2}}})
    with pytest.raises(HandConfigValidationError, match="index_mcpp"):
        OrcaHandConfig.from_config_path(config_path=path)


def test_packaged_models_pin_no_gains_so_constants_stay_authoritative():
    """No packaged config.yaml may hard-code gains: doing so would shadow
    control/constants.py and make editing the defaults a silent no-op."""
    import glob

    paths = glob.glob(
        os.path.join(REPO_ROOT, "orca_core", "models", "*", "*", "config.yaml")
    )
    assert paths, "packaged model configs must be found for this check to bite"
    for path in paths:
        doc = yaml.safe_load(open(path)) or {}
        assert "joint_control_gains" not in doc, path

    for model in ("orcahand-joint-right", "orcahand-full-right"):
        config = OrcaHandConfig.from_config_path(model_version="v2",
                                                 model_name=model)
        assert config.joint_gains_all == {}
        assert config.joint_gains_overrides == {}


def test_shipped_per_joint_defaults_apply_without_any_config(tmp_path):
    """thumb_cmc and thumb_abd ship with a lower kp: they reach their travel in
    less motor rotation, so they respond faster and have less margin."""
    config = OrcaHandConfig.from_config_path(model_version="v2",
                                             model_name="orcahand-joint-right")
    for joint in ("thumb_cmc", "thumb_abd"):
        gains = config.joint_gains_for(joint)
        assert gains.kp == 0.35
        # Only kp is overridden; the rest fall through to the scalars.
        assert gains.ki == DEFAULT_KI
        assert gains.correction_max_deg == DEFAULT_CORRECTION_MAX_DEG

    assert config.joint_gains_for("thumb_mcp").kp == DEFAULT_KP
    assert config.joint_gains_for("index_mcp").kp == DEFAULT_KP


def test_shipped_per_joint_defaults_name_real_joints():
    """A typo here would silently never apply to anything."""
    config = OrcaHandConfig.from_config_path(model_version="v2",
                                             model_name="orcahand-joint-right")
    assert set(DEFAULT_JOINT_GAIN_OVERRIDES) <= set(config.joint_ids)


def test_config_all_overrides_the_shipped_per_joint_defaults(tmp_path):
    """'all' means all — it wins over the shipped per-joint table."""
    path = _write_config(tmp_path, {"all": {"kp": 0.8}})
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.joint_gains_for("thumb_cmc").kp == 0.8


def test_config_joints_wins_over_everything(tmp_path):
    path = _write_config(tmp_path, {
        "all": {"kp": 0.8},
        "joints": {"thumb_cmc": {"kp": 0.15}},
    })
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.joint_gains_for("thumb_cmc").kp == 0.15
    assert config.joint_gains_for("thumb_abd").kp == 0.8


# ----- runtime facade -----------------------------------------------------


def test_connect_installs_per_joint_gains_from_config(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.config = dataclasses.replace(
        hand.config,
        joint_gains_all={"kp": 0.5, "ki": 5.0},
        joint_gains_overrides={"index_mcp": {"kp": 0.1}},
    )
    hand.connect()
    try:
        gains = hand.get_pid_gains()
        assert gains["index_mcp"].kp == 0.1
        assert all(g.kp == 0.5 for j, g in gains.items() if j != "index_mcp")
    finally:
        hand.disconnect()


def test_set_pid_gains_dict_updates_only_named_joints(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        before = hand.get_pid_gains()
        hand.set_pid_gains(Kp={"index_mcp": 0.9})
        after = hand.get_pid_gains()

        assert after["index_mcp"].kp == 0.9
        assert after["index_mcp"].ki == before["index_mcp"].ki
        for joint in after:
            if joint != "index_mcp":
                assert after[joint] == before[joint]
    finally:
        hand.disconnect()


def test_set_pid_gains_scalar_applies_to_every_joint(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        hand.set_pid_gains(Kp=0.3, Ki=2.0, correction_max_deg=9.0)
        assert all(
            (g.kp, g.ki, g.correction_max_deg) == (0.3, 2.0, 9.0)
            for g in hand.get_pid_gains().values()
        )
    finally:
        hand.disconnect()


def test_set_pid_gains_leaves_omitted_arguments_untouched(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        hand.set_pid_gains(Kp=0.3, Ki=2.0, correction_max_deg=9.0)
        hand.set_pid_gains(Ki=7.0)
        gains = hand.get_pid_gains()
        assert all(g.kp == 0.3 for g in gains.values())
        assert all(g.ki == 7.0 for g in gains.values())
        assert all(g.correction_max_deg == 9.0 for g in gains.values())
    finally:
        hand.disconnect()


def test_set_pid_gains_accepts_array_in_loop_order(joint_feedback_config):
    """Downstream tuners pass full per-joint vectors; shape is validated."""
    import numpy as np

    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        n = len(hand.loop_joint_names)
        hand.set_pid_gains(Kp=np.full(n, 0.4))
        assert all(g.kp == 0.4 for g in hand.get_pid_gains().values())
        with pytest.raises(ValueError, match="shape"):
            hand.set_pid_gains(Kp=np.full(n + 1, 0.4))
    finally:
        hand.disconnect()


def test_set_pid_gains_rejects_uncontrolled_joint(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    hand.connect()
    try:
        with pytest.raises(ValueError, match="not_a_joint"):
            hand.set_pid_gains(Kp={"not_a_joint": 0.5})
    finally:
        hand.disconnect()


def test_set_and_get_pid_gains_require_a_running_loop(joint_feedback_config):
    hand = make_calibrated_joint_feedback_hand(joint_feedback_config)
    with pytest.raises(RuntimeError, match="not running"):
        hand.set_pid_gains(Kp=0.5)
    with pytest.raises(RuntimeError, match="not running"):
        hand.get_pid_gains()
