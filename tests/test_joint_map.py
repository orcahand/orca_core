"""``joint_map.yaml``: a hand's own stops and nonlinear joint↔motor curves.

Without the file (or for a joint it gives no curve) the map must stay the
linear one bit for bit; with it, forward and inverse must round-trip, the
joint loop must agree with the open-loop map, and anything malformed must
fall back to linear with a warning.
"""

from __future__ import annotations

import json
import logging
import math
import os

import numpy as np
import pytest
import yaml

from orca_core.control import JointController, JointLoopThread
from orca_core.hand_config import OrcaHandConfig
from orca_core.joint_map import (
    JOINT_MAP_FILE,
    MAX_KNOTS,
    CurveBank,
    JointCurve,
    from_visual_calibration,
    load_joint_map,
)
from orca_core.hand_config import write_standalone_config
from orca_core.tools import joint_map_from_visual
from tests._loop_helpers import (
    StaticEncoderSource,
    encoder_reading_from_joint_angles,
    make_calibrated_hand,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(REPO_ROOT, "orca_core", "models", "v2")
JOINT_RIGHT = os.path.join(MODELS, "orcahand-joint-right", "config.yaml")
TOUCH_RIGHT = os.path.join(MODELS, "orcahand-touch-right", "config.yaml")
VISUAL = os.path.join(REPO_ROOT, "tests", "fixtures", "visual_calibration_ser0000.json")
LOGGER = "orca_core.joint_map"


def _bent(rom, n=7, amp=0.05):
    """A strictly increasing, visibly nonlinear curve over ``rom``."""
    lo, hi = rom
    s = np.linspace(0.0, 1.0, n)
    return {"s": s.tolist(), "deg": (lo + (hi - lo) * (s + amp * np.sin(np.pi * s))).tolist()}


def _write_map(directory, curves=None, joint_roms=None, **extra):
    doc = {"format_version": 1, "hand_serial": "ser-test", "model": None,
           "joint_roms": joint_roms or {}, "curves": curves or {}, **extra}
    (directory / JOINT_MAP_FILE).write_text(yaml.safe_dump(doc), encoding="utf-8")


def _copy_config(tmp_path, source=JOINT_RIGHT):
    return write_standalone_config(str(tmp_path / "config.yaml"), config_path=source)


def _hand(tmp_path, curves=None, joint_roms=None, source=JOINT_RIGHT):
    config_path = _copy_config(tmp_path, source)
    if curves is not None or joint_roms is not None:
        _write_map(tmp_path, curves, joint_roms)
    return make_calibrated_hand(config_path)


def _all_curves(config_path):
    roms = OrcaHandConfig.from_config_path(config_path=config_path).joint_roms_dict
    return {j: _bent(rom) for j, rom in roms.items()}


# --- the pre-joint_map linear map, verbatim, as the parity reference -------

def _old_motor_to_joint(hand, motor_pos):
    joint_roms = hand.effective_joint_roms_dict
    out = {}
    for idx, pos in enumerate(motor_pos):
        motor_id = hand.config.motor_ids[idx]
        joint_name = hand.config.motor_to_joint_dict.get(motor_id)
        wrapped_pos = pos - hand._wrap_offsets_dict.get(motor_id, 0.0)
        d = (wrapped_pos - hand.motor_limits_dict[motor_id][0]) / hand.calibration.joint_to_motor_ratios_dict[motor_id]
        lo, hi = joint_roms[joint_name]
        out[joint_name] = hi - d if hand.config.joint_inversion_dict.get(joint_name, False) else lo + d
    return out


def _old_joint_to_motor(hand, joint_pos):
    joint_roms = hand.effective_joint_roms_dict
    motor_pos = [None] * len(hand.config.motor_ids)
    for joint_name, pos in joint_pos.items():
        motor_id = hand.config.joint_to_motor_map.get(joint_name)
        if motor_id is None:
            continue
        idx = hand.config.motor_id_to_idx_dict[motor_id]
        lo, hi = joint_roms[joint_name]
        d = ((hi - pos) if hand.config.joint_inversion_dict.get(joint_name, False) else (pos - lo)) \
            * hand.calibration.joint_to_motor_ratios_dict[motor_id]
        motor_pos[idx] = hand.motor_limits_dict[motor_id][0] + d + hand._wrap_offsets_dict.get(motor_id, 0.0)
    return motor_pos


def _old_loop_joint_to_motor(loop, cmd):
    inverted_term = loop._motor_limits_lower + (loop._joint_rom_upper - cmd) * loop._joint_to_motor_ratios
    forward_term = loop._motor_limits_lower + (cmd - loop._joint_rom_lower) * loop._joint_to_motor_ratios
    return np.where(loop._joint_inversion_mask, inverted_term, forward_term) + loop._wrap_offsets


def _loop(hand):
    controller = JointController(num_joints=len(hand._encoder_backed_joints()))
    reading = encoder_reading_from_joint_angles(hand, {j: 0.0 for j in hand._encoder_backed_joints()})
    loop = JointLoopThread(hand, StaticEncoderSource(reading), controller, target_hz=100)
    loop.prime_for_step()
    return loop


# --- linear parity ---------------------------------------------------------

@pytest.mark.parametrize("with_empty_map", [False, True])
def test_without_curves_the_map_is_bit_identical_to_the_linear_one(tmp_path, with_empty_map):
    hand = _hand(tmp_path, curves={} if with_empty_map else None)
    try:
        rng = np.random.default_rng(0)
        for _ in range(50):
            motor = rng.uniform(-0.8, 0.8, len(hand.config.motor_ids))
            assert hand._motor_to_joint_pos(motor) == _old_motor_to_joint(hand, motor)
            joints = {j: float(v) for j, v in zip(hand.config.joint_ids, rng.uniform(-40, 110, 17))}
            assert hand._joint_to_motor_pos(joints) == _old_joint_to_motor(hand, joints)
        loop = _loop(hand)
        assert loop._curve_bank is None
        for _ in range(50):
            cmd = rng.uniform(-40, 110, len(loop._joint_names))
            assert np.array_equal(loop._joint_to_motor_pos(cmd), _old_loop_joint_to_motor(loop, cmd))
    finally:
        hand.disconnect()


def test_joints_without_a_curve_stay_bit_identical(tmp_path):
    config_path = _copy_config(tmp_path)
    curves = {"index_mcp": _all_curves(config_path)["index_mcp"]}
    _write_map(tmp_path, curves)
    hand = make_calibrated_hand(config_path)
    try:
        motor = np.linspace(-0.6, 0.6, len(hand.config.motor_ids))
        new, old = hand._motor_to_joint_pos(motor), _old_motor_to_joint(hand, motor)
        assert {j: v for j, v in new.items() if j != "index_mcp"} == \
               {j: v for j, v in old.items() if j != "index_mcp"}
        assert new["index_mcp"] != old["index_mcp"]
    finally:
        hand.disconnect()


# --- forward / inverse -----------------------------------------------------

def test_curve_round_trip_and_extrapolation():
    curve = JointCurve(**_bent([-25.0, 100.0]))
    for s in np.linspace(-0.5, 1.5, 401):
        assert abs(curve.inverse(curve.forward(s)) - s) < 1e-12
    for deg in np.linspace(-80, 160, 401):
        assert abs(curve.forward(curve.inverse(deg)) - deg) < 1e-9
    assert curve.forward(0.0) == -25.0 and curve.forward(1.0) == 100.0
    # Past the stops the map continues at the stop-to-stop slope.
    assert curve.forward(1.2) == pytest.approx(100.0 + 0.2 * 125.0)
    assert curve.forward(-0.1) == pytest.approx(-25.0 - 0.1 * 125.0)


def test_curve_bank_matches_scalar_curves_bit_for_bit():
    curves = [JointCurve(**_bent([-25.0, 100.0])), JointCurve(**_bent([-30.0, 30.0], n=4)),
              JointCurve([0.0, 1.0], [-65.0, 35.0])]
    bank = CurveBank(curves)
    out = np.empty(3)
    rng = np.random.default_rng(1)
    for _ in range(200):
        deg = rng.uniform(-70, 130, 3)
        bank.inverse(deg, out)
        assert out.tolist() == [c.inverse(d) for c, d in zip(curves, deg)]


def test_hand_round_trip_below_1e6_with_curves_on_every_joint(tmp_path):
    config_path = _copy_config(tmp_path)
    _write_map(tmp_path, _all_curves(config_path))
    hand = make_calibrated_hand(config_path)
    try:
        assert set(hand._joint_curves) == set(hand.config.joint_ids)
        rng = np.random.default_rng(2)
        roms = hand.config.joint_roms_dict
        for _ in range(100):
            joints = {j: float(rng.uniform(lo, hi)) for j, (lo, hi) in roms.items()}
            back = hand._motor_to_joint_pos(np.array(hand._joint_to_motor_pos(joints)))
            assert max(abs(back[j] - joints[j]) for j in joints) < 1e-6
            motor = rng.uniform(-0.5, 0.5, len(hand.config.motor_ids))
            again = np.array(hand._joint_to_motor_pos(hand._motor_to_joint_pos(motor)))
            assert np.max(np.abs(again - motor)) < 1e-6
    finally:
        hand.disconnect()


def test_curve_ends_sit_on_the_motor_limits(tmp_path):
    """s = 0 is the joint's lower stop on the motor, whichever way the motor turns."""
    config_path = _copy_config(tmp_path)
    curves = _all_curves(config_path)
    _write_map(tmp_path, curves)
    hand = make_calibrated_hand(config_path)
    try:
        inverted = [j for j, inv in hand.config.joint_inversion_dict.items() if inv]
        forward = [j for j in hand.config.joint_ids if j not in inverted]
        assert inverted and forward
        for joint in hand.config.joint_ids:
            motor_id = hand.config.joint_to_motor_map[joint]
            idx = hand.config.motor_id_to_idx_dict[motor_id]
            lower, upper = hand.motor_limits_dict[motor_id]
            wrap = hand._wrap_offsets_dict.get(motor_id, 0.0)
            at_lo = hand._joint_to_motor_pos({joint: curves[joint]["deg"][0]})[idx] - wrap
            expect = upper if joint in inverted else lower
            assert at_lo == pytest.approx(expect, abs=1e-12)
    finally:
        hand.disconnect()


# --- the joint loop ----------------------------------------------------------

def test_joint_loop_matches_open_loop_map_with_curves(tmp_path):
    config_path = _copy_config(tmp_path)
    _write_map(tmp_path, _all_curves(config_path))
    hand = make_calibrated_hand(config_path)
    try:
        loop = _loop(hand)
        assert loop._curve_bank is not None
        assert len(loop._curve_idx) == len(loop._joint_names)
        idx_of = hand.config.motor_id_to_idx_dict
        for cmd in ([5.0, -5.0, 28.0, -28.0] * 5, [130.0] * 20, [-90.0] * 20):
            cmd = np.array(cmd[: len(loop._joint_names)], dtype=np.float64)
            loop_motor = loop._joint_to_motor_pos(cmd)
            open_loop = hand._joint_to_motor_pos(dict(zip(loop._joint_names, cmd)))
            for i, motor_id in enumerate(loop._motor_ids):
                assert loop_motor[i] == open_loop[idx_of[motor_id]]
    finally:
        hand.disconnect()


# --- stops ---------------------------------------------------------------------

def test_own_stops_override_config_and_survive_v211(tmp_path):
    path = _copy_config(tmp_path)
    raw = yaml.safe_load(open(path, encoding="utf-8"))
    raw["joint_roms_v211"] = {"wrist": [-40.0, 30.0], "index_mcp": [-20.0, 95.0]}
    (tmp_path / "config.yaml").write_text(yaml.safe_dump(raw), encoding="utf-8")
    _write_map(tmp_path, joint_roms={"wrist": [-63.25, 37.75]})
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.joint_roms_dict["wrist"] == [-63.25, 37.75]
    v211 = config.with_hardware_version(211)
    assert v211.joint_roms_dict["wrist"] == [-63.25, 37.75]
    assert v211.joint_roms_dict["index_mcp"] == [-20.0, 95.0]
    assert v211.joint_map is config.joint_map


# --- fallbacks -----------------------------------------------------------------

def test_no_file_means_no_map(tmp_path):
    config = OrcaHandConfig.from_config_path(config_path=_copy_config(tmp_path))
    assert config.joint_map is None


@pytest.mark.parametrize("doc", [
    "just a string",
    {"format_version": 2, "curves": {}},
    {"curves": {}},
])
def test_unreadable_file_falls_back_to_linear_with_a_warning(tmp_path, caplog, doc):
    config_path = _copy_config(tmp_path)
    (tmp_path / JOINT_MAP_FILE).write_text(yaml.safe_dump(doc), encoding="utf-8")
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        config = OrcaHandConfig.from_config_path(config_path=config_path)
    assert config.joint_map is None
    assert "linear joint map" in caplog.text


@pytest.mark.parametrize("knots, why", [
    ({"s": [0, 0.5, 1], "deg": [0, 50, 40]}, "non-monotone"),
    ({"s": [0, 0.5, 1], "deg": [0, 50, 50]}, "non-monotone"),
    ({"s": [0, 0.7, 0.5, 1], "deg": [0, 10, 20, 60]}, "s is not strictly increasing"),
    ({"s": [0.0], "deg": [0.0]}, "knots"),
    ({"s": np.linspace(0, 1, MAX_KNOTS + 1).tolist(),
      "deg": np.linspace(-20, 90, MAX_KNOTS + 1).tolist()}, "knots"),
    ({"s": [0, 1], "deg": [0, 1, 2]}, "s knots but"),
    ({"s": [0.1, 1], "deg": [0, 50]}, "from 0 to 1"),
    ({"s": [0, 1], "deg": [-25, 140]}, "outside the stops"),
    ({"s": [0, 1], "deg": [0, float("nan")]}, "non-finite"),
    ("not a mapping", "knots"),
])
def test_bad_curve_leaves_that_joint_linear(tmp_path, caplog, knots, why):
    config_path = _copy_config(tmp_path)
    pip = OrcaHandConfig.from_config_path(config_path=config_path).joint_roms_dict["index_pip"]
    _write_map(tmp_path, {"index_mcp": knots, "index_pip": _bent(pip)})
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        config = OrcaHandConfig.from_config_path(config_path=config_path)
    assert set(config.joint_map.curves) == {"index_pip"}
    assert "index_mcp curve rejected" in caplog.text and why in caplog.text
    assert "use the linear map" in caplog.text


def test_unknown_joint_and_bad_stops_are_ignored(tmp_path, caplog):
    config_path = _copy_config(tmp_path)
    wrist = OrcaHandConfig.from_config_path(config_path=config_path).joint_roms_dict["wrist"]
    _write_map(tmp_path, {"little_toe": _bent([0, 1])},
               joint_roms={"wrist": [10, -10], "index_mcp": [-20, 101]})
    with caplog.at_level(logging.WARNING, logger=LOGGER):
        config = OrcaHandConfig.from_config_path(config_path=config_path)
    assert config.joint_map.curves == {}
    assert config.joint_map.joint_roms == {"index_mcp": [-20.0, 101.0]}
    assert config.joint_roms_dict["index_mcp"] == [-20.0, 101.0]
    assert config.joint_roms_dict["wrist"] == wrist
    assert "little_toe" in caplog.text and "joint_roms[wrist]" in caplog.text


def test_load_joint_map_checks_curves_against_own_stops(tmp_path):
    path = tmp_path / JOINT_MAP_FILE
    _write_map(tmp_path, {"wrist": {"s": [0, 1], "deg": [-80.0, 50.0]}},
               joint_roms={"wrist": [-80.0, 50.0]})
    jm = load_joint_map(str(path), ["wrist"], {"wrist": [-65.0, 35.0]})
    assert set(jm.curves) == {"wrist"}


# --- the converter -----------------------------------------------------------------

def test_converter_on_ser0000(tmp_path, capsys):
    out = tmp_path / JOINT_MAP_FILE
    assert joint_map_from_visual.main([VISUAL, "-o", str(out)]) == 0
    doc = yaml.safe_load(out.read_text(encoding="utf-8"))
    src = json.load(open(VISUAL, encoding="utf-8"))
    assert doc["format_version"] == 1 and doc["hand_serial"] == "ser-0000"
    assert doc["model"] == "orcahand-touch-right"
    assert doc["joint_roms"] == src["stops"]["joint_roms"]
    assert doc["base_joint_roms"] == src["stops"]["packaged"]
    assert len(doc["curves"]) == 17
    for joint, knots in src["curve"]["knots"].items():
        lo, hi = src["stops"]["packaged"][joint]
        curve = doc["curves"][joint]
        assert curve["s"][0] == 0.0 and curve["s"][-1] == 1.0
        assert curve["deg"] == knots["true"]
        # s maps back onto quickfit's x under the base stops.
        x = [lo + s * (hi - lo) for s in curve["s"]]
        assert max(abs(a - b) for a, b in zip(x, knots["x"])) < 1e-3

    # Next to the hand's config, the file loads with every curve and stop.
    config_path = _copy_config(tmp_path, TOUCH_RIGHT)
    config = OrcaHandConfig.from_config_path(config_path=config_path)
    assert config.joint_map.hand_serial == "ser-0000"
    assert set(config.joint_map.curves) == set(config.joint_ids)
    assert config.joint_roms_dict == {j: [float(a), float(b)] for j, (a, b) in src["stops"]["joint_roms"].items()}
    # The curve reproduces quickfit's "true = interp(x, knots.x, knots.true)".
    for joint, knots in src["curve"]["knots"].items():
        lo, hi = src["stops"]["packaged"][joint]
        curve = config.joint_map.curves[joint]
        for x in np.linspace(lo, hi, 25):
            assert curve.forward((x - lo) / (hi - lo)) == pytest.approx(
                float(np.interp(x, knots["x"], knots["true"])), abs=0.02)


def test_converter_leaves_a_flat_curve_out():
    src = json.load(open(VISUAL, encoding="utf-8"))
    src["curve"]["knots"]["wrist"]["true"][3] = src["curve"]["knots"]["wrist"]["true"][2]
    doc, skipped = from_visual_calibration(src)
    assert "wrist" not in doc["curves"] and "non-monotone" in skipped["wrist"]
    assert math.isclose(doc["joint_roms"]["wrist"][0], -64.46)
