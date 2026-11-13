"""models/v2/shared.yaml: the defaults every v2 model builds on, and the
measured ranges a config can take for one hand.

``tests/fixtures/v2_configs_0.13.1/`` holds the eight v2 configs exactly as
0.13.0 and 0.13.1 shipped them (``git show release/0.13.1:<path>``; the two
releases' configs are identical). They have no shared.yaml beside them, so
they load as written: every packaged model must load to the same effective
config as its old, complete self.
"""

import dataclasses
import math
import os

import pytest
import yaml

import orca_core
from orca_core import BaseHandConfig, MockOrcaHand, OrcaHandConfig, OrcaHandTouchConfig, load_config, write_standalone_config
from orca_core.calibration import joint_fingerprints
from orca_core.constants import SUPPORTED_MOTOR_TYPES
from orca_core.hand_config import HandConfigValidationError, shared_config_path
from orca_core.hardware.motor_factory import motor_client_class
from tests._helpers import copy_model_config

V2_MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2")
V1_MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v1")
REFERENCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures", "v2_configs_0.13.1")
MODELS = sorted(m for m in os.listdir(V2_MODELS) if os.path.isdir(os.path.join(V2_MODELS, m)))
HARDWARE_VERSIONS = [None, 20, 21, 210, 211]

# The guided-tensioning holds 0.13.0 kept in constants and 0.13.1 on the motor clients.
HOLDS_0_13 = {
    "dynamixel": {"abd": 200, "mcp": 140, "pip": 100},
    "feetech": {"abd": 380, "mcp": 320, "pip": 280},
}
UNCHANGED_FIELDS = [
    f.name for f in dataclasses.fields(OrcaHandTouchConfig)
    if f.name not in ("config_path", "calibration_path", "guided_tension_currents_dict")
]


def _read(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load(path, hardware_version):
    cls = OrcaHandTouchConfig if "sensors" in load_config(path) else OrcaHandConfig
    return cls.from_config_path(config_path=path).with_hardware_version(hardware_version)


def _effective(config):
    """What a hand runs with: every field, plus the guided-tension and current
    figures each motor family resolves to."""
    seen = {name: getattr(config, name) for name in UNCHANGED_FIELDS if hasattr(config, name)}
    seen["joint_order"] = list(config.joint_roms_dict)
    for family in SUPPORTED_MOTOR_TYPES:
        resolved = dataclasses.replace(config, motor_type=family).with_family_currents(
            motor_client_class(family))
        seen[family] = {
            "currents": resolved.guided_tension_currents_for(),
            "holds": {j: resolved.tension_hold_current_for(j) for j in config.joint_ids if j != "wrist"},
            "calibration": {j: resolved.calibration_current_for(j) for j in config.joint_ids},
            "travel": {j: resolved.expected_motor_travel_deg(j) for j in config.joint_ids},
        }
    return seen


def test_every_v2_model_has_its_0_13_reference():
    assert MODELS and MODELS == sorted(os.listdir(REFERENCE))
    assert not os.path.exists(os.path.join(REFERENCE, "shared.yaml"))


@pytest.mark.parametrize("hardware_version", HARDWARE_VERSIONS, ids=str)
@pytest.mark.parametrize("model", MODELS)
def test_every_v2_model_loads_as_0_13_did(model, hardware_version):
    new = _load(os.path.join(V2_MODELS, model, "config.yaml"), hardware_version)
    old = _load(os.path.join(REFERENCE, model, "config.yaml"), hardware_version)
    assert type(new) is type(old)
    assert _effective(new) == _effective(old)
    # A calibration stored under 0.13's config keeps every joint.
    assert joint_fingerprints(new) == joint_fingerprints(old)


@pytest.mark.parametrize("model", MODELS)
def test_the_effective_document_is_the_0_13_config(model):
    """Key for key, the merged yaml is the old file; only the guided-tension
    block now carries the family holds the old code supplied."""
    merged = load_config(os.path.join(V2_MODELS, model, "config.yaml"))
    old = _read(os.path.join(REFERENCE, model, "config.yaml"))
    tension = merged.pop("guided_tension_currents")
    old_tension = old.pop("guided_tension_currents", {})
    assert merged == old
    assert list(merged["joint_roms"]) == list(old["joint_roms"])
    for family, holds in HOLDS_0_13.items():
        expected = {**holds, **old_tension.get(family, {}).get("hold", {})}
        assert tension[family] == {"hold": expected}


@pytest.mark.parametrize("model", MODELS)
def test_the_0_13_holds_are_what_every_model_resolves(model):
    config = OrcaHandConfig.from_config_path(config_path=os.path.join(V2_MODELS, model, "config.yaml"))
    for family, holds in HOLDS_0_13.items():
        hold = config.guided_tension_currents_for(family)["hold"]
        assert {g: hold[g] for g in holds} == holds
    assert "touch" not in model or config.guided_tension_currents_for("dynamixel")["hold"]["thumb_mcp"] == 100


def test_the_models_hold_only_what_differs():
    for model in MODELS:
        own = _read(os.path.join(V2_MODELS, model, "config.yaml"))
        for key in ("joint_roms", "calibration_current_v21", "joint_motor_travel_v21"):
            assert key not in own, (model, key)


# --- the packaged holds as a constant ------------------------------------------


def test_default_guided_tension_currents_are_shared_yamls_as_plain_dicts():
    import copy
    import json
    import pickle

    from orca_core.constants import DEFAULT_GUIDED_TENSION_CURRENTS as defaults

    shared = _read(os.path.join(V2_MODELS, "shared.yaml"))["guided_tension_currents"]
    assert defaults == shared
    assert {f: e["hold"] for f, e in shared.items()} == HOLDS_0_13
    assert type(defaults) is dict and type(defaults["dynamixel"]["hold"]) is dict
    assert copy.deepcopy(defaults) == pickle.loads(pickle.dumps(defaults)) == json.loads(json.dumps(defaults))
    assert yaml.safe_load(yaml.safe_dump(defaults)) == defaults


def test_changing_the_default_guided_tension_currents_reaches_no_config(right):
    from orca_core import constants

    constants.DEFAULT_GUIDED_TENSION_CURRENTS["dynamixel"]["hold"]["abd"] = 1
    assert constants.DEFAULT_GUIDED_TENSION_CURRENTS["dynamixel"]["hold"]["abd"] == 200
    assert right.guided_tension_currents_for("dynamixel")["hold"]["abd"] == 200


def test_a_config_with_no_shared_yaml_still_gets_the_packaged_holds(tmp_path):
    copy_model_config(os.path.join(V2_MODELS, "orcahand-right", "config.yaml"), tmp_path / "config.yaml")
    raw = _read(tmp_path / "config.yaml")
    raw.pop("guided_tension_currents")
    (tmp_path / "config.yaml").write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    config = OrcaHandConfig.from_config_path(config_path=str(tmp_path / "config.yaml"))
    assert config.guided_tension_currents_dict == {}
    for family, holds in HOLDS_0_13.items():
        assert config.guided_tension_currents_for(family)["hold"] == holds


# --- load_config ---------------------------------------------------------------


@pytest.fixture
def packaged_tree(tmp_path, monkeypatch):
    """``tmp_path/models`` standing in for the package's models folder."""
    from orca_core.utils import utils

    monkeypatch.setattr(utils, "_get_models_dir", lambda: str(tmp_path / "models"))
    return tmp_path


def _tree(tmp_path, shared, model):
    (tmp_path / "models").mkdir()
    (tmp_path / "models" / "shared.yaml").write_text(yaml.safe_dump(shared, sort_keys=False))
    (tmp_path / "models" / "hand").mkdir()
    path = tmp_path / "models" / "hand" / "config.yaml"
    path.write_text(yaml.safe_dump(model, sort_keys=False))
    return str(path)


def test_shared_yaml_merges_under_the_model_and_the_model_wins(packaged_tree):
    tmp_path = packaged_tree
    path = _tree(
        tmp_path,
        {"a": 1, "roms": {"x": [0, 1], "y": [0, 2]}, "nested": {"f": {"hold": {"abd": 1, "mcp": 2}}}},
        {"a": 2, "roms": {"y": [5, 6]}, "nested": {"f": {"hold": {"mcp": 9, "thumb": 3}}}, "b": None},
    )
    assert shared_config_path(path) == str(tmp_path / "models" / "shared.yaml")
    merged = load_config(path)
    assert merged == {
        "a": 2,
        "roms": {"x": [0, 1], "y": [5, 6]},
        "nested": {"f": {"hold": {"abd": 1, "mcp": 9, "thumb": 3}}},
        "b": None,
    }
    assert list(merged["roms"]) == ["x", "y"], "a model's override keeps the shared order"


def test_a_generations_shared_yaml_sets_its_holds(packaged_tree):
    doc = _read(os.path.join(V2_MODELS, "orcahand-right", "config.yaml"))
    shared = _read(os.path.join(V2_MODELS, "shared.yaml"))
    shared["guided_tension_currents"]["feetech"]["hold"]["mcp"] = 333
    config = OrcaHandConfig.from_config_path(config_path=_tree(packaged_tree, shared, doc))
    assert config.joint_roms_dict["wrist"] == [-64.46, 38.51]
    assert config.guided_tension_currents_for("feetech")["hold"]["mcp"] == 333


def _without_wrist(doc):
    doc["joint_ids"].remove("wrist")
    doc["motor_ids"].remove(abs(doc["joint_to_motor_map"]["wrist"]))
    for key in ("joint_to_motor_map", "joint_to_motor_map_v21", "neutral_position"):
        doc.get(key, {}).pop("wrist", None)
    for key in ("calibration_sequence", "manual_calibration_sequence"):
        for step in doc.get(key, []):
            step["joints"].pop("wrist", None)
        doc[key] = [step for step in doc.get(key, []) if step["joints"]]
    return doc


def test_a_model_without_a_joint_drops_the_shared_entries_for_it(packaged_tree):
    shared = _read(os.path.join(V2_MODELS, "shared.yaml"))
    shared["joint_roms_v211"] = {"wrist": [-1, 1], "index_mcp": [-2, 2]}
    doc = _without_wrist(_read(os.path.join(V2_MODELS, "orcahand-right", "config.yaml")))
    path = _tree(packaged_tree, shared, doc)
    merged = load_config(path)
    for key in ("joint_roms", "joint_roms_v211", "joint_motor_travel_v21"):
        assert "wrist" not in merged[key], key
    assert list(merged["joint_roms"]) == [j for j in shared["joint_roms"] if j != "wrist"]
    assert merged["joint_roms_v211"] == {"index_mcp": [-2, 2]}
    config = OrcaHandConfig.from_config_path(config_path=path)
    config.validate()
    assert "wrist" not in config.joint_ids and "wrist" not in config.joint_roms_dict


def test_a_symlinked_packaged_config_takes_its_shared_yaml(tmp_path):
    source = os.path.join(V2_MODELS, "orcahand-left", "config.yaml")
    (tmp_path / "hand").mkdir()
    link = tmp_path / "hand" / "config.yaml"
    link.symlink_to(source)
    assert shared_config_path(str(link)) == os.path.realpath(os.path.join(V2_MODELS, "shared.yaml"))
    assert load_config(str(link)) == load_config(source)
    OrcaHandConfig.from_config_path(config_path=str(link)).validate()


def test_a_shared_yaml_outside_the_package_is_never_merged(tmp_path):
    """A shared.yaml near a config the package does not ship (an unrelated file,
    or a generation folder copied out) has no effect: such a config is complete."""
    root = tmp_path / "robots"
    (root / "hand").mkdir(parents=True)
    (root / "shared.yaml").write_text(yaml.safe_dump({"joint_roms": {"index_mcp": [0, 1]}, "max_current": 1}))
    source = os.path.join(V2_MODELS, "orcahand-right", "config.yaml")
    copy_model_config(source, root / "hand" / "config.yaml")
    path = str(root / "hand" / "config.yaml")
    assert shared_config_path(path) is None
    assert load_config(path) == load_config(source)

    (root / "hand" / "config.yaml").write_text(open(source, encoding="utf-8").read())
    (root / "shared.yaml").write_text(open(os.path.join(V2_MODELS, "shared.yaml"), encoding="utf-8").read())
    with pytest.raises(HandConfigValidationError, match="sets no joint_roms"):
        OrcaHandConfig.from_config_path(config_path=path)


@pytest.mark.parametrize("model", ["orcahand-left", "orcahand-right"])
def test_v1_models_are_read_as_written(model):
    path = os.path.join(V1_MODELS, model, "config.yaml")
    assert shared_config_path(path) is None
    assert load_config(path) == _read(path)


def test_load_config_resolves_a_packaged_model_by_name():
    assert load_config(model_name="orcahand-touch-left") == load_config(
        os.path.join(V2_MODELS, "orcahand-touch-left", "config.yaml"))


def test_a_complete_config_elsewhere_loads_as_written(tmp_path):
    source = os.path.join(V2_MODELS, "orcahand-touch-left", "config.yaml")
    copy_model_config(source, tmp_path / "config.yaml")
    copy = OrcaHandTouchConfig.from_config_path(config_path=str(tmp_path / "config.yaml"))
    packaged = OrcaHandTouchConfig.from_config_path(config_path=source)
    assert _effective(copy) == _effective(packaged)


@pytest.mark.parametrize("cls", [OrcaHandConfig, BaseHandConfig])
def test_a_slim_config_copied_alone_is_refused_naming_what_it_lacks(tmp_path, cls):
    source = os.path.join(V2_MODELS, "orcahand-right", "config.yaml")
    (tmp_path / "config.yaml").write_text(open(source, encoding="utf-8").read())
    with pytest.raises(HandConfigValidationError, match="write_standalone_config") as err:
        cls.from_config_path(config_path=str(tmp_path / "config.yaml"))
    assert "sets no joint_roms, calibration_current_v21, joint_motor_travel_v21." in str(err.value)


def test_a_standalone_v2_config_missing_shared_keys_warns_once(tmp_path, caplog):
    source = os.path.join(V2_MODELS, "orcahand-right", "config.yaml")
    doc = _read(source)
    doc["joint_roms"] = load_config(source)["joint_roms"]
    (tmp_path / "config.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))
    path = str(tmp_path / "config.yaml")
    with caplog.at_level("WARNING", logger="orca_core.hand_config"):
        config = OrcaHandConfig.from_config_path(config_path=path).with_hardware_version(210)
        OrcaHandConfig.from_config_path(config_path=path)
    warnings = [r.getMessage() for r in caplog.records if "sets no" in r.getMessage()]
    assert len(warnings) == 1
    assert "calibration_current_v21, joint_motor_travel_v21" in warnings[0]
    assert config.expected_motor_travel_deg("index_mcp") is None


def test_a_complete_v1_config_elsewhere_loads_without_a_warning(tmp_path, caplog):
    write_standalone_config(str(tmp_path), config_path=os.path.join(V1_MODELS, "orcahand-right", "config.yaml"))
    with caplog.at_level("WARNING", logger="orca_core.hand_config"):
        OrcaHandConfig.from_config_path(config_path=str(tmp_path / "config.yaml"))
    assert not [r for r in caplog.records if "sets no" in r.getMessage()]


# --- write_standalone_config -----------------------------------------------------


@pytest.mark.parametrize("model", MODELS)
def test_a_standalone_copy_loads_as_its_packaged_model(tmp_path, model, caplog):
    source = os.path.join(V2_MODELS, model, "config.yaml")
    with caplog.at_level("WARNING", logger="orca_core.hand_config"):
        path = write_standalone_config(str(tmp_path / "hands" / "mine" / "config.yaml"), model_name=model)
        assert path == str(tmp_path / "hands" / "mine" / "config.yaml")
        assert shared_config_path(path) is None and load_config(path) == load_config(source)
        for hardware_version in HARDWARE_VERSIONS:
            assert _effective(_load(path, hardware_version)) == _effective(_load(source, hardware_version))
    assert not [r for r in caplog.records if "sets no" in r.getMessage()]


def test_a_standalone_copy_into_a_folder_is_its_config_yaml(tmp_path):
    path = write_standalone_config(str(tmp_path), model_name="orcahand-touch-left")
    assert path == os.path.join(str(tmp_path), "config.yaml")
    assert _read(path) == load_config(model_name="orcahand-touch-left")


def test_a_standalone_copy_never_lands_in_the_package(tmp_path):
    with pytest.raises(ValueError, match="inside the package"):
        write_standalone_config(os.path.join(V2_MODELS, "orcahand-right"), model_name="orcahand-right")
    link = tmp_path / "link.yaml"
    link.symlink_to(os.path.join(V2_MODELS, "orcahand-left", "config.yaml"))
    with pytest.raises(ValueError, match="inside the package"):
        write_standalone_config(str(link), model_name="orcahand-right")


def test_a_shared_yaml_that_is_not_a_mapping_is_refused(packaged_tree):
    path = _tree(packaged_tree, {}, {"a": 1})
    (packaged_tree / "models" / "shared.yaml").write_text("- a\n- b\n")
    with pytest.raises(HandConfigValidationError, match="mapping"):
        load_config(path)


# --- with_measured_ranges --------------------------------------------------------


@pytest.fixture
def right():
    return OrcaHandConfig.from_config_path(config_path=os.path.join(V2_MODELS, "orcahand-right", "config.yaml"))


def test_measured_ranges_become_the_clamp(right, monkeypatch):
    from orca_core import OrcaJointPositions

    # Another test's hand may have registered a joint set without index_mcp.
    monkeypatch.setattr(OrcaJointPositions, "_default_joint_ids", None)
    measured = right.with_measured_ranges({"index_mcp": [-10, 90], "wrist": (-50.5, 30)})
    assert measured.joint_roms_dict["index_mcp"] == [-10.0, 90.0]
    assert measured.joint_roms_dict["wrist"] == [-50.5, 30.0]
    assert measured.measured_joint_roms_dict == {"index_mcp": [-10.0, 90.0], "wrist": [-50.5, 30.0]}
    assert list(measured.joint_roms_dict) == list(right.joint_roms_dict)
    assert all(measured.joint_roms_dict[j] == right.joint_roms_dict[j]
               for j in right.joint_ids if j not in ("index_mcp", "wrist"))
    assert right.measured_joint_roms_dict == {} and right.joint_roms_dict["index_mcp"] == [-22.75, 103.69]

    clamped = measured.clamp_joint_positions(OrcaJointPositions.from_dict({"index_mcp": 100.0}))
    assert clamped.as_dict()["index_mcp"] == 90.0


def test_measured_ranges_accumulate_and_survive_a_v211_overlay(right):
    config = dataclasses.replace(right, joint_roms_v211_dict={"index_mcp": [-1, 1], "index_pip": [-2, 2]})
    config = config.with_measured_ranges({"index_mcp": [-10, 90]}).with_measured_ranges({"ring_pip": [0, 80]})
    assert set(config.measured_joint_roms_dict) == {"index_mcp", "ring_pip"}
    v211 = config.with_hardware_version(211)
    assert v211.joint_roms_dict["index_mcp"] == [-10.0, 90.0]
    assert v211.joint_roms_dict["index_pip"] == [-2, 2]
    assert v211.joint_roms_dict["ring_pip"] == [0.0, 80.0]


@pytest.mark.parametrize("roms, match", [
    ({"elbow": [0, 1]}, "not one of this hand's joints"),
    ({"index_mcp": [0, 1, 2]}, r"\[min, max\]"),
    ({"index_mcp": [5, 5]}, "min < max"),
    ({"index_mcp": [6, 5]}, "min < max"),
    ({"index_mcp": "0,90"}, r"\[min, max\]"),
    ({"index_mcp": [0, math.nan]}, r"\[min, max\]"),
    ({"index_mcp": [0, math.inf]}, r"\[min, max\]"),
    ({"index_mcp": [False, True]}, r"\[min, max\]"),
    ({"index_mcp": None}, r"\[min, max\]"),
    ([("index_mcp", [0, 1])], "must map a joint"),
])
def test_measured_ranges_are_validated(right, roms, match):
    with pytest.raises(HandConfigValidationError, match=match):
        right.with_measured_ranges(roms)


def test_measured_ranges_win_over_an_encoder_measured_span_in_the_calibration_frame(tmp_path):
    copy_model_config(os.path.join(V2_MODELS, "orcahand-right", "config.yaml"), tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    hand.calibration = dataclasses.replace(
        hand.calibration, joint_roms_measured_dict={"index_mcp": [-30.0, 103.69], "index_pip": [-20.0, 106.43]})
    hand.config = hand.config.with_measured_ranges({"index_mcp": [-15.0, 95.0]})
    assert hand.effective_joint_roms_dict["index_mcp"] == [-15.0, 95.0]
    assert hand.effective_joint_roms_dict["index_pip"] == [-20.0, 106.43]


def test_a_calibration_maps_motor_travel_onto_the_measured_range(tmp_path):
    copy_model_config(os.path.join(V2_MODELS, "orcahand-right", "config.yaml"), tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    hand.config = hand.config.with_measured_ranges({"index_mcp": [-15.0, 95.0]})
    ok, msg = hand.connect()
    assert ok, msg
    try:
        hand.calibrate(persist=False)
        motor = hand.config.joint_to_motor_map["index_mcp"]
        lo, hi = hand.motor_limits_dict[motor]
        assert hand.joint_to_motor_ratios_dict[motor] == pytest.approx((hi - lo) / 110.0)
        other = hand.config.joint_to_motor_map["index_pip"]
        lo, hi = hand.motor_limits_dict[other]
        rom = hand.config.joint_roms_dict["index_pip"]
        assert hand.joint_to_motor_ratios_dict[other] == pytest.approx((hi - lo) / (rom[1] - rom[0]))
    finally:
        hand.disconnect()
