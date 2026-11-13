"""models/v2/shared.yaml: the defaults every v2 model builds on.

``tests/fixtures/v2_configs_0.13.1/`` holds the eight v2 configs exactly as
0.13.0 and 0.13.1 shipped them (``git show release/0.13.1:<path>``; the two
releases' configs are identical). They have no shared.yaml beside them, so
they load as written: every packaged model must load to the same effective
config as its old, complete self.
"""

import dataclasses
import os
from types import MappingProxyType

import pytest
import yaml

import orca_core
from orca_core import OrcaHandConfig, OrcaHandTouchConfig, load_config
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


# --- the read-only view of the holds ------------------------------------------


def test_default_guided_tension_currents_is_a_read_only_view_of_shared_yaml():
    from orca_core.constants import DEFAULT_GUIDED_TENSION_CURRENTS

    shared = _read(os.path.join(V2_MODELS, "shared.yaml"))["guided_tension_currents"]
    assert {f: {"hold": dict(e["hold"])} for f, e in DEFAULT_GUIDED_TENSION_CURRENTS.items()} == shared
    assert {f: dict(e["hold"]) for f, e in shared.items()} == HOLDS_0_13
    assert isinstance(DEFAULT_GUIDED_TENSION_CURRENTS, MappingProxyType)
    with pytest.raises(TypeError):
        DEFAULT_GUIDED_TENSION_CURRENTS["dynamixel"]["hold"]["abd"] = 1


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


def _tree(tmp_path, shared, model):
    (tmp_path / "models").mkdir()
    (tmp_path / "models" / "shared.yaml").write_text(yaml.safe_dump(shared, sort_keys=False))
    (tmp_path / "models" / "hand").mkdir()
    path = tmp_path / "models" / "hand" / "config.yaml"
    path.write_text(yaml.safe_dump(model, sort_keys=False))
    return str(path)


def test_shared_yaml_merges_under_the_model_and_the_model_wins(tmp_path):
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


def test_a_model_folders_own_shared_yaml_sets_its_holds(tmp_path):
    doc = load_config(os.path.join(V2_MODELS, "orcahand-right", "config.yaml"))
    shared = {k: doc.pop(k) for k in ("joint_roms", "guided_tension_currents")}
    shared["guided_tension_currents"]["feetech"]["hold"]["mcp"] = 333
    config = OrcaHandConfig.from_config_path(config_path=_tree(tmp_path, shared, doc))
    assert config.joint_roms_dict["wrist"] == [-64.46, 38.51]
    assert config.guided_tension_currents_for("feetech")["hold"]["mcp"] == 333


@pytest.mark.parametrize("model", ["orcahand-left", "orcahand-right"])
def test_v1_models_are_read_as_written(model):
    path = os.path.join(V1_MODELS, model, "config.yaml")
    assert not os.path.exists(shared_config_path(path))
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


def test_a_slim_config_copied_alone_is_refused_with_the_way_out(tmp_path):
    source = os.path.join(V2_MODELS, "orcahand-right", "config.yaml")
    (tmp_path / "config.yaml").write_text(open(source, encoding="utf-8").read())
    with pytest.raises(HandConfigValidationError, match="shared.yaml.*load_config"):
        OrcaHandConfig.from_config_path(config_path=str(tmp_path / "config.yaml"))


def test_a_shared_yaml_that_is_not_a_mapping_is_refused(tmp_path):
    path = _tree(tmp_path, {}, {"a": 1})
    (tmp_path / "models" / "shared.yaml").write_text("- a\n- b\n")
    with pytest.raises(HandConfigValidationError, match="mapping"):
        load_config(path)
