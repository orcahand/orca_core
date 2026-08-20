"""Tests for tools/extract_urdf_kinematics.py: config resolution and sign coherence."""

import importlib.util
from pathlib import Path

import pytest

_TOOL_PATH = Path(__file__).resolve().parents[1] / "tools" / "extract_urdf_kinematics.py"
_spec = importlib.util.spec_from_file_location("extract_urdf_kinematics", _TOOL_PATH)
extractor = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(extractor)

VERIFIED = "verified (mirrored limits)"
UNVERIFIED = "unverified (symmetric limits)"


def _abd_entry(finger: str, sign: int = 1, axis=(0.0, 0.0, 1.0)) -> dict:
    return {
        "joint": f"{finger}_abd",
        "xyz": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0],
        "axis": list(axis),
        "sign": sign,
    }


def _make_chains(entries: dict[str, dict]) -> dict:
    return {"fingers": {finger: [entry] for finger, entry in entries.items()}}


def _default_entries() -> dict[str, dict]:
    return {finger: _abd_entry(finger) for finger in extractor.ABD_FINGERS}


class TestLoadJointRoms:
    """--config accepts the same model selectors as the rest of orca_core."""

    def test_no_config_yields_no_roms(self):
        assert extractor.load_joint_roms(None) == {}

    def test_model_name_resolves_through_the_package(self):
        roms = extractor.load_joint_roms("orcahand-right")
        assert "wrist" in roms and len(roms["wrist"]) == 2

    def test_config_yaml_path_matches_the_model_name(self):
        from orca_core.utils.utils import get_model_path

        config_path = Path(get_model_path("orcahand-right")) / "config.yaml"
        assert extractor.load_joint_roms(str(config_path)) == extractor.load_joint_roms(
            "orcahand-right"
        )

    def test_unknown_model_is_fatal_rather_than_an_empty_audit(self):
        with pytest.raises(FileNotFoundError):
            extractor.load_joint_roms("orcahand-nonexistent")


class TestResolveAbductionSigns:
    def test_unverified_antiparallel_sign_is_flipped_and_audit_rewritten(self):
        entries = _default_entries()
        entries["middle"] = _abd_entry("middle", sign=1, axis=(0.0, 0.0, -1.0))
        audits = {f"{f}_abd": UNVERIFIED for f in extractor.ABD_FINGERS}
        audits["index_abd"] = VERIFIED

        extractor.resolve_abduction_signs(_make_chains(entries), audits)

        assert entries["middle"]["sign"] == -1
        assert audits["middle_abd"] == "resolved by axis coherence with index_abd"

    def test_verified_antiparallel_sign_raises_instead_of_silently_flipping(self):
        entries = _default_entries()
        entries["middle"] = _abd_entry("middle", sign=1, axis=(0.0, 0.0, -1.0))
        audits = {f"{f}_abd": UNVERIFIED for f in extractor.ABD_FINGERS}
        audits["index_abd"] = VERIFIED
        audits["middle_abd"] = "verified (limits lean opposite ways)"

        with pytest.raises(ValueError, match="middle_abd"):
            extractor.resolve_abduction_signs(_make_chains(entries), audits)

        assert entries["middle"]["sign"] == 1
        assert audits["middle_abd"] == "verified (limits lean opposite ways)"

    def test_verified_parallel_sign_and_audit_are_untouched(self):
        entries = _default_entries()
        audits = {f"{f}_abd": UNVERIFIED for f in extractor.ABD_FINGERS}
        audits["index_abd"] = VERIFIED
        audits["ring_abd"] = "verified (limits match)"

        extractor.resolve_abduction_signs(_make_chains(entries), audits)

        assert entries["ring"]["sign"] == 1
        assert audits["ring_abd"] == "verified (limits match)"
        assert audits["pinky_abd"] == "resolved by axis coherence with index_abd"

    def test_no_verified_reference_is_fatal(self):
        audits = {f"{f}_abd": UNVERIFIED for f in extractor.ABD_FINGERS}
        with pytest.raises(ValueError, match="cannot be anchored"):
            extractor.resolve_abduction_signs(_make_chains(_default_entries()), audits)

    def test_non_parallel_axis_is_fatal(self):
        entries = _default_entries()
        entries["pinky"] = _abd_entry("pinky", axis=(1.0, 0.0, 0.0))
        audits = {f"{f}_abd": UNVERIFIED for f in extractor.ABD_FINGERS}
        audits["index_abd"] = VERIFIED

        with pytest.raises(ValueError, match="not parallel"):
            extractor.resolve_abduction_signs(_make_chains(entries), audits)
