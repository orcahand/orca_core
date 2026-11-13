"""setup.yaml lives in the hand's folder and keeps only what is current;
everything else moves to the hand's archive, one file per record."""

import os
from types import SimpleNamespace

import pytest

import orca_core
from orca_core import MockOrcaHand
from orca_core.maintenance import setup_file
from orca_core.utils import read_yaml


@pytest.fixture
def config(tmp_path):
    return SimpleNamespace(hand_dir=str(tmp_path / "hand"), calibration_path="")


def slack(n):
    return {"recorded_at": f"2026-11-1{n}T10:00:00+00:00", "slack": n}


class TestWhere:
    def test_a_hand_s_records_go_to_its_folder(self, tmp_path):
        import shutil

        model = tmp_path / "orcahand-right"
        model.mkdir()
        shutil.copy(os.path.join(os.path.dirname(orca_core.__file__), "models", "v2",
                                 "orcahand-right", "config.yaml"), model / "config.yaml")
        hand = MockOrcaHand(config_path=str(model / "config.yaml"))
        setup_file.write_section(hand.config, "last_tension", {"mode": "hold"})
        assert setup_file.setup_path(hand.config) == os.path.join(hand.hand_dir, "setup.yaml")
        assert read_yaml(setup_file.setup_path(hand.config))["last_tension"] == {"mode": "hold"}
        assert sorted(p.name for p in model.iterdir()) == ["config.yaml"]

    def test_a_config_of_no_hand_is_refused_rather_than_written_beside_a_model(self):
        config = SimpleNamespace(hand_dir="", calibration_path="")
        with pytest.raises(ValueError, match="no hand folder"):
            setup_file.write_section(config, "last_tension", {"mode": "hold"})

    def test_nothing_is_ever_written_inside_the_package(self):
        packaged = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right")
        config = SimpleNamespace(hand_dir=packaged, calibration_path="")
        with pytest.raises(PermissionError):
            setup_file.write_section(config, "last_tension", {"mode": "hold"})
        assert not os.path.exists(os.path.join(packaged, "setup.yaml"))

    def test_the_file_carries_its_schema(self, config):
        setup_file.write_section(config, "last_tension", {"mode": "hold"})
        assert read_yaml(setup_file.setup_path(config))["schema"] == 1


class TestSlackTests:
    def test_only_the_newest_stays_and_the_rest_are_archived(self, config):
        for n in (1, 2, 3):
            setup_file.update_section(
                config, "slack_tests",
                lambda data, n=n: {"tests": {**(data or {}).get("tests", {}), f"t{n}": slack(n)}})
        assert setup_file.read_section(config, "slack_tests") == {"tests": {"t3": slack(3)}}
        assert setup_file.read_archive(config, "slack_tests") == {"t1": slack(1), "t2": slack(2)}
        assert setup_file.read_archive(config, "slack_tests", "t2") == slack(2)

    def test_the_value_update_section_returns_is_what_was_kept(self, config):
        setup_file.write_section(config, "slack_tests", {"tests": {"t1": slack(1)}})
        kept = setup_file.update_section(
            config, "slack_tests", lambda data: {"tests": {**data["tests"], "t2": slack(2)}})
        assert kept == {"tests": {"t2": slack(2)}}


class TestSweeps:
    def sweep(self, day):
        return {"recorded_at": f"2026-11-{day:02d}T09:00:00+00:00",
                "results": [{"joint": "index_mcp", "samples": [[0.0, 10.0], [1.0, 12.0]]}]}

    def test_the_newest_and_the_reference_stay(self, config):
        sweeps = {f"GT_sweep_{d:02d}": self.sweep(d) for d in (1, 2, 3)}
        setup_file.write_section(
            config, "tension_sweeps", {"reference": "GT_sweep_01", "sweeps": sweeps})
        kept = setup_file.read_section(config, "tension_sweeps")
        assert sorted(kept["sweeps"]) == ["GT_sweep_01", "GT_sweep_03"]
        assert kept["reference"] == "GT_sweep_01"
        # Kept sweeps keep their data: the console scores against the reference.
        assert kept["sweeps"]["GT_sweep_01"]["results"][0]["samples"]

    def test_every_sweep_s_raw_data_is_archived(self, config):
        sweeps = {f"GT_sweep_{d:02d}": self.sweep(d) for d in (1, 2)}
        setup_file.write_section(config, "tension_sweeps", {"reference": None, "sweeps": sweeps})
        assert setup_file.read_archive(config, "tension_sweeps") == sweeps

    def test_rewriting_the_kept_sweeps_does_not_duplicate_the_archive(self, config):
        data = {"reference": None, "sweeps": {"GT_sweep_01": self.sweep(1)}}
        setup_file.write_section(config, "tension_sweeps", data)
        setup_file.write_section(config, "tension_sweeps", data)
        folder = os.path.join(config.hand_dir, "archive", "tension_sweeps")
        assert len(os.listdir(folder)) == 1


class TestTravelBaselines:
    def test_the_active_and_newest_baseline_and_the_newest_full_snapshot_stay(self, config):
        def entry(day, **extra):
            return {"recorded_at": f"2026-11-{day:02d}T09:00:00+00:00", "travel": {}, **extra}

        setup_file.write_section(config, "travel_baselines", {
            "active": "b1",
            "baselines": {"b1": entry(1), "b2": entry(2), "b3": entry(3)},
            "snapshots": {"s1": entry(1), "s2": entry(2), "s3": entry(3, partial=True)},
        })
        kept = setup_file.read_section(config, "travel_baselines")
        assert sorted(kept["baselines"]) == ["b1", "b3"]
        assert sorted(kept["snapshots"]) == ["s2", "s3"]
        assert sorted(setup_file.read_archive(config, "travel_baselines-baselines")) == ["b2"]
        assert sorted(setup_file.read_archive(config, "travel_baselines-snapshots")) == ["s1"]


class TestUnprunedSections:
    def test_tension_progress_is_kept_whole(self, config):
        progress = {"hands": {"ser-0001": {"rounds": 3}, "ser-0002": {"rounds": 1}}}
        setup_file.write_section(config, "tension_progress", progress)
        assert setup_file.read_section(config, "tension_progress") == progress
        assert not os.path.exists(os.path.join(config.hand_dir, "archive"))


def test_records_are_archived_before_setup_yaml_drops_them(config, monkeypatch):
    """A crash between the two writes leaves a record in both places, never in neither."""
    setup_file.write_section(config, "slack_tests", {"tests": {"t1": slack(1)}})

    def crash(*args, **kwargs):
        raise OSError("power cut")

    monkeypatch.setattr(setup_file, "write_yaml_atomic", crash)
    with pytest.raises(OSError):
        setup_file.write_section(config, "slack_tests", {"tests": {"t1": slack(1), "t2": slack(2)}})
    assert setup_file.read_archive(config, "slack_tests") == {"t1": slack(1)}
    assert setup_file.read_section(config, "slack_tests") == {"tests": {"t1": slack(1)}}
