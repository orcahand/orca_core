"""calibration.yaml stores measurements; ratios are derived when it is loaded,
so a calibration made under other joint ranges still maps every joint right."""

import os

import pytest
import yaml

import orca_core
from orca_core import CalibrationResult, MockOrcaHand, OrcaHand, calibration_store
from orca_core.calibration import (
    _derive_joint_to_motor_ratios,
    joint_fingerprints,
    _mismatched_joints,
)
from orca_core.utils import read_yaml
from tests._helpers import copy_model_config

MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2")

# An index_mcp range other than the model's, as a calibration made under it stored.
RANGE_012 = [-25.0, 100.0]


@pytest.fixture
def model_dir(tmp_path):
    def copy(name="orcahand-right"):
        target = tmp_path / name
        target.mkdir()
        copy_model_config(os.path.join(MODELS, name, "config.yaml"), target / "config.yaml")
        return target
    return copy


def write_012_calibration(path, hand_config):
    """A calibration as 0.12 wrote it: limits plus ratios fitted to 0.12 ranges."""
    limits, ratios = {}, {}
    for joint, motor_id in hand_config.joint_to_motor_map.items():
        lo, hi = RANGE_012 if joint == "index_mcp" else hand_config.joint_roms_dict[joint]
        limits[motor_id] = [0.5, 0.5 + 0.02 * (hi - lo)]
        ratios[motor_id] = 0.02
    path.write_text(yaml.safe_dump({
        "calibrated": True, "wrist_calibrated": True,
        "motor_limits": limits, "joint_to_motor_ratios": ratios,
    }))
    return limits


class TestRatiosAreDerivedAtLoad:
    def test_a_calibration_made_under_0_12_ranges_maps_correctly_under_today_s(self, model_dir):
        folder = model_dir()
        calibration = folder / "old_calibration.yaml"
        probe = OrcaHand(config_path=str(folder / "config.yaml"), calibration_path=str(calibration))
        limits = write_012_calibration(calibration, probe.config)

        hand = OrcaHand(config_path=str(folder / "config.yaml"), calibration_path=str(calibration))
        motor_id = hand.config.joint_to_motor_map["index_mcp"]
        lo, hi = hand.config.joint_roms_dict["index_mcp"]
        assert [lo, hi] != RANGE_012, "the test needs today's range to differ from 0.12's"

        # The motor's measured travel spans today's range, not the stored 0.02.
        expected = (limits[motor_id][1] - limits[motor_id][0]) / (hi - lo)
        assert hand.joint_to_motor_ratios_dict[motor_id] == pytest.approx(expected)
        assert hand.joint_to_motor_ratios_dict[motor_id] != pytest.approx(0.02)

        # Both hardstops read as today's range ends.
        hand._wrap_offsets_dict = {m: 0.0 for m in hand.config.motor_ids}
        idx = hand.config.motor_id_to_idx_dict[motor_id]
        for limit, angle in ((limits[motor_id][0], lo), (limits[motor_id][1], hi)):
            motor_pos = [0.0] * len(hand.config.motor_ids)
            motor_pos[idx] = limit
            assert hand._motor_to_joint_pos(motor_pos)["index_mcp"] == pytest.approx(angle)

    def test_a_bare_load_with_the_config_derives_too(self, model_dir):
        folder = model_dir()
        calibration = folder / "old_calibration.yaml"
        config = OrcaHand(config_path=str(folder / "config.yaml")).config
        limits = write_012_calibration(calibration, config)
        motor_id = config.joint_to_motor_map["index_mcp"]
        lo, hi = config.joint_roms_dict["index_mcp"]
        result = CalibrationResult.from_calibration_path(str(calibration), config.motor_ids, config=config)
        assert result.joint_to_motor_ratios_dict[motor_id] == pytest.approx(
            (limits[motor_id][1] - limits[motor_id][0]) / (hi - lo))

    def test_ratios_follow_a_range_change_on_a_live_hand(self, model_dir):
        hand = MockOrcaHand(config_path=str(model_dir() / "config.yaml"))
        motor_id = hand.config.joint_to_motor_map["index_mcp"]
        before = hand.joint_to_motor_ratios_dict[motor_id]
        lo, hi = hand.config.joint_roms_dict["index_mcp"]
        roms = dict(hand.config.joint_roms_dict, index_mcp=[lo, lo + (hi - lo) / 2])
        import dataclasses
        hand.config = dataclasses.replace(hand.config, joint_roms_dict=roms)
        assert hand.joint_to_motor_ratios_dict[motor_id] == pytest.approx(2 * before)

    def test_no_ratio_without_both_limits_or_a_range(self):
        ratios = _derive_joint_to_motor_ratios(
            {1: [0.0, 1.0], 2: [None, None], 3: [0.0, 1.0], 4: [0.0, 1.0]},
            {"a": 1, "b": 2, "c": 3},
            {"a": [0, 10], "b": [0, 10], "c": [5, 5]},
        )
        assert ratios == {1: 0.1, 2: 0.0, 3: 0.0, 4: 0.0}

    def test_ratios_are_never_written(self, model_dir):
        hand = MockOrcaHand(config_path=str(model_dir() / "config.yaml"))
        hand.connect()
        hand.calibrate(persist=True)
        doc = read_yaml(hand.config.calibration_path)
        assert "joint_to_motor_ratios" not in doc
        assert doc["schema"] == 1


class TestFingerprint:
    def test_each_joint_records_its_signed_motor_and_directions(self, model_dir):
        config = MockOrcaHand(config_path=str(model_dir() / "config.yaml")).config
        prints = joint_fingerprints(config)
        assert prints["index_mcp"] == "+3 extend,flex"
        assert prints["ring_mcp"].startswith("-7 ")

    def test_a_file_without_one_is_trusted(self):
        assert _mismatched_joints(None, {"a": "+1 flex"}) == []

    def test_a_changed_mapping_drops_exactly_the_affected_joints(self, model_dir):
        folder = model_dir()
        hand = MockOrcaHand(config_path=str(folder / "config.yaml"))
        hand.connect()
        hand.calibrate(persist=True)
        path = hand.config.calibration_path

        # Swap index_mcp's and index_pip's motors in the config.
        config = read_yaml(str(folder / "config.yaml"))
        jmap = config["joint_to_motor_map"]
        jmap["index_mcp"], jmap["index_pip"] = jmap["index_pip"], jmap["index_mcp"]
        (folder / "config.yaml").write_text(yaml.safe_dump(config, sort_keys=False))

        again = OrcaHand(config_path=str(folder / "config.yaml"), calibration_path=path)
        uncalibrated = sorted(
            joint for joint, m in again.config.joint_to_motor_map.items()
            if not again.calibration._motor_ready(m))
        assert uncalibrated == ["index_mcp", "index_pip"]
        assert not again.calibrated
        # The file itself is left as measured.
        assert read_yaml(path)["motor_limits"] == hand.calibration.motor_limits_dict

    def test_a_hardware_version_settled_after_load_brings_its_joints_back(self, model_dir):
        """A v2.1 left hand's thumbs are calibrated under the v2.1 map; read
        before the version is known they are dropped, and come back once it is."""
        folder = model_dir("orcahand-left")
        hand = MockOrcaHand(config_path=str(folder / "config.yaml"))
        hand.config = hand.config.with_hardware_version(210)
        hand.connect()
        hand.calibrate(persist=True)
        path = hand.config.calibration_path

        fresh = OrcaHand(config_path=str(folder / "config.yaml"), calibration_path=path)
        changed = sorted(j for j, p in joint_fingerprints(fresh.config).items()
                         if p != joint_fingerprints(hand.config)[j])
        assert changed, "the v2.1 map must differ from the base map for this test"
        for joint in changed:
            assert not fresh.calibration._motor_ready(fresh.config.joint_to_motor_map[joint])

        fresh.config = fresh.config.with_hardware_version(210)
        assert fresh.calibrated
        assert all(fresh.calibration._motor_ready(m) for m in fresh.config.motor_ids)

    def test_calibrating_other_joints_keeps_the_dropped_ones_as_measured(self, model_dir):
        hand = MockOrcaHand(config_path=str(model_dir("orcahand-left") / "config.yaml"))
        hand.set_hardware_version(210)
        hand.connect()
        hand.calibrate(persist=True)
        measured = joint_fingerprints(hand.config)
        thumbs = {j: hand.config.joint_to_motor_map[j] for j in ("thumb_mcp", "thumb_dip")}
        limits = {j: hand.motor_limits_dict[m] for j, m in thumbs.items()}

        hand.set_hardware_version(200)
        hand.calibrate(persist=True, joints=["index_mcp"])
        assert all(hand.motor_limits_dict[m] == [None, None] for m in thumbs.values())
        doc = read_yaml(hand.config.calibration_path)
        for joint, motor_id in thumbs.items():
            assert doc["motor_limits"][motor_id] == limits[joint]
            assert doc["fingerprint"][joint] == measured[joint]

        hand.set_hardware_version(None)
        hand.set_hardware_version(210)
        assert hand.calibrated
        assert {j: hand.motor_limits_dict[m] for j, m in thumbs.items()} == limits


class TestOneCalibrationOfUndo:
    def calibrated_hand(self, folder):
        hand = MockOrcaHand(config_path=str(folder / "config.yaml"))
        hand.connect()
        return hand

    def test_a_new_calibration_keeps_the_one_it_replaces(self, model_dir):
        hand = self.calibrated_hand(model_dir())
        hand.calibrate(persist=True)
        path = hand.config.calibration_path
        first = read_yaml(path)
        assert first["sequence"] == 1 and first["calibrated_at"]
        assert not os.path.exists(calibration_store.previous_path(path))

        hand.calibrate(persist=True, joints=["index_mcp"])
        second = read_yaml(path)
        assert second["sequence"] == 2
        # One run commits after every step but replaces the previous only once.
        assert read_yaml(calibration_store.previous_path(path)) == first
        assert hand.calibration_info["previous"] == {
            "sequence": 1, "calibrated_at": first["calibrated_at"]}

    def test_restoring_brings_the_previous_back_and_can_be_undone(self, model_dir):
        hand = self.calibrated_hand(model_dir())
        hand.calibrate(persist=True)
        path = hand.config.calibration_path
        doc = read_yaml(path)
        doc["motor_limits"][3] = [0.1, 0.9]
        doc["sequence"] = 1
        from orca_core.hand_store import write_doc
        write_doc(path, doc)
        hand.calibrate(persist=True)
        assert hand.motor_limits_dict[3] != [0.1, 0.9]

        info = hand.restore_previous_calibration()
        assert info["sequence"] == 1
        assert hand.motor_limits_dict[3] == [0.1, 0.9]
        assert info["previous"]["sequence"] == 2

        hand.restore_previous_calibration()
        assert hand.calibration_info["sequence"] == 2
        assert hand.motor_limits_dict[3] != [0.1, 0.9]

    def test_a_calibration_put_in_force_while_connected_rechecks_wrap_offsets(self, model_dir):
        """Offsets judged against one calibration's limits can be a full turn
        wrong against another's."""
        import math

        hand = self.calibrated_hand(model_dir())
        hand.calibrate(persist=True)
        hand.calibrate(persist=True, joints=["index_mcp"])
        hand._wrap_offsets_dict = {m: 2 * math.pi for m in hand.config.motor_ids}
        hand.restore_previous_calibration()
        hand.get_joint_position()
        assert set(hand._wrap_offsets_dict.values()) == {0.0}

    def test_a_config_change_that_rederives_the_calibration_rechecks_wrap_offsets(
            self, model_dir):
        import math

        hand = self.calibrated_hand(model_dir("orcahand-left"))
        hand.calibrate(persist=True)
        hand._wrap_offsets_dict = {m: 2 * math.pi for m in hand.config.motor_ids}
        hand.set_hardware_version(210)
        hand.get_joint_position()
        assert set(hand._wrap_offsets_dict.values()) == {0.0}

    def test_nothing_in_force_changes_under_a_running_task_or_joint_loop(self, model_dir):
        import threading

        from orca_core import OrcaHandFull

        hand = self.calibrated_hand(model_dir())
        hand.calibrate(persist=True)
        hand.calibrate(persist=True, joints=["index_mcp"])
        before = dict(hand.motor_limits_dict)
        release = threading.Event()
        hand._task_thread = threading.Thread(target=release.wait)
        hand._task_thread.start()
        try:
            for change in (hand.restore_previous_calibration, hand.adopt_board_calibration,
                           lambda: hand.set_hardware_version(210)):
                with pytest.raises(RuntimeError, match="task"):
                    change()
        finally:
            release.set()
            hand._task_thread.join()
        assert hand.motor_limits_dict == before and hand.hardware_version is None

        looping = OrcaHandFull.__new__(OrcaHandFull)
        looping._loop, looping._task_thread = object(), None
        with pytest.raises(RuntimeError, match="joint-feedback loop"):
            looping.set_hardware_version(210)

    def test_with_nothing_to_restore_it_says_so(self, model_dir):
        hand = self.calibrated_hand(model_dir())
        hand.calibrate(persist=True)
        with pytest.raises(FileNotFoundError):
            hand.restore_previous_calibration()

    def test_other_routines_keys_survive_a_calibration(self, model_dir):
        hand = self.calibrated_hand(model_dir())
        hand.calibrate(persist=True)
        path = hand.config.calibration_path
        doc = read_yaml(path)
        doc["sensor_offsets"] = {"index": [[1.0, 2.0, 3.0]]}
        from orca_core.hand_store import write_doc
        write_doc(path, doc)
        hand.calibrate(persist=True)
        assert read_yaml(path)["sensor_offsets"] == {"index": [[1.0, 2.0, 3.0]]}

    def test_the_run_says_which_file_it_wrote(self, model_dir):
        hand = self.calibrated_hand(model_dir())
        events = []
        hand.calibrate(persist=True, progress_callback=events.append)
        files = [e for e in events if e["event"] == "calibration_file"]
        done = [e for e in events if e["event"] == "calibration_done"]
        assert len(files) == 1
        assert files[0]["path"] == hand.config.calibration_path
        assert done[0]["calibration_path"] == hand.config.calibration_path

    def test_an_unpersisted_run_names_no_file(self, model_dir):
        hand = self.calibrated_hand(model_dir())
        events = []
        hand.calibrate(progress_callback=events.append)
        assert not any(e["event"] == "calibration_file" for e in events)
        assert [e for e in events if e["event"] == "calibration_done"][0]["calibration_path"] is None


class TestASwapCutShort:
    """restore_previous swaps two files with three renames; a crash between
    any two leaves a state the next read puts right, never two copies of one."""

    @pytest.fixture
    def pair(self, tmp_path):
        from orca_core.hand_store import write_doc

        path = tmp_path / "calibration.yaml"
        write_doc(path, {"sequence": 2})
        write_doc(calibration_store.previous_path(str(path)), {"sequence": 1})
        return str(path)

    def stamps(self, path):
        current = calibration_store.read_document(path)["sequence"]
        previous = calibration_store.read_document(calibration_store.previous_path(path))
        return current, previous.get("sequence")

    @pytest.mark.parametrize("renames_done, expected", [(1, (2, 1)), (2, (1, 2))])
    def test_the_next_read_finds_a_whole_state(self, pair, monkeypatch, renames_done, expected):
        real, calls = os.replace, []

        def crash_after(src, dst):
            if len(calls) == renames_done:
                raise KeyboardInterrupt("power cut")
            calls.append(src)
            real(src, dst)

        monkeypatch.setattr(calibration_store.os, "replace", crash_after)
        with pytest.raises(KeyboardInterrupt):
            calibration_store.restore_previous(pair)
        monkeypatch.setattr(calibration_store.os, "replace", real)
        assert os.path.exists(calibration_store.swap_path(pair))
        assert self.stamps(pair) == expected
        assert not os.path.exists(calibration_store.swap_path(pair))

    def test_an_uninterrupted_swap_leaves_nothing_behind(self, pair):
        assert calibration_store.restore_previous(pair)["sequence"] == 1
        assert self.stamps(pair) == (1, 2)
        assert sorted(os.listdir(os.path.dirname(pair))) == [
            "calibration.previous.yaml", "calibration.yaml"]
