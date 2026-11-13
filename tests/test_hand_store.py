"""The per-hand folder: where it is, what its files say, and that writes are safe."""

import os
from pathlib import Path

import pytest
import yaml

from orca_core import hand_store


class TestWhereHandsLive:
    def test_orca_home_follows_the_environment(self, tmp_path, monkeypatch):
        monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path / "elsewhere"))
        assert hand_store.orca_home() == tmp_path / "elsewhere"
        assert hand_store.hand_dir("ser-0000") == tmp_path / "elsewhere" / "hands" / "ser-0000"

    def test_without_the_variable_it_is_dot_orca_in_home(self, monkeypatch):
        monkeypatch.delenv(hand_store.ORCA_HOME_ENV)
        assert hand_store.orca_home() == Path.home() / ".orca"

    def test_tests_never_see_the_real_home(self):
        assert hand_store.orca_home() != Path.home() / ".orca"

    @pytest.mark.parametrize("bad", ["", "..", ".", "a/b", "../x", "x" * 65, "---", "a b"])
    def test_an_id_that_is_not_one_plain_folder_name_is_refused(self, bad):
        with pytest.raises(hand_store.HandStoreError):
            hand_store.hand_dir(bad)

    def test_a_board_names_its_hand_by_serial_then_board_id(self):
        assert hand_store.resolve_hand_id("ser-0000", "0123456789ABCDEF") == "ser-0000"
        assert hand_store.resolve_hand_id(None, "0123456789ABCDEF") == "0123456789ABCDEF"
        assert hand_store.resolve_hand_id("", "") is None
        # A serial that cannot name a folder falls through to the board id.
        assert hand_store.resolve_hand_id("bad/serial", "0123456789ABCDEF") == "0123456789ABCDEF"

    def test_hands_without_identity_share_one_folder_per_model(self):
        assert hand_store.no_board_hand_id("orcahand-left") == "noboard-orcahand-left"
        assert hand_store.valid_hand_id(hand_store.no_board_hand_id("orcahand-touch-right"))

    def test_mock_hands_live_outside_orca_home(self):
        folder = hand_store._mock_hand_dir(hand_store._mock_hand_id("orcahand-right"))
        assert hand_store.orca_home() not in folder.parents


class TestDocuments:
    def test_every_document_carries_the_schema_first(self, tmp_path):
        path = hand_store.write_doc(tmp_path / "x" / "doc.yaml", {"a": 1, "schema": 0})
        text = path.read_text()
        assert text.startswith("schema: 1\n")
        assert yaml.safe_load(text) == {"schema": 1, "a": 1}

    def test_a_newer_hand_yaml_keeps_its_schema_and_keys_when_written(self, tmp_path):
        (tmp_path / "hand.yaml").write_text("schema: 7\nfuture: yes\n")
        hand_store.set_nickname(tmp_path, "bench")
        assert yaml.safe_load((tmp_path / "hand.yaml").read_text()) == {
            "schema": 7, "future": True, "nickname": "bench"}

    def test_an_absent_document_reads_empty(self, tmp_path):
        assert hand_store.read_doc(tmp_path / "missing.yaml", "hand") == {}

    def test_a_newer_schema_is_read_as_is_and_said(self, tmp_path, caplog):
        (tmp_path / "new.yaml").write_text("schema: 7\nfuture: yes\n")
        doc = hand_store.read_doc(tmp_path / "new.yaml", "hand")
        assert doc["future"] is True
        assert "newer orca_core" in caplog.text

    def test_a_write_that_fails_leaves_the_old_file_whole(self, tmp_path, monkeypatch):
        path = hand_store.write_doc(tmp_path / "doc.yaml", {"a": 1})

        def boom(*args, **kwargs):
            raise OSError("disk full")

        monkeypatch.setattr(os, "replace", boom)
        with pytest.raises(OSError):
            hand_store.write_doc(path, {"a": 2})
        assert yaml.safe_load(path.read_text()) == {"schema": 1, "a": 1}
        assert [p.name for p in tmp_path.iterdir()] == ["doc.yaml"]


class TestHandRecord:
    def test_first_contact_writes_identity_and_both_stamps(self, tmp_path):
        doc = hand_store.record_contact(
            tmp_path, "ser-0000", model="orcahand-right", serial="ser-0000",
            board_id="0123456789ABCDEF")
        on_disk = hand_store.read_hand(tmp_path)
        assert on_disk["id"] == "ser-0000"
        assert on_disk["model"] == "orcahand-right"
        assert on_disk["aliases"] == {"serial": ["ser-0000"], "board_id": ["0123456789ABCDEF"]}
        assert on_disk["first_seen"] == on_disk["last_seen"] == doc["first_seen"]
        assert on_disk["nickname"] is None

    def test_a_later_contact_keeps_first_seen_and_every_alias(self, tmp_path, monkeypatch):
        hand_store.record_contact(tmp_path, "ser-0000", serial="ser-0000", board_id="AAAA")
        first = hand_store.read_hand(tmp_path)["first_seen"]
        monkeypatch.setattr(hand_store, "now_iso", lambda: "2030-01-01T00:00:00Z")
        hand_store.record_contact(tmp_path, "ser-0000", serial="ser-0000", board_id="BBBB")
        doc = hand_store.read_hand(tmp_path)
        assert doc["first_seen"] == first
        assert doc["last_seen"] == "2030-01-01T00:00:00Z"
        assert doc["aliases"]["board_id"] == ["AAAA", "BBBB"]

    def test_a_contact_keeps_the_nickname(self, tmp_path):
        hand_store.record_contact(tmp_path, "ser-0000")
        hand_store.set_nickname(tmp_path, "  bench left ")
        hand_store.record_contact(tmp_path, "ser-0000")
        assert hand_store.read_hand(tmp_path)["nickname"] == "bench left"

    def test_a_blank_nickname_clears_it(self, tmp_path):
        hand_store.set_nickname(tmp_path, "x")
        hand_store.set_nickname(tmp_path, "   ")
        assert hand_store.read_hand(tmp_path)["nickname"] is None

    def test_an_unprintable_nickname_is_refused(self, tmp_path):
        with pytest.raises(hand_store.HandStoreError):
            hand_store.set_nickname(tmp_path, "a\nb")

    def test_known_hands_lists_every_folder(self):
        for hand_id in ("ser-0001", "ser-0002"):
            folder, _ = hand_store.open_hand_folder(hand_id)
            hand_store.record_contact(folder, hand_id)
        assert [h["id"] for h in hand_store.known_hands()] == ["ser-0001", "ser-0002"]


class TestOpeningAFolder:
    def test_the_first_open_creates_it_and_says_so(self):
        folder, created = hand_store.open_hand_folder("ser-0000")
        assert folder.is_dir() and created
        assert hand_store.open_hand_folder("ser-0000") == (folder, False)

    def test_a_board_id_folder_becomes_the_serial_s_once_one_is_reported(self):
        old, _ = hand_store.open_hand_folder("0123456789ABCDEF")
        (old / "calibration.yaml").write_text("schema: 1\nsequence: 3\n")
        folder, created = hand_store.open_hand_folder(
            "ser-0000", serial="ser-0000", board_id="0123456789ABCDEF")
        assert not created
        assert folder.name == "ser-0000"
        assert (folder / "calibration.yaml").read_text().endswith("sequence: 3\n")
        assert not old.exists()

    def test_with_both_folders_present_the_serial_s_is_used_and_the_other_kept(self):
        old, _ = hand_store.open_hand_folder("0123456789ABCDEF")
        new, _ = hand_store.open_hand_folder("ser-0000")
        folder, _ = hand_store.open_hand_folder(
            "ser-0000", serial="ser-0000", board_id="0123456789ABCDEF")
        assert folder == new
        assert old.is_dir()


class TestLegacyFiles:
    def test_old_per_hand_files_beside_a_config_are_found(self, tmp_path):
        (tmp_path / "config.yaml").write_text("x: 1\n")
        for name in ("calibration.yaml", "local_config.yaml", "tension_sweeps.yaml"):
            (tmp_path / name).write_text("{}\n")
        found = hand_store.find_legacy_files(tmp_path)
        assert sorted(Path(p).name for p in found) == [
            "calibration.yaml", "local_config.yaml", "tension_sweeps.yaml"]

    def test_none_found_where_there_are_none(self, tmp_path):
        assert hand_store.find_legacy_files(tmp_path) == []
        assert hand_store.find_legacy_files(None) == []


class TestArchive:
    def test_a_record_is_one_file_and_reads_back(self, tmp_path):
        path = hand_store.archive_record(tmp_path, "slack_tests", "2026-11-13T10:00:00+00:00",
                                         {"a": 1})
        assert path.parent == tmp_path / "archive" / "slack_tests"
        assert hand_store.read_archive(tmp_path, "slack_tests") == {
            "2026-11-13T10:00:00+00:00": {"a": 1}}
        assert hand_store.read_archive(tmp_path, "slack_tests", "2026-11-13T10:00:00+00:00") == {"a": 1}
        assert hand_store.read_archive(tmp_path, "slack_tests", "nope") is None

    def test_archiving_the_same_record_twice_keeps_one_file(self, tmp_path):
        hand_store.archive_record(tmp_path, "s", "n", {"a": 1})
        hand_store.archive_record(tmp_path, "s", "n", {"a": 1})
        assert len(list((tmp_path / "archive" / "s").iterdir())) == 1

    def test_a_different_record_under_a_taken_name_never_overwrites(self, tmp_path):
        first = hand_store.archive_record(tmp_path, "s", "n", {"a": 1})
        second = hand_store.archive_record(tmp_path, "s", "n", {"a": 2})
        assert first != second
        assert hand_store.read_doc(first, "archive")["record"] == {"a": 1}
        assert hand_store.read_archive(tmp_path, "s", "n") == {"a": 2}

    def test_an_empty_archive_reads_empty(self, tmp_path):
        assert hand_store.read_archive(tmp_path, "nothing") == {}
