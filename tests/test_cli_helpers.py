"""orca_core.utils.cli: joint selection, encoder stream lifecycle, recording helpers."""

import re
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import orca_core.utils.cli as cli
from orca_core.hardware.joint_encoder_client import EncodersNotAvailableError

JOINTS = ["thumb_cmc", "thumb_mcp", "index_mcp", "index_pip", "wrist"]
TIMESTAMP = r"\d{8}_\d{6}"


class TestSelectJoints:
    def test_no_selection_returns_none(self):
        assert cli.select_joints(JOINTS) is None

    def test_an_unknown_joint_is_rejected(self):
        with pytest.raises(ValueError, match=r"Unknown joint\(s\) \['index_dip'\]"):
            cli.select_joints(JOINTS, joints=["index_dip"])


@pytest.fixture
def encoder_stack(monkeypatch):
    stack = Mock()
    monkeypatch.setattr(cli, "HandSerialLink", stack.Link)
    monkeypatch.setattr(cli, "JointEncoderClient", stack.Client)
    monkeypatch.setattr(
        cli, "resolve_sensing_ports",
        lambda **kw: SimpleNamespace(encoder="/dev/fake-encoder"),
    )
    return stack


def _calls(stack):
    return [name for name, *_ in stack.mock_calls]


class TestOpenEncoderStream:
    def test_streams_on_the_resolved_port_then_closes_client_before_link(self, encoder_stack):
        with cli.open_encoder_stream("auto", 921600) as client:
            assert client is encoder_stack.Client.return_value

        encoder_stack.Link.assert_called_once_with("/dev/fake-encoder", baudrate=921600)
        assert _calls(encoder_stack) == [
            "Link", "Link().connect",
            "Client", "Client().connect", "Client().start_stream",
            "Client().stop_stream", "Client().disconnect", "Link().disconnect",
        ]

    @pytest.mark.parametrize(
        "start_error", [EncodersNotAvailableError("no frames"), None],
        ids=["stream_never_starts", "body_raises"],
    )
    def test_an_error_still_closes_client_and_link(self, encoder_stack, start_error):
        encoder_stack.Client.return_value.start_stream.side_effect = start_error

        with pytest.raises((EncodersNotAvailableError, KeyboardInterrupt)):
            with cli.open_encoder_stream("auto", 921600):
                raise KeyboardInterrupt

        assert _calls(encoder_stack)[-2:] == ["Client().disconnect", "Link().disconnect"]

    def test_no_encoder_port_is_an_error_naming_the_override(self, monkeypatch):
        monkeypatch.setattr(
            cli, "resolve_sensing_ports", lambda **kw: SimpleNamespace(encoder=None)
        )

        with pytest.raises(RuntimeError, match="Pass --encoder-port to override"):
            with cli.open_encoder_stream("auto", 921600):
                pass


class TestRecordingHelpers:
    def test_path_carries_the_prefix_kind_and_a_timestamp(self, tmp_path):
        named = cli.build_recording_path(tmp_path, "replay_sequence", "pinch")
        unnamed = cli.build_recording_path(tmp_path, "continuous_angles")

        assert named.parent == unnamed.parent == tmp_path
        assert re.fullmatch(rf"pinch_replay_sequence_{TIMESTAMP}\.yaml", named.name)
        assert re.fullmatch(rf"continuous_angles_{TIMESTAMP}\.yaml", unnamed.name)

    def test_metadata_names_the_hand_the_recording_is_valid_for(self):
        hand = SimpleNamespace(config=SimpleNamespace(joint_ids=["wrist"], type="left"))

        metadata = cli.recording_metadata("continuous", hand, requested_frequency_hz=50.0)

        assert re.fullmatch(TIMESTAMP, metadata.pop("created_at"))
        assert metadata == {
            "type": "continuous",
            "joint_ids": ["wrist"],
            "hand_type": "left",
            "requested_frequency_hz": 50.0,
        }

    def test_a_missing_or_empty_recording_does_not_raise(self, tmp_path, capsys):
        assert cli.load_recording(str(tmp_path / "absent.yaml")) is None
        assert "Replay file not found" in capsys.readouterr().out

        empty = tmp_path / "empty.yaml"
        empty.write_text("")
        assert cli.load_recording(str(empty)) == (empty, {})


class TestRecordingMatchesHand:
    hand = SimpleNamespace(config=SimpleNamespace(joint_ids=["wrist", "index_mcp"], type="right"))

    def test_a_recording_without_metadata_passes(self):
        cli.check_recording_matches_hand({}, self.hand)

    def test_a_different_joint_order_is_refused_even_with_force(self):
        with pytest.raises(ValueError, match="joint order does not match"):
            cli.check_recording_matches_hand(
                {"joint_ids": ["index_mcp", "wrist"]}, self.hand, force=True
            )

    @pytest.mark.parametrize(
        "force_flag, hint", [(None, ""), ("--force", " Pass --force to replay it anyway.")]
    )
    def test_the_other_side_is_refused_naming_the_force_flag_only_if_there_is_one(
        self, force_flag, hint,
    ):
        with pytest.raises(ValueError) as exc:
            cli.check_recording_matches_hand(
                {"hand_type": "left"}, self.hand, force_flag=force_flag
            )
        assert str(exc.value) == (
            "Replay was recorded for hand_type=left, but the connected config is right." + hint
        )
