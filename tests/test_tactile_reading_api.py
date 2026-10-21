"""Frame numbers, resultant saturation and fixed-order arrays on tactile readings."""
import json
import threading
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import pytest
from hypothesis import given, settings, strategies as st

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
from orca_core.hardware.sensing.tactile_mock import (
    TactileMockState,
    feed_combined_frame,
    feed_resultant_frame,
    install_tactile_mock,
)
from orca_core.hardware.sensing.tactile_protocol import build_auto_frame
from orca_core.hardware.sensing.types import NUM_TAXELS, TAXEL_SLICES, ResultantReading
from orca_core.hardware.tactile_client import TactileClient
from tests._helpers import wait_until

MODES = [(True, False), (False, True), (True, True)]
FIXTURES = Path(__file__).parent / "fixtures"


@contextmanager
def _streaming_client(fingers, resultant=True, taxels=False):
    state = TactileMockState(connected_fingers=list(fingers))
    link = MockHandSerialLink()
    install_tactile_mock(link, state)
    link.connect()
    client = TactileClient(link, finger_to_sensor_id=state.finger_to_sensor_id)
    client.connect()
    client.start_stream(resultant=resultant, taxels=taxels)
    try:
        yield link, client, state
    finally:
        client.disconnect()
        link.disconnect()


def _reference_decode(payload: bytes, fingers, resultant, taxels):
    """Decode an AA 56 payload with numpy views, independently of tactile_protocol."""
    buf = np.frombuffer(payload, np.uint8)
    forces, taxel_forces, saturated, pos = {}, {}, set(), 0
    for finger in fingers:
        if resultant:
            lo = buf[pos:pos + 6:2]
            forces[finger] = np.array([lo[:2].view(np.int8)[0], lo[:2].view(np.int8)[1], lo[2]]) * 0.1
            if lo[2] == 255:
                saturated.add(finger)
            pos += 6
        if taxels:
            n = DEFAULT_TAXEL_COUNTS[finger]
            t = buf[pos:pos + 3 * n].reshape(n, 3)
            taxel_forces[finger] = np.column_stack(
                [t[:, 0].view(np.int8), t[:, 1].view(np.int8), t[:, 2]]
            ) * 0.1
            pos += 3 * n
    assert pos == len(buf)
    return forces, taxel_forces, saturated


def _payload_size(fingers, resultant, taxels):
    return sum((6 if resultant else 0) + (3 * DEFAULT_TAXEL_COUNTS[f] if taxels else 0) for f in fingers)


@settings(max_examples=60, deadline=None)
@given(
    fingers=st.lists(st.sampled_from(FINGER_NAMES), min_size=1, unique=True).map(
        lambda fs: [f for f in FINGER_NAMES if f in fs]),
    mode=st.sampled_from(MODES),
    data=st.data(),
)
def test_decoded_reading_matches_reference_decoder(fingers, mode, data):
    resultant, taxels = mode
    payload = data.draw(st.binary(
        min_size=_payload_size(fingers, *mode), max_size=_payload_size(fingers, *mode)))
    ref_forces, ref_taxels, ref_saturated = _reference_decode(payload, fingers, *mode)

    with _streaming_client(fingers, resultant, taxels) as (_, client, _):
        client._on_tactile_frame(build_auto_frame(payload))
        reading = client.get_latest()

    assert reading.seq == 1
    if resultant:
        for row, finger in enumerate(FINGER_NAMES):
            if finger in ref_forces:
                np.testing.assert_allclose(reading.forces.array[row], ref_forces[finger], atol=1e-9)
                np.testing.assert_allclose(reading.forces[finger], ref_forces[finger], atol=1e-9)
            else:
                assert np.isnan(reading.forces.array[row]).all()
        assert reading.forces.saturated == ref_saturated
    if taxels:
        for finger, rows in TAXEL_SLICES.items():
            if finger in ref_taxels:
                np.testing.assert_allclose(reading.taxels.array[rows], ref_taxels[finger], atol=1e-9)
            else:
                assert np.isnan(reading.taxels.array[rows]).all()


def test_seq_counts_received_frames_and_marks_drops(tactile_mock):
    link, client, state = tactile_mock
    forces = {f: [0.0, 0.0, 1.0] for f in FINGER_NAMES}
    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, forces, FINGER_NAMES)
    wait_until(lambda: client.get_latest() is not None)
    first = client.get_latest()

    assert first.seq == 1
    assert client.get_latest() is first  # polling again returns the same frame
    assert client.get_latest_forces() is first.forces

    link.feed_bytes(build_auto_frame(b"\x00" * 7))  # wrong size: dropped
    feed_resultant_frame(link, forces, FINGER_NAMES)
    wait_until(lambda: client.get_latest().seq == 3)
    assert client.get_stats().frames_ok == 2

    client.start_stream(resultant=True, taxels=False)
    feed_resultant_frame(link, forces, FINGER_NAMES)
    wait_until(lambda: client.get_latest() is not None)
    assert client.get_latest().seq == 1


def test_saturation_is_judged_before_zeroing(tactile_mock):
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=False)
    client.set_resultant_offsets({f: [0.0, 0.0, 0.3] for f in FINGER_NAMES})
    forces = {f: [0.0, 0.0, 3.0] for f in FINGER_NAMES}
    forces["index"] = [0.0, 0.0, 25.5]
    forces["ring"] = [0.0, 0.0, 25.4]
    feed_resultant_frame(link, forces, FINGER_NAMES)
    wait_until(lambda: client.get_latest() is not None)

    reading = client.get_latest_forces()
    assert reading["index"][2] == 25.2
    assert reading.saturated == {"index"}


def test_arrays_are_fixed_order_read_only_and_cached(tactile_mock_factory):
    link, client, state = tactile_mock_factory(["index", "pinky"])
    client.start_stream(resultant=True, taxels=True)
    forces = {"index": [1.0, -1.0, 2.0], "pinky": [0.5, 0.0, 4.0]}
    taxels = {f: [[0.0, 0.0, 0.2]] * DEFAULT_TAXEL_COUNTS[f] for f in forces}
    feed_combined_frame(link, forces, taxels, state.active_sensors)
    wait_until(lambda: client.get_latest() is not None)
    reading = client.get_latest()

    resultant = reading.forces.array
    assert resultant.shape == (5, 3)
    np.testing.assert_array_equal(resultant[1], [1.0, -1.0, 2.0])
    np.testing.assert_array_equal(resultant[4], [0.5, 0.0, 4.0])
    assert np.isnan(resultant[[0, 2, 3]]).all()

    taxel_array = reading.taxels.array
    assert taxel_array.shape == (NUM_TAXELS, 3) == (363, 3)
    assert (taxel_array[TAXEL_SLICES["index"], 2] == 0.2).all()
    assert np.isnan(taxel_array[TAXEL_SLICES["thumb"]]).all()

    assert reading.forces.array is resultant
    with pytest.raises(ValueError):
        resultant[0, 0] = 1.0
    with pytest.raises(ValueError):
        taxel_array[0, 0] = 1.0


def test_concurrent_readers_see_whole_frames_in_order(tactile_mock):
    """Readers racing the demuxer never see a torn frame or a frame number going back."""
    link, client, state = tactile_mock
    client.start_stream(resultant=True, taxels=True)
    frames = 300
    stop = threading.Event()
    errors = []

    def reader():
        last = 0
        try:
            while not stop.is_set():
                reading = client.get_latest()
                if reading is None:
                    continue
                assert reading.seq >= last
                last = reading.seq
                assert reading.forces.seq == reading.taxels.seq == reading.seq
                # Each frame carries its own number in every value.
                marker = reading.forces.array[0, 2]
                assert (reading.forces.array[:, 2] == marker).all()
                assert (reading.taxels.array[:, 2] == marker).all()
                assert marker == round(reading.seq % 200 * 0.1, 1)
        except Exception as e:  # noqa: BLE001 - surfaced by the main thread
            errors.append(e)

    readers = [threading.Thread(target=reader, daemon=True) for _ in range(4)]
    for t in readers:
        t.start()
    try:
        for i in range(1, frames + 1):
            fz = round(i % 200 * 0.1, 1)
            forces = {f: [0.0, 0.0, fz] for f in FINGER_NAMES}
            taxels = {f: [[0.0, 0.0, fz]] * DEFAULT_TAXEL_COUNTS[f] for f in FINGER_NAMES}
            feed_combined_frame(link, forces, taxels, FINGER_NAMES)
        wait_until(lambda: client.get_stats().frames_ok == frames, timeout=5.0)
    finally:
        stop.set()
        for t in readers:
            t.join(timeout=2.0)

    assert errors == []
    assert client.get_latest().seq == frames


def test_alphabetical_as_array_is_deprecated():
    reading = ResultantReading({"thumb": [0.0, 0.0, 1.0], "index": [0.0, 0.0, 2.0]})
    with pytest.warns(DeprecationWarning, match=r"\.array"):
        rows = reading.as_array()
    np.testing.assert_array_equal(rows[:, 2], [2.0, 1.0])


def _fixture_chunks(name):
    raw = (FIXTURES / name).read_bytes()
    chunks, pos = [], 0
    while pos < len(raw):
        n = int.from_bytes(raw[pos:pos + 2], "little")
        chunks.append(raw[pos + 2:pos + 2 + n])
        pos += 2 + n
    return chunks


def _scan_frames(stream: bytes):
    """Find complete, checksum-valid AA 56 / AA A9 frames; returns ``[(kind, end, frame)]``."""
    frames, i = [], 0
    while i + 5 <= len(stream):
        if stream[i] == 0xAA and stream[i + 1] in (0x56, 0xA9) and stream[i + 2] == 0:
            end = i + 5 + int.from_bytes(stream[i + 3:i + 5], "little") + 1
            frame = stream[i:end]
            if end <= len(stream) and (-sum(frame[:-1])) & 0xFF == frame[-1]:
                frames.append((frame[1], end, frame))
                i = end
                continue
        i += 1
    return frames


def test_raw_bytes_from_a_touch_hand_replay_through_framer_and_decoder(tactile_mock):
    """Read chunks captured off a real hand's sensing port (combined mode, a press
    on index reaching the 25.5 N ceiling) decode frame for frame like the reference."""
    link, client, state = tactile_mock
    meta = json.loads((FIXTURES / "tactile_touch_chunks.json").read_text())
    assert state.active_sensors == meta["fingers"]
    chunks = _fixture_chunks("tactile_touch_chunks.bin")
    frames = _scan_frames(b"".join(chunks))
    tactile = [(end, f) for kind, end, f in frames if kind == 0x56]
    encoder_count = sum(kind == 0xA9 for kind, _, _ in frames)
    assert len(tactile) >= 25 and encoder_count > 0

    encoder_frames = []
    link.register_frame_handler(0xA9, encoder_frames.append)
    client.start_stream(resultant=True, taxels=True)

    fed, next_frame, saturated_seen = 0, 0, set()
    for chunk in chunks:
        link.feed_bytes(chunk)
        fed += len(chunk)
        while next_frame < len(tactile) and tactile[next_frame][0] <= fed:
            next_frame += 1
            wait_until(lambda: client.get_stats().frames_ok == next_frame)
            reading = client.get_latest()
            frame = tactile[next_frame - 1][1]
            ref_forces, ref_taxels, ref_saturated = _reference_decode(
                frame[6:-1], meta["fingers"], True, True)
            assert reading.seq == next_frame
            for row, finger in enumerate(FINGER_NAMES):
                np.testing.assert_allclose(reading.forces.array[row], ref_forces[finger], atol=1e-9)
                np.testing.assert_allclose(
                    reading.taxels.array[TAXEL_SLICES[finger]], ref_taxels[finger], atol=1e-9)
            assert reading.forces.saturated == ref_saturated
            saturated_seen |= ref_saturated

    stats = client.get_stats()
    assert stats.frames_ok == len(tactile)
    assert stats.frames_bad_payload_size == stats.frames_bad_payload == 0
    assert saturated_seen == {"index"}
    wait_until(lambda: len(encoder_frames) == encoder_count)
