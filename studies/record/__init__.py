"""Capture: rings, sinks, recorders, and the reader that takes them back."""

from .drain import Drain
from .frame_recorder import FrameRecorder, RecordingJointEncoderClient, record_frames
from .loop_recorder import LoopRecorder
from .metadata import Ritual, RitualNotDeclared, collect_metadata
from .reader import Dataset, Table, join_asof, read_table
from .ring import SampleRing
from .session import RecordingSession
from .sink import CsvRowSink, ListRowSink, RowSink
from .window import JointAngleWindow

__all__ = [
    "CsvRowSink",
    "Dataset",
    "Drain",
    "FrameRecorder",
    "JointAngleWindow",
    "ListRowSink",
    "LoopRecorder",
    "RecordingJointEncoderClient",
    "RecordingSession",
    "Ritual",
    "RitualNotDeclared",
    "RowSink",
    "SampleRing",
    "Table",
    "collect_metadata",
    "join_asof",
    "read_table",
    "record_frames",
]
