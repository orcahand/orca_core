import pytest

from studies.record import metadata


@pytest.fixture(autouse=True)
def _short_scheduling_probe(monkeypatch):
    """Shorten the host-scheduling probe.

    It samples for half a second to characterise a bench machine; a test only
    needs it to produce a distribution.
    """
    monkeypatch.setattr(metadata, "SCHEDULING_PROBE_S", 0.01)
