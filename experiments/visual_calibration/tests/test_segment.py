"""The BiRefNet segmenter on a fake model: the weights as transformers 5 loads them (float16)
run on either device."""
import sys
import types

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from hand_study import segment


class FakeNet(torch.nn.Module):
    """A one-conv stand-in for BiRefNet, returned in float16 like the published checkpoint."""

    def __init__(self):
        super().__init__()
        self.conv = torch.nn.Conv2d(3, 1, 1)

    def forward(self, x):
        return [self.conv(x)]


@pytest.fixture
def fake_transformers(monkeypatch):
    loaded = []

    class Auto:
        @staticmethod
        def from_pretrained(repo, trust_remote_code=False):
            loaded.append((repo, trust_remote_code))
            return FakeNet().half()

    monkeypatch.setitem(sys.modules, "transformers", types.SimpleNamespace(AutoModelForImageSegmentation=Auto))
    return loaded


def test_birefnet_runs_float16_weights_on_a_cpu(fake_transformers, monkeypatch):
    monkeypatch.setattr(torch.backends.mps, "is_available", lambda: False)
    s = segment.BiRefNet(long_side=64)
    assert s.dev == "cpu" and next(s.model.parameters()).dtype == torch.float32
    p = s.prob(np.zeros((48, 64, 3), np.uint8))
    assert p.shape == (48, 64) and p.dtype == np.float32
    assert fake_transformers == [("ZhengPeng7/BiRefNet_dynamic", True)]
