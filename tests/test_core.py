from orca_core import MockOrcaHand, OrcaHand


def test_import_and_instantiation():
    hand = OrcaHand()
    assert isinstance(hand, OrcaHand)


def test_orca_hand_exposes_hardware_methods():
    hand = OrcaHand()
    for method_name in [
        "connect",
        "disconnect",
        "enable_torque",
        "disable_torque",
        "set_control_mode",
        "set_max_current",
        "calibrate",
        "tension",
        "jitter",
    ]:
        assert hasattr(hand, method_name), f"Missing hardware method: {method_name}"


def test_mock_connection():
    mock_hand = MockOrcaHand()
    status = mock_hand.connect()
    assert status[0], "Mock connection failed"
