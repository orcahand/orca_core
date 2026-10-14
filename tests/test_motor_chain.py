"""Tests for the motor-chain maintenance operation.

The bus is faked: ``scan_motors`` is replaced by a stateful in-memory chain, so
the ID/baud bookkeeping, resume logic and callback contract are exercised
without hardware. The module under test is family-agnostic, so the family facts
come from the real client classes via ``motor_client_class``.
"""

import contextlib

import pytest

from orca_core.constants import DYNAMIXEL, DYNAMIXEL_RETURN_DELAY_TIME_US, FEETECH
from orca_core.maintenance import motor_chain as mc


DYNA_CFG = {
    "motor_ids": list(range(1, 18)),
    "joint_to_motor_map": {"wrist": 1, "index_mcp": 2},
    "baudrate": 1_000_000,
}


def dyna_plan(**overrides) -> mc.MotorChainPlan:
    cfg = {**DYNA_CFG, **overrides}
    return mc.plan_motor_chain(cfg, port="/dev/fake", motor_type=DYNAMIXEL)


def motor(mid: int, model: str, baud: int) -> dict:
    return {"id": mid, "model_name": model, "baud_rate": baud}


class FakeBus:
    """Motors addressable by (id, baud).

    ``plug`` queues motor models the operator will connect when asked: the bus
    materialises the next one at factory defaults the first time it is scanned
    for a default motor and none is present.
    """

    def __init__(self, motors=(), plan=None):
        self.motors = list(motors)
        self.plan = plan
        self.to_plug: list[str] = []
        self.port_present = True
        self.unopenable: set[int] = set()     # rates this host's driver refuses
        self.mirrored: set[int] = set()       # rates the bridge ignores: the bus stays put

    def plug(self, *models: str) -> None:
        self.to_plug.extend(models)

    def scan(self, motor_type, port, baudrate, id_range):
        if (self.plan is not None and self.to_plug
                and baudrate == self.plan.default_baud
                and id_range == (self.plan.default_id, self.plan.default_id)
                and not self._default_motor()):
            self.motors.append(motor(self.plan.default_id, self.to_plug.pop(0),
                                     self.plan.default_baud))
        lo, hi = id_range
        if baudrate in self.mirrored and self.plan is not None:
            baudrate = self.plan.target_baud
        return [m for m in self.motors if m["baud_rate"] == baudrate and lo <= m["id"] <= hi]

    def _default_motor(self):
        return [m for m in self.motors
                if m["id"] == self.plan.default_id and m["baud_rate"] == self.plan.default_baud]

    def at(self, mid: int, baud: int) -> list[dict]:
        return [m for m in self.motors if m["id"] == mid and m["baud_rate"] == baud]

    def install(self, monkeypatch):
        monkeypatch.setattr(mc, "scan_motors", self.scan)
        monkeypatch.setattr(mc, "serial_port_exists", lambda port: self.port_present)
        monkeypatch.setattr(mc, "_host_can_open", lambda port, baud: baud not in self.unopenable)


# --- family facts come from the client classes ------------------------------


def test_plan_reads_factory_defaults_from_the_motor_client():
    from orca_core.hardware.dynamixel_client import DynamixelClient
    from orca_core.hardware.feetech_client import FeetechClient

    plan = dyna_plan()
    assert plan.default_id == DynamixelClient.factory_default_id
    assert plan.default_baud == DynamixelClient.factory_default_baudrate
    assert plan.requires_unpowered_hotplug is False

    feetech = mc.plan_motor_chain(DYNA_CFG, "/dev/fake", FEETECH)
    assert feetech.default_baud == FeetechClient.factory_default_baudrate
    assert feetech.requires_unpowered_hotplug is True


def test_valid_baudrates_come_from_the_client_baud_map():
    from orca_core.hardware.dynamixel_client import BAUD_RATE_MAP

    assert mc.valid_baudrates(DYNAMIXEL) == sorted(BAUD_RATE_MAP, reverse=True)


def test_unknown_motor_type_is_rejected():
    with pytest.raises(ValueError, match="Unknown motor_type"):
        mc.valid_baudrates("servo9000")


# --- plan -------------------------------------------------------------------


def test_plan_reserves_id_1_for_the_wrist():
    plan = dyna_plan()
    assert plan.wrist_id == 1
    assert 1 not in plan.finger_ids
    assert plan.finger_ids == sorted(range(2, 18), reverse=True)
    assert plan.total_motors == 17


def test_plan_without_wrist_uses_every_motor_id_as_a_finger():
    plan = dyna_plan(joint_to_motor_map={"index_mcp": 2})
    assert plan.wrist_id is None
    assert 1 in plan.finger_ids
    assert plan.total_motors == 17


def test_plan_model_for_distinguishes_wrist_from_fingers():
    plan = dyna_plan()
    assert plan.model_for(1) == "XC430"
    assert plan.model_for(17) == "XC330"


def test_plan_without_finger_motors_raises():
    with pytest.raises(mc.MotorChainError, match="no finger motors"):
        mc.plan_motor_chain({"motor_ids": [1], "joint_to_motor_map": {"wrist": 1}},
                            "/dev/fake", DYNAMIXEL)


# --- v1-style layout: the wrist ID comes from the config map ----------------


V1_CFG = {
    "motor_ids": list(range(1, 18)),
    "joint_to_motor_map": {"wrist": -17, "thumb_pip": -1},
    "baudrate": 3_000_000,
}


def test_plan_takes_the_wrist_id_from_the_config_map():
    plan = mc.plan_motor_chain(V1_CFG, "/dev/fake", DYNAMIXEL)
    assert plan.wrist_id == 17
    assert 17 not in plan.finger_ids
    assert plan.finger_ids == sorted(range(1, 17), reverse=True)
    assert plan.model_for(17) == "XC430"
    assert plan.model_for(1) == "XC330"
    assert plan.all_target_ids == sorted(range(1, 18), reverse=True)


def test_plan_rejects_a_wrist_motor_missing_from_motor_ids():
    cfg = {**DYNA_CFG, "joint_to_motor_map": {"wrist": -19}}
    with pytest.raises(mc.MotorChainError, match="wrist motor 19"):
        mc.plan_motor_chain(cfg, "/dev/fake", DYNAMIXEL)


def test_scan_accepts_a_prefix_led_by_a_highest_id_wrist(monkeypatch):
    plan = mc.plan_motor_chain(V1_CFG, "/dev/fake", DYNAMIXEL)
    FakeBus([motor(17, "XC430", 3_000_000), motor(16, "XC330", 3_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(plan)
    assert scan.valid_ids == [17, 16]
    assert scan.invalid_ids == []


def test_scan_rejects_a_finger_model_in_a_highest_id_wrist_slot(monkeypatch):
    plan = mc.plan_motor_chain(V1_CFG, "/dev/fake", DYNAMIXEL)
    FakeBus([motor(17, "XC330", 3_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(plan)
    assert scan.valid_ids == []
    assert scan.invalid_ids == [17]


def test_configure_chain_programs_a_highest_id_wrist_first(monkeypatch):
    cfg = {"motor_ids": [1, 2, 3], "joint_to_motor_map": {"wrist": -3}, "baudrate": 1_000_000}
    plan = mc.plan_motor_chain(cfg, "/dev/fake", DYNAMIXEL)
    bus = _wire(monkeypatch, plan)
    bus.plug("XC430", "XC330", "XC330")  # wrist attaches to the board, then the fingers
    events = []

    configured = mc.configure_motor_chain(plan, progress_callback=events.append, poll_interval=0)

    assert configured == [3, 2, 1]
    steps = [(e["target_id"], e["expected_model"], e["attaches_to"])
             for e in events if e["event"] == "step_started"]
    assert steps == [(3, "XC430", "board"), (2, "XC330", "motor 3"), (1, "XC330", "motor 2")]


# --- scan_configured_motors -------------------------------------------------


def test_empty_bus_scans_clean(monkeypatch):
    FakeBus().install(monkeypatch)
    scan = mc.scan_configured_motors(dyna_plan())
    assert scan.valid_ids == [] and scan.invalid_ids == []


def test_contiguous_prefix_from_the_top_is_valid(monkeypatch):
    FakeBus([motor(17, "XC330", 1_000_000), motor(16, "XC330", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(dyna_plan())
    assert scan.valid_ids == [17, 16]
    assert scan.invalid_ids == []


def test_a_gap_in_the_prefix_marks_the_rest_invalid(monkeypatch):
    # 17 present, 16 missing, 15 present -> 15 is not a valid continuation.
    FakeBus([motor(17, "XC330", 1_000_000), motor(15, "XC330", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(dyna_plan())
    assert scan.valid_ids == [17]
    assert scan.invalid_ids == [15]


def test_wrong_model_in_a_finger_slot_is_invalid(monkeypatch):
    FakeBus([motor(17, "XC430", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(dyna_plan())
    assert scan.valid_ids == []
    assert scan.invalid_ids == [17]


def test_wrist_counts_only_once_the_fingers_are_complete(monkeypatch):
    fingers = [motor(i, "XC330", 1_000_000) for i in range(17, 1, -1)]
    FakeBus(fingers + [motor(1, "XC430", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(dyna_plan())
    assert scan.valid_ids[-1] == 1
    assert scan.invalid_ids == []


def test_wrist_without_complete_fingers_is_invalid(monkeypatch):
    FakeBus([motor(17, "XC330", 1_000_000), motor(1, "XC430", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(dyna_plan())
    assert scan.valid_ids == [17]
    assert scan.invalid_ids == [1]


def test_feetech_fresh_motor_at_default_id_is_not_mistaken_for_a_wrist(monkeypatch):
    """Feetech target baud == default baud, so a fresh HLS3915 at ID 1 looks like
    a configured wrist until the model name disambiguates it."""
    cfg = {**DYNA_CFG, "baudrate": 1_000_000}
    plan = mc.plan_motor_chain(cfg, "/dev/fake", FEETECH)
    assert plan.target_baud == plan.default_baud
    FakeBus([motor(1, "HLS3915", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(plan)
    assert scan.valid_ids == [] and scan.invalid_ids == []

    FakeBus([motor(1, "HLS3930", 1_000_000)]).install(monkeypatch)
    scan = mc.scan_configured_motors(plan)
    assert scan.invalid_ids == [1]  # real wrist, but fingers aren't done


# --- verify_chain -----------------------------------------------------------


def test_verify_chain_detects_a_dropped_motor(monkeypatch):
    FakeBus([motor(17, "XC330", 1_000_000)]).install(monkeypatch)
    with pytest.raises(mc.MotorChainError, match="verification failed"):
        mc.verify_chain(dyna_plan(), [17, 16])


def test_verify_chain_passes_when_ids_match(monkeypatch):
    FakeBus([motor(17, "XC330", 1_000_000), motor(16, "XC330", 1_000_000)]).install(monkeypatch)
    mc.verify_chain(dyna_plan(), [16, 17])


# --- find_default_motor -----------------------------------------------------


def test_find_default_motor_rejects_the_wrong_model(monkeypatch):
    FakeBus([motor(1, "XC430", 57600)]).install(monkeypatch)
    with pytest.raises(mc.MotorChainError, match="wrong motor type"):
        mc.find_default_motor(dyna_plan(), "XC330")


def test_find_default_motor_returns_none_on_empty_bus(monkeypatch):
    FakeBus().install(monkeypatch)
    assert mc.find_default_motor(dyna_plan(), "XC330") is None


# --- configure_default_motor ------------------------------------------------


class FakeSessionClient:
    """Records ID/baud/return-delay writes issued through a config session and,
    when bound to a :class:`FakeBus`, applies an accepted write to the first
    motor answering at (id, session baud) — as a real motor would.

    ``*_ok`` False rejects the write (the motor answers with an error).
    ``ignore_id_writes`` acknowledges that many ID writes without applying them,
    the way a motor that did not take an EEPROM write behaves.
    """

    def __init__(self, bus=None):
        self.calls = []
        self.bus = bus
        self.session_baud = None
        self.baud_ok = True
        self.id_ok = True
        self.delay_ok = True
        self.ignore_id_writes = 0

    def change_return_delay_time(self, motor_id, delay_us):
        self.calls.append(("delay", motor_id, delay_us))
        return self.delay_ok

    def change_motor_baudrate(self, motor_id, new_baud):
        self.calls.append(("baud", motor_id, new_baud))
        if self.baud_ok and self.bus is not None:
            for m in self.bus.at(motor_id, self.session_baud)[:1]:
                m["baud_rate"] = new_baud
        return self.baud_ok

    def change_motor_id(self, current_id, new_id):
        self.calls.append(("id", current_id, new_id))
        if self.ignore_id_writes > 0:
            self.ignore_id_writes -= 1
            return True
        if self.id_ok and self.bus is not None:
            for m in self.bus.at(current_id, self.session_baud)[:1]:
                m["id"] = new_id
        return self.id_ok


def _wire_sessions(monkeypatch, client, scan=None, plan=None, bus=None):
    """Route _config_session through ``client``, recording each session's baud.

    Without an explicit ``scan`` the bus is a :class:`FakeBus` holding one
    fresh finger motor for ``plan`` (default: the Dynamixel plan), and
    ``client`` writes to it.
    """
    session_bauds = []
    if scan is None:
        if bus is None:
            plan = plan or dyna_plan()
            bus = FakeBus([motor(plan.default_id, plan.finger_model, plan.default_baud)], plan)
        bus.install(monkeypatch)
        client.bus = bus
        scan = bus.scan

    @contextlib.contextmanager
    def fake_session(motor_type, motor_ids, port, baudrate):
        session_bauds.append(baudrate)
        client.session_baud = baudrate
        yield client

    monkeypatch.setattr(mc, "_config_session", fake_session)
    monkeypatch.setattr(mc, "scan_motors", scan)
    monkeypatch.setattr(mc.time, "sleep", lambda _s: None)
    return session_bauds


def test_configure_default_motor_changes_baud_before_id(monkeypatch):
    """The ID write must happen at the target baud: an interruption then leaves
    the motor at (default ID, target baud), which the prescan can still see."""
    plan = dyna_plan()
    client = FakeSessionClient()
    session_bauds = _wire_sessions(monkeypatch, client)

    mc.configure_default_motor(plan, target_id=3)

    assert client.calls == [("baud", plan.default_id, plan.target_baud),
                            ("delay", plan.default_id, DYNAMIXEL_RETURN_DELAY_TIME_US),
                            ("id", plan.default_id, 3)]
    assert session_bauds == [plan.default_baud, plan.target_baud]


def test_a_dynamixel_gets_its_return_delay_before_its_id(monkeypatch):
    """Delay before ID: a motor that reaches its target ID always carries it."""
    client = FakeSessionClient()
    _wire_sessions(monkeypatch, client)

    mc.configure_default_motor(dyna_plan(), target_id=3)

    kinds = [call[0] for call in client.calls]
    assert kinds.index("delay") < kinds.index("id")


def test_a_failed_return_delay_write_never_touches_the_id(monkeypatch):
    client = FakeSessionClient()
    client.delay_ok = False
    _wire_sessions(monkeypatch, client)

    with pytest.raises(mc.MotorChainError, match="return delay"):
        mc.configure_default_motor(dyna_plan(), target_id=3)
    assert not any(call[0] == "id" for call in client.calls)


def test_feetech_return_delay_is_left_at_the_factory_value(monkeypatch):
    plan = mc.plan_motor_chain(DYNA_CFG, "/dev/fake", FEETECH)
    client = FakeSessionClient()
    _wire_sessions(monkeypatch, client, plan=plan)

    mc.configure_default_motor(plan, target_id=3)
    assert not any(call[0] == "delay" for call in client.calls)


def test_a_failed_baud_change_never_touches_the_id(monkeypatch):
    plan = dyna_plan()
    client = FakeSessionClient()
    client.baud_ok = False
    _wire_sessions(monkeypatch, client)
    events = []

    with pytest.raises(mc.MotorChainError, match="did not take the baud rate"):
        mc.configure_default_motor(plan, target_id=3, progress_callback=events.append)
    assert not any(call[0] == "id" for call in client.calls)
    assert [c[0] for c in client.calls] == ["baud"] * mc.WRITE_ATTEMPTS
    assert [e["event"] for e in events] == ["write_retry"] * (mc.WRITE_ATTEMPTS - 1)


def test_matching_bauds_skip_the_baud_write(monkeypatch):
    plan = mc.plan_motor_chain({**DYNA_CFG, "baudrate": 1_000_000}, "/dev/fake", FEETECH)
    client = FakeSessionClient()
    _wire_sessions(monkeypatch, client, plan=plan)

    mc.configure_default_motor(plan, target_id=3)
    assert client.calls == [("id", plan.default_id, 3)]


def test_a_leftover_default_motor_after_programming_is_refused(monkeypatch):
    """Two default motors on the bus accept the same writes; the survivor at the
    default ID is the only detectable symptom, so the step must fail loudly."""
    plan = dyna_plan()
    client = FakeSessionClient()
    # Two fresh motors; writes land on the first, the second keeps answering at the default.
    bus = FakeBus([motor(1, "XC330", plan.default_baud), motor(1, "XC330", plan.default_baud)], plan)
    _wire_sessions(monkeypatch, client, bus=bus)
    events = []

    with pytest.raises(mc.MotorChainError, match="factory-default motor still answers"):
        mc.configure_default_motor(plan, target_id=3, progress_callback=events.append)
    assert [e["event"] for e in events] == ["duplicate_default_motor"]
    assert events[0]["target_id"] == 3


def test_a_wrist_programmed_to_the_default_id_is_not_a_duplicate(monkeypatch):
    plan = dyna_plan()  # wrist_id == 1 == the factory-default ID
    client = FakeSessionClient()
    bus = FakeBus([motor(1, "XC430", plan.default_baud)], plan)
    _wire_sessions(monkeypatch, client, bus=bus)
    events = []

    mc.configure_default_motor(plan, target_id=plan.default_id, progress_callback=events.append)
    assert events == []
    assert bus.motors == [motor(1, "XC430", plan.target_baud)]


def test_prescan_surfaces_a_motor_stranded_at_the_default_id(monkeypatch):
    """A run interrupted between the baud and ID writes leaves the motor at
    (default ID, target baud); the resume prescan must report it, not skip it."""
    plan = dyna_plan()
    FakeBus([motor(1, "XC330", plan.target_baud)]).install(monkeypatch)
    scan = mc.scan_configured_motors(plan)
    assert scan.valid_ids == []
    assert scan.invalid_ids == [1]


# --- exit-time cleanup registry ---------------------------------------------


class StubPortHandler:
    def __init__(self):
        self.closed = False

    def closePort(self):
        self.closed = True


class StubClient:
    """Mimics the client families' registry contract: register in __init__,
    deregister only through disconnect()."""

    OPEN_CLIENTS = set()

    def __init__(self):
        self.port_handler = StubPortHandler()
        self._connected = False
        self.OPEN_CLIENTS.add(self)

    def connect(self):
        self._connected = True

    def scan_for_motors(self, port, id_range, baud_rates=None):
        return []


@pytest.fixture
def stub_clients(monkeypatch):
    StubClient.OPEN_CLIENTS.clear()
    created = []

    def factory(motor_type, motor_ids, port, baudrate):
        client = StubClient()
        created.append(client)
        return client

    monkeypatch.setattr(mc, "create_motor_client", factory)
    return created


def test_config_session_clients_never_reach_the_exit_cleanup(stub_clients):
    """The session bypasses disconnect(), so it must scrub the connected flag
    and the registry itself — a stale entry makes the atexit torque-disable
    handler write to a closed port."""
    with mc._config_session(DYNAMIXEL, [1], "/dev/fake", 57600) as client:
        assert client._connected
    assert client.port_handler.closed
    assert client._connected is False
    assert client not in StubClient.OPEN_CLIENTS


def test_scan_polling_does_not_accumulate_clients(stub_clients):
    for _ in range(5):
        mc.scan_motors(DYNAMIXEL, "/dev/fake", 57600, (1, 1))
    assert len(stub_clients) == 5
    assert StubClient.OPEN_CLIENTS == set()


def test_scan_motors_leaves_the_real_dynamixel_registry_unchanged():
    from orca_core.hardware.dynamixel_client import DynamixelClient

    before = set(DynamixelClient.OPEN_CLIENTS)
    mc.scan_motors(DYNAMIXEL, "/dev/nonexistent-orca-chain-test", 57600, (1, 1))
    assert set(DynamixelClient.OPEN_CLIENTS) == before


# --- configure_motor_chain --------------------------------------------------


def small_plan(motor_type=DYNAMIXEL) -> mc.MotorChainPlan:
    cfg = {"motor_ids": [1, 2, 3], "joint_to_motor_map": {"wrist": 1}, "baudrate": 1_000_000}
    return mc.plan_motor_chain(cfg, "/dev/fake", motor_type)


def _wire(monkeypatch, plan):
    """A bus whose configure step moves the factory-default motor to its target."""
    bus = FakeBus(plan=plan)
    bus.install(monkeypatch)

    def fake_configure(p, target_id, progress_callback=None, current_baud=None):
        fresh = bus.at(p.default_id, p.default_baud if current_baud is None else current_baud)
        assert fresh, "configure called with no default motor on the bus"
        fresh[0].update(id=target_id, baud_rate=p.target_baud)

    monkeypatch.setattr(mc, "configure_default_motor", fake_configure)
    monkeypatch.setattr(mc.time, "sleep", lambda _s: None)
    return bus


@pytest.fixture
def wired_bus(monkeypatch):
    plan = small_plan()
    return plan, _wire(monkeypatch, plan)


def test_configure_chain_programs_every_motor_in_order(wired_bus):
    plan, bus = wired_bus
    bus.plug("XC330", "XC330", "XC430")  # the operator connects 3, then 2, then the wrist
    events = []

    configured = mc.configure_motor_chain(plan, progress_callback=events.append, poll_interval=0)

    assert configured == [3, 2, 1]           # fingers high->low, wrist last
    assert sorted(m["id"] for m in bus.motors) == [1, 2, 3]
    assert all(m["baud_rate"] == plan.target_baud for m in bus.motors)

    names = [e["event"] for e in events]
    assert names[0] == "chain_started" and names[-1] == "chain_done"
    assert names.count("motor_configured") == 3
    assert names.count("chain_verified") == 3

    steps = [e["target_id"] for e in events if e["event"] == "step_started"]
    assert steps == [3, 2, 1]


def test_a_hot_pluggable_family_needs_no_prompt_callback(wired_bus):
    """Dynamixel motors can be plugged onto a live bus, so chain assembly runs
    headless: the operator is told what to do via progress events, not a prompt."""
    plan, bus = wired_bus
    bus.plug("XC330", "XC330", "XC430")
    assert mc.configure_motor_chain(plan, poll_interval=0) == [3, 2, 1]


def test_configure_chain_resumes_and_skips_done_motors(wired_bus):
    plan, bus = wired_bus
    bus.motors.append(motor(3, "XC330", plan.target_baud))  # already configured
    bus.plug("XC330", "XC430")
    events = []

    configured = mc.configure_motor_chain(plan, progress_callback=events.append, poll_interval=0)
    steps = [e["target_id"] for e in events if e["event"] == "step_started"]
    assert steps == [2, 1], "motor 3 should not be asked for again"
    assert configured == [3, 2, 1]


def test_configure_chain_is_a_noop_when_already_complete(wired_bus):
    plan, bus = wired_bus
    bus.motors.extend([motor(3, "XC330", plan.target_baud), motor(2, "XC330", plan.target_baud),
                       motor(1, "XC430", plan.target_baud)])
    events = []
    configured = mc.configure_motor_chain(plan, progress_callback=events.append,
                                          prompt_callback=lambda p: pytest.fail("prompted"))
    assert sorted(configured) == [1, 2, 3]
    assert events[-1]["event"] == "chain_done"


def test_configure_chain_aborts_on_a_misprogrammed_bus(wired_bus):
    plan, bus = wired_bus
    bus.motors.append(motor(2, "XC330", plan.target_baud))  # 3 missing -> 2 is not a valid prefix
    with pytest.raises(mc.MotorChainAborted) as excinfo:
        mc.configure_motor_chain(plan, prompt_callback=lambda p: None)
    assert excinfo.value.invalid_ids == [2]
    assert excinfo.value.invalid_motors == [motor(2, "XC330", plan.target_baud)]
    assert "check its connector" in str(excinfo.value) and "reset" in str(excinfo.value)


def test_a_family_needing_an_unpowered_bus_requires_a_prompt_callback(monkeypatch):
    plan = small_plan(FEETECH)
    FakeBus(plan=plan).install(monkeypatch)
    monkeypatch.setattr(mc, "wait_for_port", lambda *a, **k: None)
    with pytest.raises(mc.MotorChainError, match="no prompt_callback"):
        mc.configure_motor_chain(plan, poll_interval=0)


def test_a_family_needing_an_unpowered_bus_is_power_cycled_per_motor(monkeypatch):
    plan = small_plan(FEETECH)
    bus = _wire(monkeypatch, plan)

    port_states = []
    monkeypatch.setattr(mc, "wait_for_port",
                        lambda port, *, present, **k: port_states.append(present))

    def on_prompt(p):
        bus.plug(plan.model_for(p["target_id"]))

    mc.configure_motor_chain(plan, prompt_callback=on_prompt, poll_interval=0)
    # Each of the 3 motors: unplug (False), operator connects, replug (True).
    assert port_states == [False, True] * 3


def test_should_stop_cancels_the_chain(wired_bus):
    plan, bus = wired_bus
    bus.plug("XC330", "XC330", "XC430")
    with pytest.raises(mc.MotorChainError, match="cancelled"):
        mc.configure_motor_chain(plan, should_stop=lambda: True, poll_interval=0)


def test_a_failing_progress_callback_does_not_abort_the_run(wired_bus):
    plan, bus = wired_bus
    bus.plug("XC330", "XC330", "XC430")

    def boom(_event):
        raise RuntimeError("callback exploded")

    assert mc.configure_motor_chain(plan, progress_callback=boom, poll_interval=0) == [3, 2, 1]


# --- bulk operations --------------------------------------------------------


def test_change_all_baudrates_rejects_an_invalid_rate(monkeypatch):
    FakeBus().install(monkeypatch)
    with pytest.raises(mc.MotorChainError, match="invalid baud rate"):
        mc.change_all_baudrates(dyna_plan(), 12345)


def test_reset_all_motors_reports_per_motor_failures(monkeypatch):
    plan = dyna_plan()
    FakeBus([motor(17, "XC330", 1_000_000), motor(16, "XC330", 1_000_000)]).install(monkeypatch)

    def flaky(p, mid, baud):
        if mid == 16:
            raise mc.MotorChainError("bus error")

    monkeypatch.setattr(mc, "reset_motor_to_factory", flaky)
    events = []
    acted = mc.reset_all_motors(plan, progress_callback=events.append)

    assert [m["id"] for m in acted] == [17]
    failures = [e for e in events if e["event"] == "motor_update_failed"]
    assert len(failures) == 1 and failures[0]["motor"]["id"] == 16


# --- set_all_return_delays --------------------------------------------------


def test_set_all_return_delays_writes_every_motor_at_its_own_baud(monkeypatch):
    plan = dyna_plan()
    client = FakeSessionClient()
    bus = [motor(17, "XC330", 1_000_000), motor(16, "XC330", 1_000_000)]
    session_bauds = _wire_sessions(monkeypatch, client, scan=lambda *a, **k: bus)

    acted = mc.set_all_return_delays(plan)

    assert [m["id"] for m in acted] == [17, 16]
    assert client.calls == [("delay", 17, DYNAMIXEL_RETURN_DELAY_TIME_US),
                            ("delay", 16, DYNAMIXEL_RETURN_DELAY_TIME_US)]
    assert session_bauds == [1_000_000, 1_000_000]


def test_set_all_return_delays_reports_a_rejected_write(monkeypatch):
    client = FakeSessionClient()
    client.delay_ok = False
    _wire_sessions(monkeypatch, client, scan=lambda *a, **k: [motor(17, "XC330", 1_000_000)])
    events = []

    acted = mc.set_all_return_delays(dyna_plan(), progress_callback=events.append)

    assert acted == []
    assert [e["event"] for e in events] == ["motor_update_failed"]


def test_set_all_return_delays_refuses_feetech_before_touching_the_bus(monkeypatch):
    plan = mc.plan_motor_chain(DYNA_CFG, "/dev/fake", FEETECH)
    _wire_sessions(monkeypatch, FakeSessionClient(),
                   scan=lambda *a, **k: pytest.fail("scanned the bus"))
    monkeypatch.setattr(mc, "wait_for_port", lambda *a, **k: pytest.fail("power-cycled the bus"))

    with pytest.raises(mc.MotorChainError, match="no return delay"):
        mc.set_all_return_delays(plan)


# --- disturbances: the run follows the bus, not its own bookkeeping ----------


def _wire_real(monkeypatch, plan, motors=()):
    """The whole routine against a stateful bus with real programming: scans
    read the bus, config-session writes change it. Returns (bus, client)."""
    bus = FakeBus(list(motors), plan)
    client = FakeSessionClient(bus)
    _wire_sessions(monkeypatch, client, bus=bus)
    return bus, client


def _plug_on_request(bus, plan, events, extra=None):
    """A progress callback that behaves like an attentive operator: when the
    routine asks for a motor, a fresh one of the expected model appears."""
    def on_event(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and not bus.at(plan.default_id, plan.default_baud) \
                and not bus.at(plan.default_id, plan.target_baud):
            bus.motors.append(motor(plan.default_id, e["expected_model"], plan.default_baud))
        if extra is not None:
            extra(e)
    return on_event


def test_a_motor_stranded_after_the_baud_write_is_adopted_and_finished(monkeypatch):
    """An interrupted run leaves a motor at (default ID, target baud). The next
    run adopts it as the step's motor and skips straight to the ID write."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan, [motor(1, "XC330", plan.target_baud)])
    events = []

    configured = mc.configure_motor_chain(
        plan, progress_callback=_plug_on_request(bus, plan, events), poll_interval=0)

    assert configured == [3, 2, 1]
    adopted = [e for e in events if e["event"] == "half_programmed_motor_adopted"]
    assert [e["target_id"] for e in adopted] == [3]
    first_writes = client.calls[:2]
    assert first_writes == [("delay", 1, DYNAMIXEL_RETURN_DELAY_TIME_US), ("id", 1, 3)]
    assert sorted(m["id"] for m in bus.motors) == [1, 2, 3]
    assert all(m["baud_rate"] == plan.target_baud for m in bus.motors)


def test_a_stray_motor_plugged_in_mid_run_is_reset_then_programmed(monkeypatch):
    """A motor carrying an ID from another hand is not a dead end: the bus
    survey resets it to factory defaults and the step programs it."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.at(9, plan.target_baud) \
                and not any(x["event"] == "stray_motor_reset" for x in events):
            bus.motors.append(motor(9, "XC330", plan.target_baud))
        if e["event"] == "awaiting_motor" and e["target_id"] in (2, 1):
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    configured = mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0)

    assert configured == [3, 2, 1]
    resets = [e for e in events if e["event"] == "stray_motor_reset"]
    assert [e["motor"]["id"] for e in resets] == [9]
    assert ("baud", 9, plan.default_baud) in client.calls and ("id", 9, 1) in client.calls
    assert sorted(m["id"] for m in bus.motors) == [1, 2, 3]


def test_a_stray_at_a_foreign_baud_is_found_and_reset(monkeypatch):
    plan = small_plan()
    foreign = 2_000_000
    assert foreign not in (plan.default_baud, plan.target_baud)
    bus, client = _wire_real(monkeypatch, plan)
    events = []

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.motors:
            bus.motors.append(motor(2, "XC330", foreign))
        if e["event"] == "awaiting_motor" and e["target_id"] in (2, 1):
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    assert [e["motor"] for e in events if e["event"] == "stray_motor_reset"] == \
        [motor(2, "XC330", foreign)]


def test_a_wrong_model_beyond_the_prefix_is_reset_on_the_first_scan(monkeypatch):
    """A wrist motor sitting in a finger slot cannot be a configured motor
    behind a loose connector; it is reset without aborting."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan,
                             [motor(3, "XC330", plan.target_baud), motor(2, "XC430", plan.target_baud)])
    events = []

    def operator(e):
        events.append(e)
        if e["event"] == "wrong_motor_detected":   # the operator swaps the wrist for a finger
            bus.at(plan.default_id, plan.default_baud)[0]["model_name"] = "XC330"
        if e["event"] == "awaiting_motor" and e["target_id"] == 1:
            bus.motors.append(motor(1, "XC430", plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    assert names.index("stray_motor_reset") < names.index("wrong_motor_detected")


def test_a_right_model_beyond_the_prefix_aborts_on_the_first_scan_only(monkeypatch):
    """Before the run has validated anything, a right-model motor beyond the
    prefix could be a configured motor behind a loose one: never reset blind."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan, [motor(2, "XC330", plan.target_baud)])
    with pytest.raises(mc.MotorChainAborted) as excinfo:
        mc.configure_motor_chain(plan, poll_interval=0)
    assert excinfo.value.invalid_motors == [motor(2, "XC330", plan.target_baud)]
    assert client.calls == []


def test_port_loss_while_waiting_is_survived_and_the_chain_reread(monkeypatch):
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []
    waited = []

    def fake_wait(port, *, present, **k):
        waited.append(present)
        bus.port_present = True          # the operator plugs the board back in

    monkeypatch.setattr(mc, "wait_for_port", fake_wait)

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 2 and not waited:
            bus.port_present = False     # cable knocked out while waiting for motor 2
        elif e["event"] == "awaiting_motor":
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    assert waited == [True]
    names = [e["event"] for e in events]
    assert "port_lost" in names
    assert names.count("step_started") == 3, "the step is announced once, not after every re-read"
    assert names.count("prescan_done") == 1


def test_an_id_write_the_motor_ignored_once_is_retried(monkeypatch):
    plan = small_plan()
    client = FakeSessionClient()
    client.ignore_id_writes = 1
    _wire_sessions(monkeypatch, client, plan=plan)
    events = []

    mc.configure_default_motor(plan, target_id=3, progress_callback=events.append)

    assert [c for c in client.calls if c[0] == "id"] == [("id", 1, 3), ("id", 1, 3)]
    retries = [e for e in events if e["event"] == "write_retry"]
    assert len(retries) == 1 and retries[0]["setting"] == "id" and retries[0]["still_default"]
    assert client.bus.motors == [motor(3, "XC330", plan.target_baud)]


def test_an_id_write_never_taken_fails_clearly_not_as_a_duplicate(monkeypatch):
    plan = small_plan()
    client = FakeSessionClient()
    client.ignore_id_writes = 99
    _wire_sessions(monkeypatch, client, plan=plan)
    events = []

    with pytest.raises(mc.MotorChainError, match="kept answering"):
        mc.configure_default_motor(plan, target_id=3, progress_callback=events.append)
    assert len([c for c in client.calls if c[0] == "id"]) == mc.WRITE_ATTEMPTS
    assert "duplicate_default_motor" not in [e["event"] for e in events]


def test_a_motor_that_goes_silent_after_the_id_write_is_named(monkeypatch):
    plan = small_plan()
    client = FakeSessionClient()
    _wire_sessions(monkeypatch, client, plan=plan)
    original = client.change_motor_id

    def vanish(current_id, new_id):
        original(current_id, new_id)
        client.bus.motors.clear()        # connector let go
        return True

    client.change_motor_id = vanish
    with pytest.raises(mc.MotorChainError, match="stopped answering"):
        mc.configure_default_motor(plan, target_id=3)


def test_a_single_dropped_verification_scan_is_retried(monkeypatch):
    plan = small_plan()
    bus = _wire(monkeypatch, plan)
    bus.plug("XC330", "XC330", "XC430")
    real_scan = bus.scan
    dropped = []

    def flaky_scan(motor_type, port, baudrate, id_range):
        if id_range == (1, plan.total_motors) and baudrate == plan.target_baud \
                and len(bus.motors) == 2 and not dropped:
            dropped.append(True)
            return []                      # one scan where nothing answers
        return real_scan(motor_type, port, baudrate, id_range)

    monkeypatch.setattr(mc, "scan_motors", flaky_scan)
    events = []
    assert mc.configure_motor_chain(plan, progress_callback=events.append, poll_interval=0) == [3, 2, 1]
    retries = [e for e in events if e["event"] == "verify_retry"]
    assert len(retries) == 1 and retries[0]["missing"] == [2, 3]


def test_persistent_verification_failure_names_the_missing_motors(monkeypatch):
    plan = small_plan()
    FakeBus([motor(3, "XC330", plan.target_baud)], plan).install(monkeypatch)
    monkeypatch.setattr(mc.time, "sleep", lambda _s: None)
    events = []
    with pytest.raises(mc.MotorChainError, match=r"Not answering: \[2\]"):
        mc.verify_chain(plan, [3, 2], progress_callback=events.append)
    assert [e["event"] for e in events] == ["verify_retry"] * (mc.VERIFY_ATTEMPTS - 1)


def test_a_configured_motor_that_drops_off_hands_its_slot_back(monkeypatch):
    """A validated motor dropping off the bus shrinks the chain: the run goes
    back to that slot and a fresh motor takes it. Nothing is reset."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []
    dropped = []

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.motors:
            bus.motors.append(motor(1, "XC330", plan.default_baud))
        elif e["event"] == "awaiting_motor" and e["target_id"] == 2 and not dropped:
            dropped.append(bus.motors.pop())       # motor 3's connector lets go
        elif e["event"] == "step_started" and e["target_id"] == 3 and dropped:
            bus.motors.append(motor(1, "XC330", plan.default_baud))   # a fresh one for slot 3
        elif e["event"] == "awaiting_motor" and e["target_id"] in (2, 1) and dropped \
                and not bus.at(plan.default_id, plan.default_baud):
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    assert names.count("configured_motor_missing") == 1
    shortened = [e for e in events if e["event"] == "chain_shortened"]
    assert [(e["missing"], e["configured_ids"]) for e in shortened] == [([3], [])]
    assert [e["target_id"] for e in events if e["event"] == "step_started"] == [3, 2, 3, 2, 1]
    assert "stray_motor_reset" not in names
    assert names.count("prescan_done") == 1


def test_a_dropped_motor_that_comes_back_is_taken_back_not_reset(monkeypatch):
    """The orphan still carries its ID: reconnecting it rejoins the chain."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []
    dropped = []

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.motors:
            bus.motors.append(motor(1, "XC330", plan.default_baud))
        elif e["event"] == "awaiting_motor" and e["target_id"] == 2 and not dropped:
            dropped.append(bus.motors.pop())
        elif e["event"] == "step_started" and e["target_id"] == 3 and dropped:
            bus.motors.append(dropped[0])          # plugged back in, still ID 3
        elif e["event"] == "awaiting_motor" and e["target_id"] in (2, 1) and dropped \
                and not bus.at(plan.default_id, plan.default_baud):
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    assert "configured_motor_returned" in names
    assert "stray_motor_reset" not in names
    assert [c for c in client.calls if c[0] == "id"] == [("id", 1, 3), ("id", 1, 2), ("id", 1, 1)]


def test_motors_stranded_behind_a_gap_are_kept_and_rejoin(monkeypatch):
    """Motor 2 was validated, then motor 3 (before it) dropped off. Motor 2 is
    not a stray: it is never reset, and when a fresh motor takes slot 3 the
    verification accepts motor 2 answering again."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []
    state = {"dropped": None}

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] in (3, 2) and state["dropped"] is None \
                and not bus.at(plan.default_id, plan.default_baud):
            bus.motors.append(motor(1, "XC330", plan.default_baud))
        if e["event"] == "awaiting_motor" and e["target_id"] == 1 and state["dropped"] is None:
            state["dropped"] = [m for m in bus.motors if m["id"] == 3][0]
            bus.motors.remove(state["dropped"])    # motor 3 lets go; 2 is behind it
        elif e["event"] == "step_started" and e["target_id"] == 3 and state["dropped"]:
            bus.motors.append(motor(1, "XC330", plan.default_baud))   # fresh motor for slot 3
        elif e["event"] == "awaiting_motor" and e["target_id"] == 1 and state["dropped"] \
                and not bus.at(plan.default_id, plan.default_baud):
            bus.motors.append(motor(1, "XC430", plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    shortened = [e for e in events if e["event"] == "chain_shortened"]
    assert [(e["missing"], e["stranded"]) for e in shortened] == [([3], [2])]
    assert "stray_motor_reset" not in names
    assert [c for c in client.calls if c[0] == "id" and c[1:] == (2, 1)] == [], "motor 2 untouched"
    assert sorted(m["id"] for m in bus.motors) == [1, 2, 3]


def test_verify_tolerates_validated_motors_rejoining(monkeypatch):
    plan = small_plan()
    FakeBus([motor(3, "XC330", plan.target_baud), motor(2, "XC330", plan.target_baud)], plan)\
        .install(monkeypatch)
    mc.verify_chain(plan, [3], tolerated_ids={2})
    with pytest.raises(mc.MotorChainError, match=r"Unexpected: \[2\]"):
        monkeypatch.setattr(mc.time, "sleep", lambda _s: None)
        mc.verify_chain(plan, [3])


def test_the_family_of_a_resumed_chain_is_read_off_its_configured_motors(monkeypatch):
    """Nothing answers at the factory defaults once every motor is programmed;
    the chain the config describes still identifies the family."""
    plan = small_plan()
    FakeBus([motor(3, "XC330", plan.target_baud), motor(2, "XC330", plan.target_baud)], plan)\
        .install(monkeypatch)
    events = []
    cfg = {"motor_ids": [1, 2, 3], "baudrate": plan.target_baud}

    assert mc.detect_motor_type("/dev/fake", events.append) is None
    assert mc.detect_motor_type("/dev/fake", events.append, chain_config=cfg) == DYNAMIXEL
    detected = [e for e in events if e["event"] == "motor_type_detected"]
    assert len(detected) == 1 and detected[0]["configured"] is True


def test_family_detection_from_the_chain_sweeps_the_family_rates_when_unpinned(monkeypatch):
    plan = small_plan()
    FakeBus([motor(3, "XC330", 3_000_000)], plan).install(monkeypatch)
    assert mc.detect_motor_type("/dev/fake", chain_config={"motor_ids": [1, 2, 3]}) == DYNAMIXEL
    assert mc.detect_motor_type("/dev/fake", chain_config={"motor_ids": [1, 2, 3],
                                                           "baudrate": 1_000_000}) is None


def test_waiting_reports_liveness(monkeypatch):
    plan = small_plan()
    bus = _wire(monkeypatch, plan)
    clock = {"now": 0.0}
    monkeypatch.setattr(mc.time, "monotonic", lambda: clock["now"])
    events = []

    def operator(e):
        events.append(e)
        if e["event"] == "still_waiting" and e["elapsed_s"] >= 2 * mc.HEARTBEAT_INTERVAL_S:
            bus.plug(e["expected_model"])

    real_scan = bus.scan

    def ticking_scan(*a, **k):
        clock["now"] += mc.HEARTBEAT_INTERVAL_S / 4
        return real_scan(*a, **k)

    monkeypatch.setattr(mc, "scan_motors", ticking_scan)
    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    beats = [e for e in events if e["event"] == "still_waiting"]
    assert len(beats) >= 6
    assert all(b["elapsed_s"] >= mc.HEARTBEAT_INTERVAL_S for b in beats)


# --- survey tiers and the bus survey -----------------------------------------


def test_census_tiers_widen_with_time_and_escalate():
    assert mc._census_tier(5, 0) == 1
    assert mc._census_tier(10, 0) == 2
    assert mc._census_tier(30, 0) == 3
    assert mc._census_tier(5, mc.ESCALATE_AFTER_S) == 3


def test_census_reports_what_it_checked(monkeypatch):
    plan = small_plan()
    bus = FakeBus(plan=plan)
    bus.install(monkeypatch)
    quick = mc._census(plan, [], 3, None, tier=1)
    deep = mc._census(plan, [], 3, None, tier=3)
    others = len(mc.valid_baudrates(DYNAMIXEL)) - 2
    assert quick["checked"] == [f"IDs 1-3 @ {plan.target_baud}", f"ID 1 @ {others} other baud rate(s)"]
    assert deep["checked"] == [f"IDs 1-{mc.FULL_ID_RANGE[1]} @ {plan.target_baud}",
                               f"IDs 1-3 @ {others} other baud rate(s)"]


def test_a_stray_at_a_foreign_baud_beyond_the_default_id_waits_for_tier_two(monkeypatch):
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    scans = []
    real_scan = bus.scan

    def counting_scan(motor_type, port, baudrate, id_range):
        scans.append((baudrate, id_range))
        return real_scan(motor_type, port, baudrate, id_range)

    monkeypatch.setattr(mc, "scan_motors", counting_scan)
    events = []

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.motors:
            bus.motors.append(motor(2, "XC330", 2_000_000))
        if e["event"] == "awaiting_motor" and e["target_id"] in (2, 1):
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    reset = [e for e in events if e["event"] == "stray_motor_reset"]
    assert [e["motor"]["id"] for e in reset] == [2]
    # tier 1 only pings the default ID at foreign bauds; tier 2 pinged the chain's IDs there
    assert (2_000_000, (1, 1)) in scans and (2_000_000, (1, 3)) in scans


def test_the_heartbeat_names_what_the_survey_checked(monkeypatch):
    plan = small_plan()
    bus = _wire(monkeypatch, plan)
    clock = {"now": 0.0}
    monkeypatch.setattr(mc.time, "monotonic", lambda: clock["now"])
    real_scan = bus.scan
    events = []

    def ticking_scan(*a, **k):
        clock["now"] += 1.0
        return real_scan(*a, **k)

    def operator(e):
        events.append(e)
        if e["event"] == "still_waiting" and e.get("escalated"):
            bus.plug(e["expected_model"])

    monkeypatch.setattr(mc, "scan_motors", ticking_scan)
    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    beats = [e for e in events if e["event"] == "still_waiting"]
    assert any(b["checked"] for b in beats)
    assert any(b["escalated"] for b in beats) and not beats[0]["escalated"]


def test_survey_bus_lists_every_family_baud_and_id(monkeypatch):
    found_by_family = {
        DYNAMIXEL: [motor(40, "XC330", 3_000_000)],
        FEETECH: [motor(7, "HLS3915", 500_000)],
    }

    def scan(motor_type, port, baudrate, id_range):
        assert id_range == mc.FULL_ID_RANGE
        return [m for m in found_by_family[motor_type] if m["baud_rate"] == baudrate]

    monkeypatch.setattr(mc, "scan_motors", scan)
    events = []
    found = mc.survey_bus("/dev/fake", events.append)
    assert found == [{"motor_type": DYNAMIXEL, **motor(40, "XC330", 3_000_000)},
                     {"motor_type": FEETECH, **motor(7, "HLS3915", 500_000)}]
    progress = [e for e in events if e["event"] == "survey_progress"]
    assert len(progress) == len(mc.valid_baudrates(DYNAMIXEL)) + len(mc.valid_baudrates(FEETECH))


# --- a bridge that ignores a baud, a host that cannot open one, a bus that changes under us


def test_a_rate_the_bridge_ignores_shows_the_chain_itself_and_is_never_a_stray(monkeypatch):
    """Through a USB bridge that does not honour 115200 the bus stays at the
    target rate, so a scan "at 115200" answers with the configured motors.
    They must not be reset, and the rate is dropped for the rest of the run."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    bus.mirrored.add(115_200)
    events = []
    scanned_bauds = []
    real_scan = bus.scan

    def counting_scan(motor_type, port, baudrate, id_range):
        scanned_bauds.append(baudrate)
        if baudrate == 115_200:
            assert not any(e["event"] == "phantom_baud" for e in events), \
                "a mirrored rate is skipped once recognised"
        return real_scan(motor_type, port, baudrate, id_range)

    monkeypatch.setattr(mc, "scan_motors", counting_scan)

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.motors:
            bus.motors.append(motor(1, "XC330", plan.default_baud))
        if e["event"] == "still_waiting" and e["target_id"] == 2 and e["polls"] > 12 \
                and not bus.at(plan.default_id, plan.default_baud):
            bus.motors.append(motor(1, "XC330", plan.default_baud))
        if e["event"] == "awaiting_motor" and e["target_id"] == 1:
            bus.motors.append(motor(1, "XC430", plan.default_baud))

    clock = {"now": 0.0}
    monkeypatch.setattr(mc.time, "monotonic", lambda: clock["now"])
    orig = counting_scan

    def ticking(*a, **k):
        clock["now"] += 1.0
        return orig(*a, **k)

    monkeypatch.setattr(mc, "scan_motors", ticking)
    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    assert "stray_motor_reset" not in names
    phantom = [e for e in events if e["event"] == "phantom_baud"]
    assert len(phantom) == 1 and phantom[0]["baudrate"] == 115_200 and phantom[0]["motor_ids"] == [3]
    assert 115_200 in scanned_bauds


def test_a_rate_the_host_cannot_open_is_skipped_quietly(monkeypatch):
    plan = small_plan()
    bus = FakeBus(plan=plan)
    bus.unopenable.add(10_500_000)
    bus.install(monkeypatch)
    scanned = []
    real_scan = bus.scan
    monkeypatch.setattr(mc, "scan_motors",
                        lambda mt, port, baud, ids: scanned.append(baud) or real_scan(mt, port, baud, ids))
    state = {"skip_bauds": set()}
    mc._census(plan, [], 3, None, tier=1, state=state)
    assert 10_500_000 not in scanned and 10_500_000 in state["skip_bauds"]


def test_only_one_stray_is_reset_per_survey(monkeypatch):
    """Two strays reset together would meet at the default ID."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan,
                             [motor(1, "XC330", 2_000_000), motor(2, "XC330", 2_000_000)])
    events = []
    mc._census(plan, [], 3, events.append, tier=2, state={"skip_bauds": set()})
    assert [e["motor"]["id"] for e in events if e["event"] == "stray_motor_reset"] == [1]
    assert len(bus.at(plan.default_id, plan.default_baud)) == 1
    assert bus.at(2, 2_000_000), "the second stray waits for the next survey"


def test_two_strays_at_a_foreign_baud_are_taken_one_per_survey(monkeypatch):
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []

    def operator(e):
        events.append(e)
        if e["event"] == "awaiting_motor" and e["target_id"] == 3 and not bus.motors:
            bus.motors.extend([motor(1, "XC330", 2_000_000), motor(2, "XC330", 2_000_000)])
        if e["event"] == "awaiting_motor" and e["target_id"] == 1:
            bus.motors.append(motor(1, "XC430", plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    first, second = (i for i, n in enumerate(names) if n == "stray_motor_reset")
    assert "motor_configured" in names[first:second]


def test_a_stray_carrying_a_configured_id_at_another_baud_is_left_alone(monkeypatch):
    """Indistinguishable from the chain seen through a bridge that ignored the
    rate; never reset on a guess."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan,
                             [motor(3, "XC330", plan.target_baud), motor(3, "XC330", 2_000_000)])
    events = []
    mc._census(plan, [3], 2, events.append, tier=2, state={"skip_bauds": set()})
    names = [e["event"] for e in events]
    assert "stray_motor_reset" not in names and "phantom_baud" in names
    assert bus.at(3, 2_000_000)


def test_a_failed_verification_rereads_the_chain_instead_of_ending(monkeypatch):
    """Motor 3 drops off right after it was programmed: verification fails,
    the chain is re-read and shrinks, and a fresh motor takes slot 3."""
    plan = small_plan()
    bus, client = _wire_real(monkeypatch, plan)
    events = []
    knocked = []

    def operator(e):
        events.append(e)
        if e["event"] == "motor_configured" and e["target_id"] == 3 and not knocked:
            knocked.append(bus.motors.pop())      # connector lets go before the chain check
        if e["event"] in ("awaiting_motor", "step_started") \
                and not bus.at(plan.default_id, plan.default_baud) \
                and (e["event"] == "awaiting_motor" or knocked):
            bus.motors.append(motor(1, e["expected_model"], plan.default_baud))

    assert mc.configure_motor_chain(plan, progress_callback=operator, poll_interval=0) == [3, 2, 1]
    names = [e["event"] for e in events]
    assert names.count("verify_failed") == 1
    assert names.index("verify_failed") < names.index("chain_shortened")
    assert names[-1] == "chain_done"
    assert sorted(m["id"] for m in bus.motors) == [1, 2, 3]
