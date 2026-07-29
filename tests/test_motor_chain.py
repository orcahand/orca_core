"""Tests for the motor-chain maintenance operation.

The bus is faked: ``scan_motors`` is replaced by a stateful in-memory chain, so
the ID/baud bookkeeping, resume logic and callback contract are exercised
without hardware. The module under test is family-agnostic, so the family facts
come from the real client classes via ``motor_client_class``.
"""

import pytest

from orca_core.constants import DYNAMIXEL, FEETECH
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
        return [m for m in self.motors if m["baud_rate"] == baudrate and lo <= m["id"] <= hi]

    def _default_motor(self):
        return [m for m in self.motors
                if m["id"] == self.plan.default_id and m["baud_rate"] == self.plan.default_baud]

    def install(self, monkeypatch):
        monkeypatch.setattr(mc, "scan_motors", self.scan)


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


# --- configure_motor_chain --------------------------------------------------


def small_plan(motor_type=DYNAMIXEL) -> mc.MotorChainPlan:
    cfg = {"motor_ids": [1, 2, 3], "joint_to_motor_map": {"wrist": 1}, "baudrate": 1_000_000}
    return mc.plan_motor_chain(cfg, "/dev/fake", motor_type)


def _wire(monkeypatch, plan):
    """A bus whose configure step moves the factory-default motor to its target."""
    bus = FakeBus(plan=plan)
    bus.install(monkeypatch)

    def fake_configure(p, target_id):
        fresh = bus._default_motor()
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
