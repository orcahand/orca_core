#!/usr/bin/env python
"""Report what hand autodetection finds on the connected hardware.

Probes the controller board, prints the bundled model that matches (side +
sensing capabilities), the resolved serial ports, and whether that model's
calibration is ready for closed-loop joint feedback.

Nothing is driven and no motor is torqued: this only reads.

Usage:
    uv run python scripts/detect.py
"""
import argparse
import contextlib
import io
import sys

from orca_core import HandSelector, detect_hand, detect_hands, load_hand
from orca_core.hardware.sensing.serial_discovery import count_classic_motor_adapters


@contextlib.contextmanager
def _quiet():
    """Swallow what constructing a hand prints: the model-path banner and one
    warning per uncalibrated motor. This script reports that state itself."""
    with contextlib.redirect_stdout(io.StringIO()):
        yield


def _describe(tactile: bool, encoders: bool) -> str:
    present = [n for n, on in (("tactile", tactile), ("encoders", encoders)) if on]
    return " + ".join(present) if present else "nothing"


def _capabilities(detection) -> str:
    present = [detection.side]
    if detection.has_tactile:
        present.append("tactile")
    if detection.has_encoders:
        present.append("encoders")
    return ", ".join(present)


def _motor_family(detection) -> str:
    """The motor family answering on the bus, as a suffix for the port line."""
    if detection.motor_type is None:
        return "" if detection.motor_port is None else "  (no motor answered)"
    baud = f" @ {detection.motor_baudrate}" if detection.motor_baudrate else ""
    return f"  ({detection.motor_type}{baud})"


def _print_detection(detection) -> None:
    print(f"Model:       {detection.model_name}  ({_capabilities(detection)})")

    identity = detection.identity
    if identity is not None:
        print(
            f"Identity:    {identity.hand_id or 'unprovisioned'} "
            f"(hw {identity.hw_version}, fw {identity.fw_version})"
        )
    elif detection.board_id is not None:
        print(f"Identity:    {detection.board_id} (from USB descriptor; "
              "the board did not answer)")
    else:
        print("Identity:    not reported")

    if detection.declared_config is None:
        print("Sensing cfg: not declared (capabilities came from probing)")
    else:
        probed = _describe(detection.probed_tactile, detection.probed_encoders)
        print(f"Sensing cfg: CFG={detection.declared_config} declared, {probed} responding")

    print(f"Motor bus:   {detection.motor_port or 'not detected'}{_motor_family(detection)}")
    print(f"Sensing:     {detection.sensing_port or 'not detected'}")
    if detection.tactile_port and detection.tactile_port != detection.sensing_port:
        print(f"Tactile:     {detection.tactile_port}")


def _print_calibration(detection) -> None:
    """Report the calibration state of the bundled model detection picked."""
    with _quiet():
        # Motor-only: this only reads calibration off disk, and opening
        # the sensing links would fight anything already on that port.
        # select= pins this to the hand this detection describes: load_hand()
        # re-probes the bus regardless of model_name, and with several hands
        # attached an unselected re-probe is ambiguous between all of them.
        hand = load_hand(
            model_name=detection.model_name,
            engage_feedback=False,
            engage_sensors=False,
            select=HandSelector(hand_id=detection.hand_id) if detection.hand_id else None,
        )

    print(f"\nCalibration ({hand.config.model_path}):")
    print(f"  motor limits:    {'ok' if hand.calibrated else 'NOT calibrated'}")
    print(f"  wrist:           {'ok' if hand.wrist_calibrated else 'NOT calibrated'}")

    if not hand.config.joint_feedback_enabled:
        return

    anchored = hand.calibration.joint_encoder_calibration_dict
    backed = hand.encoder_backed_joints
    missing = [joint for joint in backed if joint not in anchored]
    print(f"  encoder anchors: {len(backed) - len(missing)}/{len(backed)} joints")

    if not missing:
        return
    # Naming names only helps while a few joints are outstanding; on a model
    # anchored nowhere the count above already says everything.
    if len(missing) < len(backed):
        print(f"    missing: {', '.join(missing)}")
    print("    Run scripts/calibrate.py; closed-loop connect() fails without these.")


def _print_notes(detection) -> None:
    """Explain anything detection could not see, so a partial result is never
    mistaken for a simpler hand."""
    notes = []

    if detection.missing_capabilities:
        notes.append(
            f"This hand declares {' and '.join(detection.missing_capabilities)}, "
            f"but that hardware did not respond on "
            f"{detection.sensing_port or 'any port'}. The model above is loaded "
            "from the declaration, so connect() will fail until it is fixed. "
            "Check the sensing cable and power-cycle the hand, then run "
            "scripts/check_sensors.py."
        )
    if detection.undeclared_capabilities:
        notes.append(
            f"{' and '.join(detection.undeclared_capabilities)} answered but "
            f"CFG={detection.declared_config} does not declare it, so it stays "
            "unused. The board is provisioned wrong — re-provision it."
        )
    if detection.busy_ports:
        notes.append(
            f"{', '.join(detection.busy_ports)} is held by another process (a "
            "running UI, script or serial monitor). Ports are probed "
            "exclusively, so whatever sits behind it is missing above — close "
            "the other client and re-run."
        )
    elif detection.identity is None and detection.motor_port is None:
        notes.append(
            "No controller board answered. With nothing plugged in, detection "
            "falls back to the plain right-hand model above."
        )
    elif detection.identity is None:
        notes.append(
            "No controller board answered an identity query (predates "
            "ORCA_ID?/ORCA_INFO?, or there is no controller board at all — a "
            "legacy hand), so side and sensing capabilities could not be read "
            "from hardware. The motor bus above was found by USB-vendor-ID "
            "probing instead; name the model explicitly if it isn't a plain "
            "right hand."
        )
    elif detection.motor_port is None:
        notes.append(
            "No port identified itself as the motor bus. connect() falls back "
            "to matching a motor adapter by USB vendor ID."
        )

    for note in notes:
        print(f"\nNote: {note}")


def _report(detection) -> None:
    _print_detection(detection)
    try:
        _print_calibration(detection)
    except Exception as e:
        print(f"\nCould not read calibration for {detection.model_name}: {e}")
    _print_notes(detection)


def main() -> None:
    argparse.ArgumentParser(description=__doc__.split("\n", 1)[0]).parse_args()

    try:
        detections = detect_hands()
    except Exception as e:
        print(f"Detection failed: {e}")
        sys.exit(1)

    if not detections:
        legacy_count = count_classic_motor_adapters()
        if legacy_count > 1:
            print(
                f"{legacy_count} bare Feetech/Dynamixel motor adapters are plugged "
                "in with no controller board behind any of them, so none can be "
                "matched to a single legacy hand — nothing is reported below "
                "instead of guessing which is which.\n"
                "Pin each hand's port: explicitly in its own "
                "orca_core/models/local/<name>/config.yaml (sensors.port: too, "
                "for a touch hand — see that directory's README) and run this "
                "script with that config_path to inspect one at a time."
            )
            return
        _report(detect_hand())
        return

    for index, detection in enumerate(detections):
        if len(detections) > 1:
            if index:
                print()
            name = detection.hand_id or "unidentified"
            print(f"=== Hand {index + 1}/{len(detections)}: {name} ===")
        _report(detection)


if __name__ == "__main__":
    main()
