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

from orca_core import detect_hand, load_hand


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


def _print_detection(detection) -> None:
    print(f"Model:       {detection.model_name}  ({_capabilities(detection)})")

    identity = detection.identity
    if identity is None:
        print("Identity:    not reported")
    else:
        print(
            f"Identity:    {identity.hand_id or 'unprovisioned'} "
            f"(hw {identity.hw_version}, fw {identity.fw_version})"
        )

    if detection.declared_config is None:
        print("Sensing cfg: not declared (capabilities came from probing)")
    else:
        probed = _describe(detection.probed_tactile, detection.probed_encoders)
        print(f"Sensing cfg: CFG={detection.declared_config} declared, {probed} responding")

    print(f"Motor bus:   {detection.motor_port or 'not detected'}")
    print(f"Sensing:     {detection.sensing_port or 'not detected'}")
    if detection.tactile_port and detection.tactile_port != detection.sensing_port:
        print(f"Tactile:     {detection.tactile_port}")


def _print_calibration(detection) -> None:
    """Report the calibration state of the bundled model detection picked."""
    with _quiet():
        # Motor-only: this only reads calibration off disk, and opening
        # the sensing links would fight anything already on that port.
        hand = load_hand(
            model_name=detection.model_name,
            engage_feedback=False,
            engage_sensors=False,
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
    elif detection.identity is None:
        notes.append(
            "No controller board answered. With nothing plugged in, detection "
            "falls back to the plain right-hand model above."
        )
    elif detection.motor_port is None:
        notes.append(
            "No port identified itself as the motor bus. connect() falls back "
            "to matching a motor adapter by USB vendor ID."
        )

    for note in notes:
        print(f"\nNote: {note}")


def main() -> None:
    argparse.ArgumentParser(description=__doc__.split("\n", 1)[0]).parse_args()

    try:
        detection = detect_hand()
    except Exception as e:
        print(f"Detection failed: {e}")
        sys.exit(1)

    _print_detection(detection)

    try:
        _print_calibration(detection)
    except Exception as e:
        print(f"\nCould not read calibration for {detection.model_name}: {e}")

    _print_notes(detection)


if __name__ == "__main__":
    main()
