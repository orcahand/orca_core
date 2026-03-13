#!/usr/bin/env python3
"""Web-based ORCA Hand Dashboard - joint control and diagnostics."""

from flask import Flask, render_template, jsonify, request
from orca_core import OrcaHand
import argparse
import threading
import os

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky", "wrist"]

app = Flask(__name__,
            template_folder=os.path.join(os.path.dirname(__file__), "dashboard_ui", "templates"),
            static_folder=os.path.join(os.path.dirname(__file__), "dashboard_ui", "static"))

hand = None
torque_on = False


def get_grouped_joints():
    grouped = {f: [] for f in FINGER_ORDER}
    for joint in hand.joint_ids:
        finger = joint.split("_")[0]
        if finger in grouped:
            grouped[finger].append(joint)
    return {f: joints for f, joints in grouped.items() if joints}


@app.route("/")
def index():
    joints_info = {}
    for joint in hand.joint_ids:
        rom_min, rom_max = hand.joint_roms_dict[joint]
        joints_info[joint] = {"min": rom_min, "max": rom_max}
    return render_template("dashboard.html",
                           grouped=get_grouped_joints(),
                           joints_info=joints_info)


@app.route("/api/positions")
def get_positions():
    try:
        positions = hand.get_joint_pos(as_list=False)
        return jsonify({j: round(v, 2) if v is not None else None for j, v in positions.items()})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/set_position", methods=["POST"])
def set_position():
    if not torque_on:
        return jsonify({"error": "Torque is off"}), 400
    data = request.json
    joint = data.get("joint")
    value = data.get("value")
    try:
        hand.set_joint_pos({joint: float(value)})
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/set_positions", methods=["POST"])
def set_positions():
    if not torque_on:
        return jsonify({"error": "Torque is off"}), 400
    data = request.json
    try:
        hand.set_joint_pos({j: float(v) for j, v in data.items()})
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/torque", methods=["POST"])
def toggle_torque():
    global torque_on
    action = request.json.get("action")
    if action == "enable":
        hand.enable_torque()
        hand.set_control_mode(hand.control_mode)
        hand.set_max_current(hand.max_current)
        if not hand.calibrated:
            hand.calibrate()
        hand._compute_wrap_offsets_dict()
        torque_on = True
    else:
        hand.disable_torque()
        torque_on = False
    return jsonify({"torque_on": torque_on})


@app.route("/api/neutral", methods=["POST"])
def go_neutral():
    if not torque_on:
        return jsonify({"error": "Torque is off"}), 400
    threading.Thread(target=lambda: hand.set_neutral_position(num_steps=25, step_size=0.02), daemon=True).start()
    neutral = {j: float(v) for j, v in hand.neutral_position.items() if v is not None}
    return jsonify({"ok": True, "positions": neutral})


@app.route("/api/diagnostics")
def get_diagnostics():
    try:
        # Build motor_id → joint_name from joint_to_motor_map
        mid_to_joint = {mid: j for j, mid in hand.joint_to_motor_map.items()}

        temps = hand.get_motor_temp(as_dict=True)
        currents = hand.get_motor_current(as_dict=True)

        temp_data = {}
        current_data = {}
        for mid, temp in temps.items():
            joint = mid_to_joint.get(mid)
            if joint:
                temp_data[joint] = round(float(temp), 1)
        for mid, cur in currents.items():
            joint = mid_to_joint.get(mid)
            if joint:
                current_data[joint] = round(float(cur), 1)

        return jsonify({"temps": temp_data, "currents": current_data})
    except Exception as e:
        print(f"Diagnostics error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500


def main():
    global hand

    parser = argparse.ArgumentParser(description="ORCA Hand Dashboard (web-based)")
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the hand model directory")
    parser.add_argument("--port", type=int, default=5002, help="Web server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind to")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    print(f"\n  Dashboard: http://localhost:{args.port}\n")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
