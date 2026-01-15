#!/usr/bin/env python3
import argparse
import sys
import os
import json
import time
import websocket
import logging
import threading
from typing import Any

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orca_core import OrcaHand, MockOrcaHand
from orca_core.utils.utils import read_yaml, get_yaml_path_and_waypoints, update_yaml


# ================================
# Configuration & Globals
# ================================

SERVER = "ws://localhost:8082/ws/orca-backend"
client_replay_dir = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'replay_sequences_clients'
)
current_client_id = None
latest_command = None
command_lock = threading.Lock()
command_available = threading.Event()
blocking_slider_commands = False
waypoint_playback_active = False
shutdown_event = threading.Event()


# ================================
# Slider Control Functions
# ================================

def get_and_clear_command_buffer() -> Any:
    global latest_command
    with command_lock:
        cmd = latest_command
        latest_command = None
        command_available.clear()
        return cmd
    
def command_processor_thread(hand: Any) -> None:
    global blocking_slider_commands
    while not shutdown_event.is_set():
        try:
            if not command_available.wait(timeout=1.0) or blocking_slider_commands:
                continue
            cmd = get_and_clear_command_buffer()
            if cmd:
                hand.set_joint_pos(cmd)
        except Exception as e:
            time.sleep(0.1)
    
def query_slider_control(_, data, ws):
    if is_authorized_client(data, ws):
        global blocking_slider_commands, latest_command
        if not blocking_slider_commands:
            with command_lock:
                latest_command = data.get("command", [])
                command_available.set()
                print(f"Received slider command: {latest_command}")


# ================================
# Client Management Handlers
# ================================

def is_authorized_client(data: dict, ws) -> bool:
    client_id = data.get('clientId')
    if client_id is None or (current_client_id is not None and client_id != current_client_id):
        ws.send(json.dumps({ "action": "response_error", "data": "Unauthorized! You are not the active client"}))
        return False
    return True

def secure_new_active_client(hand, data, _):
    global current_client_id, waypoint_playback_active
    client_id = data.get('clientId')
    if client_id != current_client_id:
        if waypoint_playback_active:
            hand.stop_task()
            waypoint_playback_active = False
        else:
            reset_to_neutral(hand)
        current_client_id = client_id
        print(f"Setting new active client: {current_client_id}")



# ================================
# Reset Management Functions
# ================================

def reset_to_neutral(hand: Any) -> None:
    print("Resetting hand to neutral position...")
    global blocking_slider_commands
    blocking_slider_commands = True
    get_and_clear_command_buffer()
    try:
        hand.set_neutral_position(num_steps=25, step_size=0.001)
    finally:
        blocking_slider_commands = False

def query_reset_to_neutral(hand, data, ws):
    if is_authorized_client(data, ws):
        global waypoint_playback_active
        if waypoint_playback_active:
            hand.stop_task()
            waypoint_playback_active = False
        else:
            reset_to_neutral(hand)


# ================================
# Waypoint Management Functions
# ================================

def query_save_waypoints(hand, data, ws):
    if is_authorized_client(data, ws):
        current_angles = hand.get_joint_pos(as_list=True)
        client_file, waypoints = get_yaml_path_and_waypoints(client_replay_dir, data.get('clientId'))
        waypoints.append([float(angle) for angle in current_angles])
        update_yaml(client_file, "waypoints", waypoints)
        print("Waypoint saved:", waypoints[-1])

def query_get_waypoint_count(_, data, ws):
    if is_authorized_client(data, ws):
        _, waypoints = get_yaml_path_and_waypoints(client_replay_dir, data.get("clientId", ""))
        ws.send(json.dumps({ "action": "response_waypoint_count_updated", "data": {"count": len(waypoints)}}))

def query_delete_last_waypoint(_, data, ws):
    if is_authorized_client(data, ws):
        client_id = data.get("clientId", "")
        client_file, waypoints = get_yaml_path_and_waypoints(client_replay_dir, client_id)
        if waypoints:
            waypoints.pop()
            if waypoints:
                update_yaml(client_file, "waypoints", waypoints)
            else:
                os.remove(client_file)
        count = len(waypoints)
        ws.send(json.dumps({ "action": "response_waypoint_count_updated", "data": {"count": count}}))
        print(f"Deleted last waypoint.")

def query_start_waypoint_playback(hand, data, ws):
    if not is_authorized_client(data, ws):
        return
    global waypoint_playback_active, blocking_slider_commands
    if waypoint_playback_active:
        hand.stop_task()
    client_file, waypoints = get_yaml_path_and_waypoints(client_replay_dir, data.get("clientId", ""))
    if not os.path.exists(client_file) or not (2 <= len(waypoints) <= 10):
        ws.send(json.dumps({"action": "response_error", "data": f"Waypoint file does not exist or number of waypoints is not between 2 and 10."}))
        return
    print(f"Starting waypoint playback")
    waypoint_playback_active = blocking_slider_commands = True
    hand.replay_waypoints(waypoints, max_iterations=50, mode="ease_in_out", on_finish=lambda: handle_playback_finish(hand, ws), blocking=False)

def handle_playback_finish(hand, ws):
    ws.send(json.dumps({"action": "response_playback_stopped", "data": None}))
    print("Stopping waypoint playback")
    reset_to_neutral(hand)
    global waypoint_playback_active, blocking_slider_commands
    waypoint_playback_active = blocking_slider_commands = False

def query_stop_waypoint_playback(hand, data, ws):
    if is_authorized_client(data, ws):
        global waypoint_playback_active
        if waypoint_playback_active:
            hand.stop_task()
        waypoint_playback_active = False


# ================================
# Main Entry Point
# ================================
def main():
    parser = argparse.ArgumentParser(
        description='Remote control of orcahand via websocket server',
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1). Not required in mock mode."
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run in mock mode (no physical robot required)"
    )
    args = parser.parse_args()
    hand = MockOrcaHand(model_path=args.model_path) if args.mock else OrcaHand(model_path=args.model_path)
    success, message = hand.connect()
    if not success:
        print(f"Failed to connect: {message}")
        return 1
    hand.enable_torque()
    hand.set_neutral_position(num_steps=25, step_size=0.001)
    print("Hand initialized and ready")
    processor_thread = threading.Thread(target=command_processor_thread, args=(hand,), daemon=True)
    processor_thread.start()
    ws = None
    try:
        ws = websocket.create_connection(SERVER)
        while True:
            message = ws.recv()
            try:
                data = json.loads(message)
                action = data.get("action")
                if action and (action.startswith('query_') or action.startswith('secure_')) and action in globals():
                    globals()[action](hand, data, ws)
            except Exception as e:
                ws.send(json.dumps({"action": "response_error", "data": f"Error processing message: {e}"}))
                continue
    except Exception as e:
        print(f"Websocket error: {e}")
    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt...")
    finally:
        print("Disabling torque and disconnecting hand...")
        hand.stop_task()
        shutdown_event.set()
        hand.disable_torque()
        hand.disconnect()
        if ws is not None:
            ws.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())