#!/usr/bin/env python3
import argparse
import sys
import os
import json
import time
import websocket
import logging
import threading
import yaml  # type: ignore

# Add the parent directory to the Python path so we can import orca_core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orca_core import OrcaHand, MockOrcaHand

# Use shared YAML utilities
from orca_core.utils.utils import read_yaml

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('remote_control')

# ================================
# Constants and Globals
# ================================

# JOINT_NAMES is now dynamically loaded from the hand model config (hand.joint_ids)


# Global state
SERVER = "ws://localhost:8082/ws/orca-backend"
SERVER_SECRET = os.environ.get("ORCA_SERVER_SECRET")

current_client_id = None
latest_command = None
command_lock = threading.Lock()
command_available = threading.Event()
blocking_slider_commands = False
waypoint_playback_active = False
waypoint_playback_stop_event = threading.Event()
shutdown_event = threading.Event()

# ================================
# Utility Functions
# ================================

def get_and_clear_command_buffer():
    """Atomically get and clear the latest command from the buffer."""
    global latest_command
    with command_lock:
        cmd = latest_command
        latest_command = None
        command_available.clear()
        return cmd


def get_client_file_path(client_id: str) -> str:
    """Get the file path for a client's waypoints."""
    client_short = (client_id or "")[:8]
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'saved_positions', f"client_{client_short}.yaml")


def save_yaml_positions(client_file: str, positions: dict):
    """Save positions dict to YAML file."""
    with open(client_file, "w") as file:
        yaml.dump(positions, file, default_flow_style=False)

def send_error(ws, message: str):
    ws.send(json.dumps({"action": "response_error", "data": message}))


# ================================
# Command Buffer Processing
# ================================
def command_processor_thread(hand):
    """Thread that processes the latest command from the buffer."""
    global current_client_id, blocking_slider_commands
    while not shutdown_event.is_set():
        try:
            if not command_available.wait(timeout=1.0) or blocking_slider_commands:
                continue
            cmd = get_and_clear_command_buffer()
            if not cmd:
                continue
            hand.set_joint_pos(cmd)
        except Exception as e:
            logger.error(f"Command processor error: {e}")
            time.sleep(0.1)


# ================================
# Waypoint and Joint Management
# ================================




def delete_last_waypoint(client_id=None):
    """Delete the last (most recent) waypoint from a client's YAML file."""
    client_file = get_client_file_path(client_id)
    client_positions = read_yaml(client_file)
    if not client_positions:
        logger.warning(f"No waypoints found in file: {client_file}")
        return 0
    # Sort by timestamp string after 'waypoint_'
    def extract_timestamp(key):
        if key.startswith("waypoint_"):
            return key[len("waypoint_"):]
        return ""
    sorted_keys = sorted(client_positions.keys(), key=extract_timestamp)
    if not sorted_keys:
        logger.warning(f"No waypoints found to delete in: {client_file}")
        return 0
    last_key = sorted_keys[-1]
    del client_positions[last_key]
    logger.info(f"Deleted waypoint: {last_key}")
    if client_positions:
        save_yaml_positions(client_file, client_positions)
    else:
        os.remove(client_file)
        logger.info(f"No waypoints remaining, deleted file: {client_file}")
    return len(client_positions)


def load_waypoints(client_file: str, joint_ids):
    positions = read_yaml(client_file)
    if len(positions) < 2:
        raise ValueError(f"Need at least 2 waypoints, found {len(positions)}")
    return [[joint_dict.get(name, 0.0) for name in joint_ids] for joint_dict in positions.values()]




def get_waypoint_count(client_id):
    """Get the number of saved waypoints for a client."""
    try:
        client_file = get_client_file_path(client_id)
        return len(read_yaml(client_file))
    except Exception as e:
        return 0
    

def reset_to_neutral(hand):
    """Reset the hand to neutral position"""
    global blocking_slider_commands
    blocking_slider_commands = True
    get_and_clear_command_buffer()
    try:
        hand.set_neutral_position(num_steps=25, step_size=0.001)
    finally:
        blocking_slider_commands = False


# ================================
# Server Message Handlers
# ================================

def is_authorized_client(data, ws):
    client_id = data.get('clientId')
    if client_id is None or (current_client_id is not None and client_id != current_client_id):
        send_error(ws, "Unauthorized: You are not the active client")
        return False
    return True


def secure_new_active_client(hand, data, _):
    """Switch active client, secured by server secret."""
    global current_client_id, waypoint_playback_active, waypoint_playback_stop_event
    client_id = data.get('clientId')
    if client_id != current_client_id:
        logger.info(f"Switching active client: {current_client_id} -> {client_id}")
        reset_to_neutral(hand)
        current_client_id = client_id


def query_reset_to_neutral(hand, data, ws):
    """Reset hand to neutral, stopping playback if active."""
    if is_authorized_client(data, ws):
        global waypoint_playback_active, waypoint_playback_stop_event
        if waypoint_playback_active:
            waypoint_playback_stop_event.set()
            waypoint_playback_active = False
        else:
            reset_to_neutral(hand)


def query_save_waypoints(hand, data, ws):
    """Handle save joints command"""
    if is_authorized_client(data, ws):
        current_angles = hand.get_joint_pos(as_list=True)
        client_file = get_client_file_path(data.get('clientId'))
        joint_dict = {name: float(angle) for name, angle in zip(hand.joint_ids, current_angles)}
        client_positions = read_yaml(client_file)
        client_positions[f"waypoint_{time.strftime('%Y%m%d_%H%M%S')}"] = joint_dict
        save_yaml_positions(client_file, client_positions)
        logger.info(f"Positions saved to {client_file}")


def query_get_waypoint_count(_, data, ws):
    """Handle get waypoint count command"""
    if is_authorized_client(data, ws):
        ws.send(json.dumps({
            "action": "response_waypoint_count_updated",
            "data": {"count": get_waypoint_count(data.get("clientId", ""))}
        }))


def query_delete_last_waypoint(_, data, ws):
    """Handle delete last waypoint command"""
    if is_authorized_client(data, ws):
        ws.send(json.dumps({
            "action": "response_waypoint_count_updated",
            "data": {"count": delete_last_waypoint(data.get("clientId", ""))}
        }))


def query_start_waypoint_playback(hand, data, ws):
    """Handle play waypoints command"""
    if not is_authorized_client(data, ws):
        return

    global waypoint_playback_active, blocking_slider_commands
    if waypoint_playback_active:
        hand.stop_task()

    client_id = data.get("clientId", "")
    count = get_waypoint_count(client_id)
    if count < 2:
        send_error(ws, f"Need at least 2 waypoints, found {count}")
        return

    logger.info(f"Starting waypoint playback for client: {client_id}")
    waypoint_playback_active = blocking_slider_commands = True
    
    client_file = get_client_file_path(client_id)
    if not os.path.exists(client_file):
        handle_playback_finish(hand, ws)
        return
    
    waypoints = load_waypoints(client_file, hand.joint_ids)
    hand.replay_waypoints(waypoints, max_iterations=50, mode="ease_in_out", on_finish=lambda: handle_playback_finish(hand, ws), blocking=False)

def handle_playback_finish(hand, ws):
    ws.send(json.dumps({"action": "response_playback_stopped", "data": None}))
    logger.info(f"Playback cleanup")
    reset_to_neutral(hand)
    global waypoint_playback_active, blocking_slider_commands
    waypoint_playback_active = blocking_slider_commands = False
    

def query_stop_waypoint_playback(hand, data, ws):
    """Handle stop playback command"""
    logger.info("Received stop playback command")
    if is_authorized_client(data, ws):
        global waypoint_playback_active
        logger.info(f"Stopping playback for client: {data.get('clientId')}")
        if waypoint_playback_active:
            logger.info("Stopping waypoint playback")
            # Use the new API to stop the background task
            hand.stop_task()
        waypoint_playback_active = False


def query_slider_control(_, data, ws):
    """Handle slider command"""
    if is_authorized_client(data, ws):
        global blocking_slider_commands, latest_command
        if not blocking_slider_commands:
            with command_lock:
                latest_command = data.get("command", [])
                command_available.set()
                logger.info(f"Setting command: {latest_command}")


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
        logger.error(f"Failed to connect: {message}")
        return 1
    
    hand.enable_torque()
    hand.set_neutral_position(num_steps=25, step_size=0.001)
    logger.info("Hand initialized and ready")

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
                ws.send(json.dumps({"action": "response_error", "data": message}))
                continue
    except Exception as e:
        logger.error(f"Websocket error: {e}")
    except KeyboardInterrupt:
        logger.info("Shutting down due to keyboard interrupt...")
    finally:
        logger.info("Disabling torque and disconnecting hand...")
        shutdown_event.set()
        hand.stop_task()
        hand.disable_torque()
        hand.disconnect()
        if ws is not None:
            ws.close()


if __name__ == "__main__":
    sys.exit(main())