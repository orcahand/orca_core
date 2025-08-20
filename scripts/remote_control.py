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
from replay_angles import ease_in_out

# Use shared YAML utilities
from orca_core.utils.yaml_utils import read_yaml

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
current_client_id = None
latest_command = None
command_lock = threading.Lock()
command_available = threading.Event()
blocking_slider_commands = False
waypoint_playback_active = False
waypoint_playback_thread = None
waypoint_playback_stop_event = threading.Event()

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

def set_command_buffer(new_command):
    """Fill the command buffer with a new command array."""
    global latest_command
    with command_lock:
        latest_command = new_command
        command_available.set()

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
    while True:
        try:
            if not command_available.wait(timeout=1.0) or blocking_slider_commands:
                continue
            cmd = get_and_clear_command_buffer()
            if not cmd:
                continue
            if current_client_id and cmd.get("clientId") != current_client_id:
                continue
            hand.set_joint_pos(cmd.get("command", []))
        except Exception as e:
            logger.error(f"Command processor error: {e}")
            time.sleep(0.1)


# ================================
# Waypoint and Joint Management
# ================================
def reset_to_neutral(hand):
    """Reset the hand to neutral position"""
    global blocking_slider_commands
    blocking_slider_commands = True
    get_and_clear_command_buffer()  # Clear any pending commands
    try:
        hand.set_neutral_position(num_steps=25, step_size=0.001)
    finally:
        blocking_slider_commands = False


def save_joint_positions(hand, client_id=None):
    """Save the current joint positions to a client-specific YAML file."""
    current_angles = hand.get_joint_pos(as_list=True)
    client_file = get_client_file_path(client_id)
    joint_dict = {name: float(angle) for name, angle in zip(hand.joint_ids, current_angles)}
    client_positions = read_yaml(client_file)
    client_positions[f"waypoint_{time.strftime('%Y%m%d_%H%M%S')}"] = joint_dict
    save_yaml_positions(client_file, client_positions)
    logger.info(f"Positions saved to {client_file}")



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


def play_waypoints(hand, client_id, ws):
    """Play waypoints from the client's YAML file."""
    global waypoint_playback_active, blocking_slider_commands, waypoint_playback_stop_event
    def cleanup(success=True, message="Completed"):
        global waypoint_playback_active, blocking_slider_commands
        reset_to_neutral(hand)
        waypoint_playback_active = False
        blocking_slider_commands = False
        return success, message
    try:
        logger.info(f"Starting waypoint playback for client: {client_id}")
        waypoint_playback_active = True
        blocking_slider_commands = True
        client_file = get_client_file_path(client_id)
        if not os.path.exists(client_file):
            return cleanup(False, "No saved waypoints found")
        waypoints = load_waypoints(client_file, hand.joint_ids)
        logger.info(f"Loaded {len(waypoints)} waypoints")
        INTERP_TIME, STEP_TIME = 0.8, 0.02
        MAX_PLAYBACK_TIME = 120
        n_steps = int(INTERP_TIME / STEP_TIME)
        waypoint_playback_stop_event.clear()
        playback_start_time = time.time()
        while not waypoint_playback_stop_event.is_set():
            if time.time() - playback_start_time > MAX_PLAYBACK_TIME:
                logger.info(f"Waypoint playback reached maximum time limit ({MAX_PLAYBACK_TIME}s), stopping")
                ws.send(json.dumps({"action": "response_playback_stopped", "data": None}))
                return cleanup(True, "Maximum playback time reached")
            for i, start in enumerate(waypoints):
                end = waypoints[(i + 1) % len(waypoints)]
                for step in range(n_steps + 1):
                    if waypoint_playback_stop_event.is_set():
                        return cleanup(True, "Waypoint playback stopped")
                    t = step / n_steps if n_steps > 0 else 1.0
                    alpha = ease_in_out(t)
                    pose = [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
                    hand.set_joint_pos(pose, num_steps=1)
                    time.sleep(STEP_TIME)
        return cleanup(True, "Waypoint playback completed")
    except Exception as e:
        logger.exception(f"Error in waypoint playback: {str(e)}")
        return cleanup(False, str(e))


def get_waypoint_count(client_id):
    """Get the number of saved waypoints for a client."""
    try:
        client_file = get_client_file_path(client_id)
        return len(read_yaml(client_file))
    except Exception as e:
        logger.error(f"Error getting waypoint count: {e}")
        return 0


def query_reset_to_neutral(hand, data, ws):
    """Handle reset command"""
    # Check if this is a system reset (client leaving/timeout) or user reset

    if not is_authorized_client(data, ws, "user"):
        return
        
    client_id = data.get('clientId', 'unknown')
    logger.info(f"Reset command from client: {client_id}")
    
    try:
        global waypoint_playback_active, waypoint_playback_stop_event
        if waypoint_playback_active:
            logger.info("Stopping waypoint playback due to reset")
            waypoint_playback_stop_event.set()
            waypoint_playback_active = False
        else:
            reset_to_neutral(hand)

    except Exception as e:
        logger.exception("Error during reset:")
        ws.send(json.dumps({"action": "response_error", "data": f"Reset failed: {e}"}))

def query_new_active_client(hand, data, ws):
    """Handle new active client notification"""
    # This is a system command - requires client ID but bypasses active client check
    if not is_authorized_client(data, ws, "system"):
        return
        
    global current_client_id, waypoint_playback_active, waypoint_playback_stop_event
    client_id = data.get('clientId')
    
    if client_id != current_client_id:
        logger.info(f"Client changing: {current_client_id} -> {client_id}")
        
        # CRITICAL: Stop any active waypoint playback from previous client
        if waypoint_playback_active:
            logger.info("Stopping waypoint playback due to client change")
            waypoint_playback_stop_event.set()
            waypoint_playback_active = False
        else:
            reset_to_neutral(hand)
        current_client_id = client_id
    # No response needed - client state is already correct

def query_save_waypoints(hand, data, ws):
    """Handle save joints command"""
    if not is_authorized_client(data, ws, "user"):
        return
        
    try:
        save_joint_positions(hand, client_id=data.get('clientId'))
    except Exception as e:
        logger.exception("Error saving joint positions:")
        send_error(ws, f"Error saving waypoints: {e}")

def query_get_waypoint_count(_, data, ws):
    """Handle get waypoint count command"""
    if not is_authorized_client(data, ws, "user"):
        return
        
    count = get_waypoint_count(data.get("clientId", ""))
    ws.send(json.dumps({
        "action": "response_waypoint_count_updated",
        "data": {"count": count}
    }))

def query_delete_last_waypoint(_, data, ws):
    """Handle delete last waypoint command"""
    if not is_authorized_client(data, ws, "user"):
        return
    remaining_count = delete_last_waypoint(data.get("clientId", ""))
    ws.send(json.dumps({
        "action": "response_waypoint_count_updated",
        "data": {"count": remaining_count}
    }))

def query_start_waypoint_playback(hand, data, ws):
    """Handle play waypoints command"""
    if not is_authorized_client(data, ws, "user"):
        return
        
    try:
        global waypoint_playback_active, waypoint_playback_stop_event, waypoint_playback_thread
        
        # Stop any existing playback
        if waypoint_playback_active:
            waypoint_playback_stop_event.set()
            time.sleep(0.5)
        
        # Verify we have enough waypoints before proceeding
        count = get_waypoint_count(data.get("clientId", ""))
        if count < 2:
            logger.warning(f"Not enough waypoints ({count}) to start playback")
            send_error(ws, f"Need at least 2 waypoints, found {count}")
            return
        
        # Start new playback
        waypoint_playback_stop_event.clear()
        
        waypoint_playback_thread = threading.Thread(
            target=play_waypoints,
            args=(hand, data.get('clientId'), ws),
            daemon=True,
            name="WaypointPlayback"
        )
        waypoint_playback_thread.start()
        
    except Exception as e:
        waypoint_playback_active = False  # Reset flag on any error
        send_error(ws, f"Failed to start playback: {e}")


def query_stop_waypoint_playback(_, data, ws):
    """Handle stop playback command"""
    if not is_authorized_client(data, ws, "user"):
        return
        
    global waypoint_playback_active
    if waypoint_playback_active:
        waypoint_playback_stop_event.set()
        time.sleep(0.5)
        ws.send(json.dumps({"action": "response_playback_stopped", "data": None}))
    waypoint_playback_active = False


def query_slider_control(_, data, ws):
    """Handle slider command"""
    if not is_authorized_client(data, ws, "user"):
        return
        
    global blocking_slider_commands
    if not blocking_slider_commands:
        command_data = {
            "clientId": data.get("clientId"),
            "command": data.get("command", [])
        }
        logger.info(f"Setting command: {command_data}")
        set_command_buffer(command_data)


def is_authorized_client(data, ws, action_type="user"):
    authorized = (
        action_type == "system"
        or (action_type == "user" and (current_client_id is None or data.get('clientId') == current_client_id))
    )
    if not authorized:
        send_error(ws, "Unauthorized: You are not the active client")
        return False
    return True


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
                if action and action.startswith('query_') and action in globals():
                    globals()[action](hand, data, ws)
            except Exception:
                ws.send(json.dumps({"action": "response_error", "data": message}))
                continue
    except Exception as e:
        logger.error(f"Websocket error: {e}")
    except KeyboardInterrupt:
         logger.info("Shutting down due to keyboard interrupt...")
    finally:
        logger.info("Disabling torque and disconnecting hand...")
        hand.disable_torque()
        hand.disconnect()
        if ws is not None:
            ws.close()


if __name__ == "__main__":
    sys.exit(main())