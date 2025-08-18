#!/usr/bin/env python3
import argparse
import sys
import os
import json
import time
import websocket
import logging
import threading
import yaml # type: ignore

# Add the parent directory to the Python path so we can import orca_core
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand, MockOrcaHand
from replay_angles import ease_in_out

###############################################################################################

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('remote_control')

# Define joint names once - used throughout the script
JOINT_NAMES = [
    "thumb_mcp", "thumb_abd", "thumb_pip", "thumb_dip",
    "index_abd", "index_mcp", "index_pip",
    "middle_abd", "middle_mcp", "middle_pip",
    "ring_abd", "ring_mcp", "ring_pip",
    "pinky_abd", "pinky_mcp", "pinky_pip",
    "wrist"
]

# Global state variables
current_client_id = None
latest_command = None
command_lock = threading.Lock()
command_available = threading.Event()
blocking_slider_commands = False
waypoint_playback_active = False
waypoint_playback_thread = None
waypoint_playback_stop_event = threading.Event()

def get_and_clear_command_buffer():
    """Atomically get the latest command from the buffer and clear the buffer"""
    global latest_command
    with command_lock:
        cmd = latest_command
        latest_command = None
        command_available.clear()
        return cmd
    
def set_command_buffer(new_command):
    """Fill the command buffer with a new command array"""
    global latest_command
    with command_lock:
        latest_command = new_command
        command_available.set()    

def command_processor_thread(hand):
    """Thread that processes the latest command from the buffer"""
    global current_client_id, blocking_slider_commands
    
    while True:
        try:
            # Wait for a command to become available, skip if blocking
            if not command_available.wait(timeout=1.0) or blocking_slider_commands:
                continue
            
            # Get and clear command atomically
            cmd = get_and_clear_command_buffer()
            if not cmd:
                continue
            
            # Block commands from non-active clients
            if current_client_id and cmd.get("clientId") != current_client_id:
                continue
            
            joint_dict = {name: float(value) for name, value in zip(JOINT_NAMES, cmd.get("command", []))}
            logger.info(f"Setting joint positions: {joint_dict}")
            hand.set_joint_pos(joint_pos=joint_dict, num_steps=1)

        except Exception as e:
            logger.error(f"Command processor error: {e}")
            time.sleep(0.1)


def reset_to_neutral(hand):
    """Reset the hand to neutral position without toggling torque"""
    global blocking_slider_commands
    
    blocking_slider_commands = True
    get_and_clear_command_buffer()  # Clear any pending commands
    
    try:
        logger.info("Starting neutral position reset...")
        return hand.set_neutral_position(num_steps=25, step_size=0.001)
    except Exception as e:
        logger.exception(f"Error during reset: {e}")
        return False
    finally:
        blocking_slider_commands = False


def save_joint_positions(hand, client_id=None):
    """Save the current joint positions to a client-specific YAML file"""
    current_angles = hand.get_joint_pos(as_list=True)
    client_file = _get_client_file_path(client_id)
    joint_dict = {name: float(angle) for name, angle in zip(JOINT_NAMES, current_angles)}
    
    # Load existing positions
    client_positions = {}
    if os.path.exists(client_file):
        try:
            with open(client_file, "r") as file:
                client_positions = yaml.safe_load(file) or {}
        except Exception as e:
            logger.warning(f"Could not load existing positions: {e}")

    client_positions[f"waypoint_{time.strftime('%Y%m%d_%H%M%S')}"] = joint_dict

    # Save to file
    with open(client_file, "w") as file:
        yaml.dump(client_positions, file, default_flow_style=False)

    logger.info(f"Positions saved to {client_file}")


def delete_last_waypoint(client_id=None):
    """Delete the last (most recent) waypoint from a client's YAML file"""
    client_file = _get_client_file_path(client_id)
    
    if not os.path.exists(client_file):
        logger.warning(f"No waypoints file found for client: {client_file}")
        return 0
    
    try:
        # Load existing positions
        with open(client_file, "r") as file:
            client_positions = yaml.safe_load(file) or {}
        
        if not client_positions:
            logger.warning(f"No waypoints found in file: {client_file}")
            return 0
        
        # Find the most recent waypoint (by timestamp in key name)
        # Sort by the timestamp part of the key (waypoint_YYYYMMDD_HHMMSS)
        sorted_keys = sorted(client_positions.keys(), key=lambda x: x.split('_')[1:3] if len(x.split('_')) >= 3 else ['0', '0'])
        
        if sorted_keys:
            last_key = sorted_keys[-1]
            del client_positions[last_key]
            logger.info(f"Deleted waypoint: {last_key}")
            
            # Save updated positions back to file
            if client_positions:
                with open(client_file, "w") as file:
                    yaml.dump(client_positions, file, default_flow_style=False)
            else:
                # If no waypoints left, delete the file
                os.remove(client_file)
                logger.info(f"No waypoints remaining, deleted file: {client_file}")
            
            return len(client_positions)
        else:
            logger.warning(f"No waypoints found to delete in: {client_file}")
            return 0
            
    except Exception as e:
        logger.exception(f"Error deleting waypoint from {client_file}:")
        raise e


def _get_client_file_path(client_id):
    """Get the file path for a client's waypoints"""
    client_short = client_id[:8]
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'saved_positions', f"client_{client_short}.yaml")

def _load_waypoints(client_file):
    """Load and validate waypoints from file"""
    with open(client_file, "r") as file:
        positions = yaml.safe_load(file) or {}
    
    if len(positions) < 2:
        raise ValueError(f"Need at least 2 waypoints, found {len(positions)}")
    
    # Convert to list of arrays using dict comprehension
    waypoints = [[joint_dict.get(name, 0.0) for name in JOINT_NAMES] for joint_dict in positions.values()]
    return waypoints

def play_waypoints(hand, client_id, ws):
    """Play waypoints from the client's YAML file"""
    global waypoint_playback_active, blocking_slider_commands, waypoint_playback_stop_event, current_client_id
    
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
        
        client_file = _get_client_file_path(client_id)
        
        if not os.path.exists(client_file):
            return cleanup(False, "No saved waypoints found")
        
        waypoints = _load_waypoints(client_file)
        logger.info(f"Loaded {len(waypoints)} waypoints")
        
        # Playback constants
        INTERP_TIME, STEP_TIME = 0.8, 0.02
        MAX_PLAYBACK_TIME = 120  # 2 minutes maximum playback time
        n_steps = int(INTERP_TIME / STEP_TIME)
        
        waypoint_playback_stop_event.clear()
        playback_start_time = time.time()
        
        while not waypoint_playback_stop_event.is_set():
            # Check maximum playback time (2 minutes)
            if time.time() - playback_start_time > MAX_PLAYBACK_TIME:
                logger.info(f"Waypoint playback reached maximum time limit ({MAX_PLAYBACK_TIME}s), stopping")
                ws.send(json.dumps({"action": "response_playback_stopped", "data": None}))
                return cleanup(True, "Maximum playback time reached")
            
            for i, start in enumerate(waypoints):
                end = waypoints[(i + 1) % len(waypoints)]
                
                for step in range(n_steps + 1):
                    # Check stop event
                    if waypoint_playback_stop_event.is_set():
                        return cleanup(True, "Waypoint playback stopped")
                    logger.info("Processing waypoint step")
                    
                    t = step / n_steps if n_steps > 0 else 1.0
                    alpha = ease_in_out(t)
                    pose = [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
                    joint_dict = {name: value for name, value in zip(JOINT_NAMES, pose)}
                    
                    hand.set_joint_pos(joint_pos=joint_dict, num_steps=1)
                    time.sleep(STEP_TIME)
        
        return cleanup(True, "Waypoint playback completed")
        
    except Exception as e:
        logger.exception(f"Error in waypoint playback: {str(e)}")
        return cleanup(False, str(e))

def get_waypoint_count(client_id):
    """Get the number of saved waypoints for a client"""
    try:
        client_file = _get_client_file_path(client_id)
        if not os.path.exists(client_file):
            return 0
        
        with open(client_file, "r") as file:
            positions = yaml.safe_load(file) or {}
            return len(positions)
    except Exception as e:
        logger.error(f"Error getting waypoint count: {e}")
        return 0


def handle_reset_action(hand, data, ws):
    """Handle reset command"""
    # Check if this is a system reset (client leaving/timeout) or user reset
    reason = data.get("reason", "")
    if reason in ["client_left", "client_timeout"]:
        # System resets are always allowed - don't check authorization
        pass
    elif not is_authorized_client(data, "user"):
        # User-initiated resets require authorization
        send_unauthorized_response(ws, data)
        return
        
    client_id = data.get('clientId', 'unknown')
    logger.info(f"Reset command from client: {client_id}, reason: {reason or 'user_initiated'}")
    
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

def handle_new_active_client(hand, data, ws):
    """Handle new active client notification"""
    # This is a system command - requires client ID but bypasses active client check
    if not is_authorized_client(data, "system"):
        send_unauthorized_response(ws, data)
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

def handle_save_joints(hand, data, ws):
    """Handle save joints command"""
    if not is_authorized_client(data, "user"):
        send_unauthorized_response(ws, data)
        return
        
    try:
        save_joint_positions(hand, client_id=data.get('clientId'))
    except Exception as e:
        logger.exception("Error saving joint positions:")
        ws.send(json.dumps({"action": "response_error", "data": f"Error saving: {e}"}))

def handle_get_waypoint_count(hand, data, ws):
    """Handle get waypoint count command"""
    if not is_authorized_client(data, "user"):
        send_unauthorized_response(ws, data)
        return
        
    count = get_waypoint_count(data.get("clientId", ""))
    ws.send(json.dumps({
        "action": "response_waypoint_count_updated",
        "data": {"count": count}
    }))

def handle_delete_last_waypoint(hand, data, ws):
    """Handle delete last waypoint command"""
    if not is_authorized_client(data, "user"):
        send_unauthorized_response(ws, data)
        return
        
    try:
        remaining_count = delete_last_waypoint(data.get("clientId", ""))
        ws.send(json.dumps({
            "action": "response_waypoint_count_updated",
            "data": {"count": remaining_count}
        }))
    except Exception as e:
        logger.exception("Error deleting waypoint:")
        ws.send(json.dumps({"action": "response_error", "data": f"Error deleting waypoint: {e}"}))

def handle_play_waypoints(hand, data, ws):
    """Handle play waypoints command"""
    if not is_authorized_client(data, "user"):
        send_unauthorized_response(ws, data)
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
            ws.send(json.dumps({"action": "response_error", "data": f"Need at least 2 waypoints, found {count}"}))
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
        ws.send(json.dumps({"action": "response_error", "data": f"Failed to start playback: {e}"}))

def handle_stop_playback(hand, data, ws):
    """Handle stop playback command"""
    if not is_authorized_client(data, "user"):
        send_unauthorized_response(ws, data)
        return
        
    global waypoint_playback_active
    try:
        # Stop playback and clear commands
        if waypoint_playback_active:
            waypoint_playback_stop_event.set()
            time.sleep(0.5)
            ws.send(json.dumps({"action": "response_playback_stopped", "data": None}))
        # No message needed if playback wasn't active - frontend should know the state
    finally:
        waypoint_playback_active = False

def handle_command(hand, data, ws):
    """Handle slider command"""
    if not is_authorized_client(data, "user"):
        send_unauthorized_response(ws, data)
        return
        
    global blocking_slider_commands
    if not blocking_slider_commands:
        # Extract the command array from the data and pass it with clientId
        command_data = {
            "clientId": data.get("clientId"),
            "command": data.get("command", [])
        }
        set_command_buffer(command_data)
        # No response needed for real-time commands - silent success


def is_authorized_client(data, action_type="user"):
    """Check if the client is authorized to perform actions"""
    client_id = data.get('clientId')
    
    # All commands must have a client ID
    if not client_id:
        return False
    
    # System commands (like newActiveClient) only need a valid client ID
    if action_type == "system":
        return True
        
    # User commands require being the active client
    return current_client_id is None or client_id == current_client_id

def send_unauthorized_response(ws, data=None):
    """Send unauthorized response to client"""
    client_id = data.get('clientId', 'unknown') if data else 'unknown'
    action = data.get('action', 'unknown') if data else 'unknown'
    logger.warning(f"Unauthorized attempt: client {client_id} tried action '{action}' (active: {current_client_id})")
    
    ws.send(json.dumps({
        "action": "response_error", 
        "data": "Unauthorized: You are not the active client"
    }))

# Action dispatch table
ACTION_HANDLERS = {
    "query_reset_to_neutral": handle_reset_action,
    "query_new_active_client": handle_new_active_client,
    "query_save_waypoints": handle_save_joints,
    "query_get_waypoint_count": handle_get_waypoint_count,
    "query_delete_last_waypoint": handle_delete_last_waypoint,
    "query_start_waypoint_playback": handle_play_waypoints,
    "query_stop_waypoint_playback": handle_stop_playback,
    "query_slider_control": handle_command,
}

def main():
    parser = argparse.ArgumentParser(
        description='Remote control for OrcaHand via WebSocket.',
        epilog='Examples:\n'
               '  # Run with real robot:\n'
               '  python remote_control.py /path/to/orcahand_model\n\n'
               '  # Run in mock mode for development:\n'
               '  python remote_control.py --mock\n\n'
               '  # Run in mock mode with debug logging:\n'
               '  python remote_control.py --mock --debug',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1). Not required in mock mode."
    )
    parser.add_argument(
        "--server",
        type=str,
        default="ws://localhost:8082/ws/orca-backend",
        help="WebSocket server URL"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run in mock mode (no physical robot required)"
    )
    
    args = parser.parse_args()
    
    # Set debug level if requested
    if args.debug:
        logger.setLevel(logging.DEBUG)
        websocket.enableTrace(True)

    try:
        # Initialize and connect to hand
        hand = MockOrcaHand(model_path=args.model_path) if args.mock else OrcaHand(model_path=args.model_path)
        logger.info(f"Starting {'MOCK' if args.mock else 'REAL'} mode")
        
        success, message = hand.connect()
        if not success:
            logger.error(f"Failed to connect: {message}")
            return 1
        
        hand.enable_torque()
        hand.set_neutral_position(num_steps=25, step_size=0.001)
        logger.info("Hand initialized and ready")

        # Start command processor thread
        processor_thread = threading.Thread(target=command_processor_thread, args=(hand,), daemon=True)
        processor_thread.start()
        
        try:
            ws = websocket.create_connection(args.server)
            logger.info("Connected to server, ready to receive commands")
            
            while True:
                try:
                    message = ws.recv()
                    try:
                        data = json.loads(message)
                        
                        if "action" in data:
                            action = data["action"]
                            if action in ACTION_HANDLERS:
                                # All action handlers now take (hand, data, ws) consistently
                                ACTION_HANDLERS[action](hand, data, ws)
                            else:
                                logger.debug(f"Unknown action: {action}")
                        else:
                            logger.debug(f"Received message without action: {data}")
                            
                    except json.JSONDecodeError:
                        logger.error(f"Failed to parse message as JSON: {message}")
                        # Send error to frontend about JSON parsing
                        ws.send(json.dumps({
                            "action": "response_error",
                            "data": "Failed to parse command from server"
                        }))
                    
                except websocket.WebSocketConnectionClosedException:
                    logger.error("WebSocket connection closed unexpectedly")
                    break
                except Exception as e:
                    logger.error(f"Error receiving/processing message: {str(e)}")
                    # Send error to frontend about communication issues
                    try:
                        ws.send(json.dumps({
                            "action": "response_error", 
                            "data": f"Communication error: {str(e)}"
                        }))
                    except:
                        break  # If we can't send error, connection is broken
                    
        except websocket.WebSocketException as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            if 'ws' in locals():
                ws.close()
        
        # Cleanup
        hand.disable_torque()
        hand.disconnect()
        logger.info("Shutdown complete")
        
        return 0
        
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        if 'hand' in locals():
            hand.disable_torque()
            hand.disconnect()
    except Exception as e:
        logger.error(f"Error: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())