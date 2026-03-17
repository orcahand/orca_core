# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
import sys
import time
import json
import asyncio
from fastapi import FastAPI, HTTPException, Body, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Dict, Optional, Union, Tuple
import numpy as np
import uvicorn
from orca_core.utils.yaml_utils import read_yaml, update_yaml

from orca_core import OrcaHand

app = FastAPI(title="OrcaHand API", version="1.0.0")

# --- Event broadcasting (WS push to bridge/local listeners) ---
event_clients = set()
_event_loop = None

@app.on_event("startup")
async def _capture_event_loop():
    global _event_loop
    _event_loop = asyncio.get_running_loop()

async def broadcast_event(data):
    """Send a JSON event to all connected /ws/events clients."""
    payload = json.dumps(data)
    dead = set()
    for ws in event_clients:
        try:
            await ws.send_text(payload)
        except Exception:
            dead.add(ws)
    event_clients.difference_update(dead)

def notify_event(data):
    """Thread-safe wrapper — schedules broadcast_event on the main event loop."""
    if _event_loop is not None:
        asyncio.run_coroutine_threadsafe(broadcast_event(data), _event_loop)

# CORS middleware — allow frontend to call API directly during local dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Global OrcaHand Instance ---
# Ensure necessary config files are present or adjust path as needed

hand = OrcaHand()

# Auto-connect and calibrate on startup if hardware is available
try:
    if not hand.is_connected():
        success, msg = hand.connect()
        if success:
            print(f"Auto-connected to hand: {msg}")
            if not hand.is_calibrated():
                hand.calibrate()
                print(f"Auto-calibrated: {hand.is_calibrated()}")
        else:
            print(f"Auto-connect failed: {msg}")
except Exception as e:
    print(f"Auto-connect/calibrate skipped: {e}")

# Global waypoint storage (per-client, keyed by clientId)
waypoint_store = {}  # clientId → list of position lists

class MotorList(BaseModel):
    motor_ids: Optional[List[int]] = None

class MaxCurrent(BaseModel):
    current: Union[float, List[float]]

class JointPositions(BaseModel):
    positions: Union[Dict[str, float], List[float]] = Field(..., example={"index_flex": 0.5, "thumb_flex": 0.2})

class TensionRequest(BaseModel):
    move_motors: bool = False

class JitterRequest(BaseModel):
    amplitude: float = 5.0
    frequency: float = 10.0
    duration: float = 3.0

class ClientIdRequest(BaseModel):
    clientId: str


def handle_hand_exception(e: Exception):
    """Translates OrcaHand runtime errors to HTTP exceptions."""
    if isinstance(e, RuntimeError):
        if "not connected" in str(e).lower():
            raise HTTPException(status_code=409, detail=f"Hand operation failed: {e}")
        elif "not calibrated" in str(e).lower():
             raise HTTPException(status_code=409, detail=f"Hand operation failed: {e}")
        else:
            raise HTTPException(status_code=400, detail=f"Hand operation failed: {e}")
    elif isinstance(e, ValueError):
         raise HTTPException(status_code=422, detail=f"Invalid input: {e}")
    else:
        raise HTTPException(status_code=500, detail=f"Internal server error: {e}")


# --- API Endpoints ---
@app.post("/config", summary="Set Hand Configuration", tags=["Configuration"])
def set_hand_config(config_path: str = Body(..., example="/path/to/config")):
    """
    Sets or updates the hand configuration by recreating the OrcaHand object.

    Args:
        config_path (str): Path to the new configuration file.

    Returns:
        dict: Success message.
    """
    global hand, current_config_path
    try:
        if hand.is_connected():
            hand.disconnect()

        current_config_path = config_path
        hand = OrcaHand(model_path=current_config_path)
        return {"message": f"Hand configuration updated to: {config_path}"}
    except Exception as e:
        handle_hand_exception(e)
        
@app.post("/connect", summary="Connect to the OrcaHand", tags=["Connection"])
def connect_hand():
    """
    Establishes a connection to the OrcaHand hardware.

    Returns:
        dict: Status message indicating success or failure.
    """
    if hand.is_connected():
        return {"message": "Hand already connected."}
    try:
        success, msg = hand.connect()
        if success:
             return {"message": msg}
        else:
            raise HTTPException(status_code=500, detail=f"Connection failed: {msg}")
    except Exception as e:
        handle_hand_exception(e)

@app.post("/disconnect", summary="Disconnect from the OrcaHand", tags=["Connection"])
def disconnect_hand():
    """
    Disconnects from the OrcaHand hardware, disabling torque first.

    Returns:
        dict: Status message indicating success or failure.
    """
    if not hand.is_connected():
        return {"message": "Hand already disconnected."}
    try:
        try:
            hand.disable_torque()
            time.sleep(0.1) 
        except Exception as torque_err:
            print(f"Warning: Error disabling torque before disconnect: {torque_err}")

        success, msg = hand.disconnect()
        if success:
            return {"message": msg}
        else:
            raise HTTPException(status_code=500, detail=f"Disconnection failed: {msg}")
    except Exception as e:
         handle_hand_exception(e)


@app.get("/status", summary="Get Hand Status", tags=["Status"])
def get_status():
    """
    Retrieves the current connection and calibration status of the hand.

    Returns:
        dict: Contains 'connected' (bool) and 'calibrated' (bool) status.
    """
    try:
        return {
            "connected": hand.is_connected(),
            "calibrated": hand.is_calibrated() if hand.is_connected() else False
        }
    except Exception as e:
        handle_hand_exception(e)

@app.post("/torque/enable", summary="Enable Motor Torque", tags=["Control"])
def enable_torque(motor_list: MotorList = Body(None)):
    """
    Enables torque for specified motors or all motors if none are specified.

    Args:
        motor_list (MotorList, optional): A JSON body containing a list of motor IDs.
                                           Example: {"motor_ids": [1, 3]}
                                           If omitted or null, torque is enabled for all motors.

    Returns:
        dict: Success message.
    """
    try:
        ids = motor_list.motor_ids if motor_list else None
        hand.enable_torque(motor_ids=ids)
        return {"message": f"Torque enabled for motors: {ids or 'all'}"}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/torque/disable", summary="Disable Motor Torque", tags=["Control"])
def disable_torque(motor_list: MotorList = Body(None)):
    """
    Disables torque for specified motors or all motors if none are specified.

    Args:
        motor_list (MotorList, optional): A JSON body containing a list of motor IDs.
                                           Example: {"motor_ids": [1, 3]}
                                           If omitted or null, torque is disabled for all motors.

    Returns:
        dict: Success message.
    """
    try:
        ids = motor_list.motor_ids if motor_list else None
        hand.disable_torque(motor_ids=ids)
        return {"message": f"Torque disabled for motors: {ids or 'all'}"}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/current/max", summary="Set Maximum Motor Current", tags=["Control"])
def set_max_current(max_current: MaxCurrent):
    """
    Sets the maximum current limit for the motors.

    Args:
        max_current (MaxCurrent): A JSON body containing either a single float
                                  (applied to all motors) or a list of floats
                                  (one per motor, in order).
                                  Example single: {"current": 300.0}
                                  Example list: {"current": [300.0, 350.0, 300.0]}

    Returns:
        dict: Success message.
    """
    try:
        hand.set_max_current(current=max_current.current)
        return {"message": "Maximum current set successfully."}
    except Exception as e:
        handle_hand_exception(e)

@app.get("/motors/position", summary="Get Motor Positions", tags=["State"])
def get_motor_position():
    """
    Retrieves the current position of all motors in radians.

    Returns:
        dict: Contains a list of motor positions: {"positions": [pos1, pos2, ...]}.
              Returns null if not connected.
    """
    try:
        pos = hand.get_motor_pos()
        return {"positions": pos.tolist() if pos is not None else None}
    except Exception as e:
        handle_hand_exception(e)


@app.get("/motors/current", summary="Get Motor Currents", tags=["State"])
def get_motor_current():
    """
    Retrieves the current current draw of all motors.

    Returns:
        dict: Contains a list of motor currents: {"currents": [cur1, cur2, ...]}.
              Returns null if not connected.
    """
    try:
        cur = hand.get_motor_current()
        return {"currents": cur.tolist() if cur is not None else None}
    except Exception as e:
        handle_hand_exception(e)

@app.get("/motors/temperature", summary="Get Motor Temperatures", tags=["State"])
def get_motor_temperature():
    """
    Retrieves the current temperature of all motors.

    Returns:
        dict: Contains a list of motor temperatures: {"temperatures": [temp1, temp2, ...]}.
              Returns null if not connected.
    """
    try:
        temp = hand.get_motor_temp()
        return {"temperatures": temp.tolist() if temp is not None else None}
    except Exception as e:
        handle_hand_exception(e)

@app.get("/joints/position", summary="Get Joint Positions", tags=["State"])
def get_joint_position():
    """
    Retrieves the current position of all calibrated joints.

    Returns:
        dict: A dictionary mapping joint names to their positions:
              {"positions": {"joint1": pos1, "joint2": pos2, ...}}.
              Returns null for positions if not connected or not calibrated.
              Individual joint values might be null if that specific joint isn't calibrated yet.
    """
    try:
        j_pos = hand.get_joint_pos()
        # Convert numpy float32 values to native Python floats for JSON serialization
        if j_pos is not None:
            if isinstance(j_pos, dict):
                j_pos = {k: float(v) if v is not None else None for k, v in j_pos.items()}
            elif isinstance(j_pos, list):
                j_pos = [float(v) if v is not None else None for v in j_pos]
        return {"positions": j_pos}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/joints/position", summary="Set Joint Positions", tags=["Control"])
def set_joint_position(joint_positions: JointPositions):
    """
    Sets the desired positions for specified joints. Requires calibration.

    Args:
        joint_positions (JointPositions): A JSON body containing a dictionary
                                          mapping joint names to desired positions (in radians).
                                          Example: {"positions": {"index_flex": 1.0, "thumb_oppose": 0.5}}

    Returns:
        dict: Success message.
    """
    try:
        hand.set_joint_pos(joint_pos=joint_positions.positions)
        return {"message": "Joint positions command sent successfully."}
    except Exception as e:
        handle_hand_exception(e)

@app.get("/calibrate/status", summary="Get Calibration Status", tags=["Calibration"])
def get_calibration_status():
    """
    Checks if the hand is fully calibrated.

    Returns:
        dict: Contains 'calibrated' (bool) status.
    """
    try:
        return {"calibrated": hand.is_calibrated()}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/calibrate")
def calibrate_auto():
    """
    Starts the automatic calibration routine defined in the configuration.
    This might take some time.

    Returns:
        dict: Message indicating calibration start and eventual result.
    """
    if not hand.is_connected():
         raise HTTPException(status_code=409, detail="Hand must be connected to calibrate.")
    try:
        hand.calibrate()
        calib_status = hand.is_calibrated()
        msg = "Automatic calibration finished." + (" Successfully." if calib_status else " Failed or incomplete.")
        return {"message": msg, "calibrated": calib_status}
    except Exception as e:
        handle_hand_exception(e)
        
# @app.get("/config/settings", summary="Get Current Configuration Settings", tags=["Configuration"])
# def get_config_settings():
#     """
#     Retrieves the current configuration settings from the file.

#     Returns:
#         dict: Current configuration settings.
#     """
#     global current_config_path
#     try:
#         if not current_config_path:
#             raise HTTPException(status_code=400, detail="No configuration file is currently loaded.")
#         config_data = read_yaml(current_config_path)
#         return {"config": config_data}
#     except Exception as e:
#         handle_hand_exception(e)


# @app.put("/config/settings", summary="Update Configuration Settings", tags=["Configuration"])
# def update_config_settings(updated_settings: dict = Body(...)):
#     """
#     Updates specific settings in the configuration file and reloads the OrcaHand object.

#     Args:
#         updated_settings (dict): A dictionary containing the settings to update.

#     Returns:
#         dict: Success message.
#     """
#     global hand, current_config_path
#     try:
#         if not current_config_path:
#             raise HTTPException(status_code=400, detail="No configuration file is currently loaded.")
        
#         # Read the current configuration
#         config_data = read_yaml(current_config_path)
        
#         # Update the configuration with the new settings
#         config_data.update(updated_settings)
        
#         # Write the updated configuration back to the file
#         write_config_file(current_config_path, config_data)
        
#         # Reinitialize the OrcaHand object with the updated configuration
#         if hand.is_connected():
#             hand.disconnect()
#         hand = OrcaHand(model_path=current_config_path)
        
#         return {"message": "Configuration updated successfully.", "updated_config": config_data}
#     except Exception as e:
#         handle_hand_exception(e)

# --- Remote Control Endpoints ---

@app.get("/joints/info", summary="Get Joint Configuration Info", tags=["Remote Control"])
def get_joints_info():
    """Returns joint IDs, ranges of motion, and neutral positions from config."""
    try:
        return {
            "joint_ids": hand.joint_ids,
            "joint_roms": hand.joint_roms_dict,
            "neutral_position": hand.neutral_position,
        }
    except Exception as e:
        handle_hand_exception(e)

POSE_OVERRIDES = {
    "fist": {
        "thumb_cmc": 10, "thumb_mcp": 15, "thumb_dip": 20,
        "index_mcp": 90, "index_pip": 70,
        "middle_mcp": 90, "middle_pip": 70,
        "ring_mcp": 90, "ring_pip": 70,
        "pinky_mcp": 90, "pinky_pip": 70,
    },
    "swag": {
        "ring_mcp": 90, "ring_pip": 70,
        "middle_mcp": 90, "middle_pip": 70,
    },
    "peace": {
        "index_abd": 25,
        "ring_mcp": 90, "ring_pip": 70,
        "pinky_mcp": 90, "pinky_pip": 70,
    },
}

@app.post("/pose/{pose_name}", summary="Set a named pose", tags=["Remote Control"])
def set_pose(pose_name: str):
    """Smoothly moves the hand to a named pose (neutral + overrides)."""
    overrides = POSE_OVERRIDES.get(pose_name)
    if not overrides:
        raise HTTPException(status_code=404, detail=f"Unknown pose: {pose_name}")
    try:
        positions = dict(hand.neutral_position)
        positions.update(overrides)
        hand.set_joint_pos(positions, num_steps=25, step_size=0.001)
        return {"message": f"Pose '{pose_name}' set", "positions": positions}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/joints/zero", summary="Set Zero Position", tags=["Remote Control"])
def set_zero_position():
    """Moves the hand to the zero position."""
    try:
        hand.set_zero_position()
        return {"message": "Hand moved to zero position."}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/joints/neutral", summary="Set Neutral Position", tags=["Remote Control"])
def set_neutral_position():
    """Moves the hand to the neutral position."""
    try:
        hand.set_neutral_position()
        return {"message": "Hand moved to neutral position."}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/init", summary="Initialize Joints", tags=["Remote Control"])
def init_joints():
    """Enables torque, sets control mode, and moves to zero position."""
    try:
        hand.init_joints()
        return {"message": "Joints initialized successfully."}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/tension", summary="Apply Tension", tags=["Remote Control"])
def apply_tension(req: TensionRequest = Body(TensionRequest())):
    """Applies tension to the tendons. Runs in background (non-blocking)."""
    try:
        hand.tension(move_motors=req.move_motors, blocking=False)
        return {"message": "Tension routine started."}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/tension/stop", summary="Stop Tension", tags=["Remote Control"])
def stop_tension():
    """Stops any running background task (e.g., tension hold)."""
    try:
        hand.stop_task()
        return {"message": "Task stopped."}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/jitter", summary="Jitter Motors", tags=["Remote Control"])
def jitter_motors(req: JitterRequest = Body(JitterRequest())):
    """Vibrates motors to release tension. Runs in background (non-blocking)."""
    try:
        hand.jitter(amplitude=req.amplitude, frequency=req.frequency, duration=req.duration, blocking=False)
        return {"message": "Jitter routine started."}
    except Exception as e:
        handle_hand_exception(e)

@app.get("/task/status", summary="Get Task Status", tags=["Remote Control"])
def get_task_status():
    """Returns whether a background task is currently running."""
    try:
        running = hasattr(hand, '_task_thread') and hand._task_thread is not None and hand._task_thread.is_alive()
        return {"running": running}
    except Exception as e:
        handle_hand_exception(e)

MAX_WAYPOINTS = 10

@app.post("/waypoints/save", summary="Save current position as waypoint", tags=["Waypoints"])
def save_waypoint(req: ClientIdRequest):
    """Reads current joint positions and appends to the client's waypoint list."""
    try:
        wps = waypoint_store.setdefault(req.clientId, [])
        if len(wps) >= MAX_WAYPOINTS:
            raise HTTPException(status_code=400, detail=f"Maximum {MAX_WAYPOINTS} waypoints reached")
        pos = hand.get_joint_pos(as_list=True)
        wps.append(pos)
        return {"count": len(wps)}
    except HTTPException:
        raise
    except Exception as e:
        handle_hand_exception(e)

@app.delete("/waypoints/last", summary="Delete last waypoint", tags=["Waypoints"])
def delete_last_waypoint(req: ClientIdRequest):
    """Removes the last saved waypoint for the client."""
    wps = waypoint_store.get(req.clientId, [])
    if wps:
        wps.pop()
    return {"count": len(wps)}

@app.get("/waypoints/count", summary="Get waypoint count", tags=["Waypoints"])
def get_waypoint_count(clientId: str):
    """Returns the number of saved waypoints for the client."""
    count = len(waypoint_store.get(clientId, []))
    return {"count": count}

@app.post("/waypoints/play", summary="Start waypoint playback", tags=["Waypoints"])
def play_waypoints(req: ClientIdRequest):
    """Smoothly moves to neutral, then replays saved waypoints in a background thread."""
    wps = waypoint_store.get(req.clientId, [])
    if len(wps) < 2:
        raise HTTPException(status_code=400, detail="Need at least 2 waypoints to play")
    try:
        client_id = req.clientId

        def playback_sequence():
            from orca_core.utils.utils import interpolate_waypoints
            # Phase 1: smoothly move to neutral position
            hand.set_neutral_position()
            if hand._task_stop_event.is_set():
                return
            # Phase 2: smoothly interpolate from neutral to first waypoint
            neutral = [hand.neutral_position[jid] for jid in hand.joint_ids]
            for position in interpolate_waypoints(neutral, wps[0], 0.5, 0.02, "ease_in_out"):
                if hand._task_stop_event.is_set():
                    return
                hand.set_joint_pos(position)
                time.sleep(0.02)
            if hand._task_stop_event.is_set():
                return
            # Phase 3: notify frontend that preparation is done, waypoint replay starting
            notify_event({"event": "playback_started", "clientId": client_id})
            # Phase 4: replay waypoints loop (on_finish fires in finally block)
            hand._replay_waypoints(wps, 0.5, 0.02, 1000, "ease_in_out",
                on_finish=lambda: notify_event({"event": "playback_finished", "clientId": client_id}))

        hand._start_task(playback_sequence)
        return {"message": "Playback started", "waypoint_count": len(wps)}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/waypoints/stop", summary="Stop waypoint playback", tags=["Waypoints"])
def stop_waypoints(req: ClientIdRequest):
    """Stops any running waypoint playback."""
    try:
        hand.stop_task()
        return {"message": "Playback stopped"}
    except Exception as e:
        handle_hand_exception(e)

@app.delete("/waypoints/clear", summary="Clear all waypoints", tags=["Waypoints"])
def clear_waypoints(req: ClientIdRequest):
    """Clears all waypoints for the client."""
    waypoint_store.pop(req.clientId, None)
    return {"message": "Waypoints cleared"}

@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    """WebSocket endpoint for low-latency real-time joint control."""
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            positions = msg.get("positions")
            if positions and isinstance(positions, dict):
                try:
                    hand.set_joint_pos(joint_pos=positions)
                    await websocket.send_text(json.dumps({"status": "ok"}))
                except Exception as e:
                    await websocket.send_text(json.dumps({"status": "error", "detail": str(e)}))
            else:
                await websocket.send_text(json.dumps({"status": "error", "detail": "Invalid positions format"}))
    except WebSocketDisconnect:
        pass
    except Exception as e:
        try:
            await websocket.send_text(json.dumps({"status": "error", "detail": str(e)}))
        except:
            pass

@app.websocket("/ws/events")
async def websocket_events(websocket: WebSocket):
    """WebSocket endpoint for push events (e.g. playback_finished)."""
    await websocket.accept()
    event_clients.add(websocket)
    try:
        while True:
            await websocket.receive_text()  # keep alive; ignore incoming
    except WebSocketDisconnect:
        pass
    finally:
        event_clients.discard(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
