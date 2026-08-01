# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import threading
import time
from contextlib import contextmanager
from typing import Dict, List, Optional, Union

from fastapi import FastAPI, HTTPException, Body
from pydantic import BaseModel, Field
import uvicorn

from orca_core import OrcaHand

app = FastAPI(title="OrcaHand API", version="1.0.0")

# Created lazily on first use so importing this module has no side effects.
hand: Optional[OrcaHand] = None

_hand_init_lock = threading.Lock()

# Serializes /connect, /disconnect, /config, and /calibrate against each other
# and in-flight operations; read endpoints stay lock-free and responsive.
_hand_lock = threading.Lock()


def _get_hand() -> OrcaHand:
    global hand
    if hand is None:
        with _hand_init_lock:
            if hand is None:
                hand = OrcaHand()
    return hand


@contextmanager
def _exclusive_hand_operation():
    """Hold the lifecycle lock for the request; reject overlap with a 409
    instead of queueing behind a possibly long-running operation."""
    if not _hand_lock.acquire(blocking=False):
        raise HTTPException(
            status_code=409,
            detail="Another hand operation (connect/disconnect/config/calibrate) is in progress.",
        )
    try:
        yield
    finally:
        _hand_lock.release()


def _require_connected(h: OrcaHand) -> None:
    if not h.is_connected():
        raise HTTPException(status_code=409, detail="Hand is not connected.")


_TASK_STOP_TIMEOUT_S = 10.0


class MotorList(BaseModel):
    motor_ids: Optional[List[int]] = None

class MaxCurrent(BaseModel):
    current: Union[float, List[float]]

class JointPositions(BaseModel):
    positions: Dict[str, float] = Field(
        ..., json_schema_extra={"example": {"index_flex": 0.5, "thumb_flex": 0.2}}
    )


def handle_hand_exception(e: Exception):
    """Translates OrcaHand runtime errors to HTTP exceptions."""
    if isinstance(e, HTTPException):
        raise e
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
def set_hand_config(
    config_path: str = Body(..., json_schema_extra={"example": "/path/to/config"})
):
    """
    Sets or updates the hand configuration by recreating the OrcaHand object.

    Args:
        config_path (str): Path to the new configuration file.

    Returns:
        dict: Success message.
    """
    global hand
    with _exclusive_hand_operation():
        try:
            # Don't build the default hand just to replace it: only an
            # already-constructed, connected hand needs disconnecting.
            if hand is not None and hand.is_connected():
                if not hand.stop_task(_TASK_STOP_TIMEOUT_S):
                    raise HTTPException(
                        status_code=500,
                        detail="A running task (e.g. calibration) did not stop; "
                        "configuration unchanged.",
                    )
                hand.disconnect()

            hand = OrcaHand(config_path=config_path)
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
    with _exclusive_hand_operation():
        h = _get_hand()
        if h.is_connected():
            return {"message": "Hand already connected."}
        try:
            # Headless server: fail cleanly instead of opening the terminal picker.
            success, msg = h.connect(interactive=False)
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

    A running background task (e.g. a calibration started via POST
    /calibrate) is signalled to stop and awaited before the disconnect, so
    this endpoint doubles as the remote abort.

    Returns:
        dict: Status message indicating success or failure.
    """
    with _exclusive_hand_operation():
        h = _get_hand()
        if not h.is_connected():
            return {"message": "Hand already disconnected."}
        try:
            if not h.stop_task(_TASK_STOP_TIMEOUT_S):
                raise HTTPException(
                    status_code=500,
                    detail="A running task (e.g. calibration) did not stop; "
                    "hand left connected.",
                )
            try:
                h.disable_torque()
                time.sleep(0.1)
            except Exception as torque_err:
                print(f"Warning: Error disabling torque before disconnect: {torque_err}")

            success, msg = h.disconnect()
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
        h = _get_hand()
        # Same source as GET /calibrate/status: calibration is persisted
        # state, meaningful whether or not the hand is currently connected.
        return {
            "connected": h.is_connected(),
            "calibrated": h.is_calibrated(),
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
        h = _get_hand()
        _require_connected(h)
        ids = motor_list.motor_ids if motor_list else None
        h.enable_torque(motor_ids=ids)
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
        h = _get_hand()
        _require_connected(h)
        ids = motor_list.motor_ids if motor_list else None
        h.disable_torque(motor_ids=ids)
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
        h = _get_hand()
        _require_connected(h)
        h.set_max_current(current=max_current.current)
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
        h = _get_hand()
        if not h.is_connected():
            return {"positions": None}
        pos = h.get_motor_pos()
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
        h = _get_hand()
        if not h.is_connected():
            return {"currents": None}
        cur = h.get_motor_current()
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
        h = _get_hand()
        if not h.is_connected():
            return {"temperatures": None}
        temp = h.get_motor_temp()
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
              Joints without a calibrated mapping are omitted.
              Responds 409 if the hand is not connected.
    """
    try:
        h = _get_hand()
        _require_connected(h)
        return {"positions": h.get_joint_position().as_dict()}
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
        h = _get_hand()
        _require_connected(h)
        h.set_joint_positions(joint_positions.positions)
        return {"message": "Joint positions command sent successfully."}
    except Exception as e:
        handle_hand_exception(e)

@app.get("/calibrate/status", summary="Get Calibration Status", tags=["Calibration"])
def get_calibration_status():
    """
    Reports whether the hand is fully calibrated and whether a background
    calibration is currently running.

    Returns:
        dict: Contains 'calibrated' (bool) and 'running' (bool) status.
    """
    try:
        h = _get_hand()
        return {"calibrated": h.is_calibrated(), "running": h.task_running}
    except Exception as e:
        handle_hand_exception(e)

@app.post("/calibrate", status_code=202)
def calibrate_auto():
    """
    Starts the automatic calibration routine in the background and returns
    immediately. Poll GET /calibrate/status for progress; POST /disconnect
    aborts a running calibration.

    Returns:
        dict: Message indicating the calibration was started.
    """
    # The lock covers only the start of the task, so /disconnect can still
    # abort a calibration that runs for minutes.
    with _exclusive_hand_operation():
        h = _get_hand()
        if not h.is_connected():
             raise HTTPException(status_code=409, detail="Hand must be connected to calibrate.")
        if h.task_running:
            raise HTTPException(
                status_code=409, detail="A hand task is already running."
            )
        try:
            h.calibrate(blocking=False)
        except Exception as e:
            handle_hand_exception(e)
    return {"message": "Calibration started. Poll /calibrate/status for progress."}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
