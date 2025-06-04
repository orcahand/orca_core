import os
import sys
import time
from fastapi import FastAPI, HTTPException, Body
from pydantic import BaseModel, Field
from typing import List, Dict, Optional, Union, Tuple
import numpy as np
import uvicorn
from orca_core.utils.yaml_utils import read_yaml, update_yaml

from orca_core import OrcaHand

app = FastAPI(title="OrcaHand API", version="1.0.0")

# --- Global OrcaHand Instance ---
# Ensure necessary config files are present or adjust path as needed

hand = OrcaHand()

class MotorList(BaseModel):
    motor_ids: Optional[List[int]] = None

class MaxCurrent(BaseModel):
    current: Union[float, List[float]]

class JointPositions(BaseModel):
    positions: Dict[str, float] = Field(..., example={"index_flex": 0.5, "thumb_flex": 0.2})


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

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
