# ORCA Core

The core package for ORCA Hand control and functionality.

## Overview

The ORCA Core package provides the fundamental functionality for controlling and interacting with the ORCA Hand.

## Software Stack Overview

### Core

- **Role:** Main control for the ORCA Hand. Abstracts the low-level hardware control of the robot's hand by providing simple methods for connecting, calibrating, and commanding the hand via joint positions
- **Features:**
  - Calibration routines.
  - Accepts joint angle commands and control motors respectively.
  - Retargeting for motion mapping.
  - Reads data from the servo motors. 
- **Language:** Python.

### Models

- **Contents:**
  - URDF/MJCO files for simulation.
  - CAD files and configuration data.

### ROS Wrappers

- **orca_ros1:**  
  Wrapper for ROS1 environments.
- **orca_ros2:**  
  Wrapper for ROS2 environments.

Both wrappers offer example scripts to kickstart your integration.

## Features

- Motor control
- Sensor integration
- Communication protocols
- Configuration management

---