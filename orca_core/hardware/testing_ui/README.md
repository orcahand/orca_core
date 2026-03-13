# ORCA Sensor Testing UI

Web-based testing interface for ORCA tactile sensors.

## Installation

```bash
pip install -r requirements.txt
```

## Usage

```bash
python app.py
```

Then open your browser to `http://localhost:5000`

## Features

- **Connection Management**: Connect/disconnect to sensor devices
- **Sensor Status**: View which sensors are connected
- **Taxel Counts**: Display number of taxels for each sensor
- **Force Visualization**: Real-time force vectors displayed as arrows and numerical values
- **Auto Update**: Continuous monitoring of sensor data
- **Hardware Info**: Display hardware version and auto data type settings

## Controls

- **Port Input**: Specify the serial port (default: `/dev/ttyACM0`)
- **Connect**: Establish connection to the sensor device
- **Disconnect**: Close the connection
- **Refresh**: Manually refresh sensor data
- **Auto Update**: Enable/disable automatic data refresh (100ms interval)

