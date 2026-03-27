# ORCA Tactile Sensing UI

Web-based interface for real-time tactile sensor visualization.

## Usage

```bash
python scripts/tactile_sensing_ui/tactile_ui.py
python scripts/tactile_sensing_ui/tactile_ui.py --config orca_core/models/v2/orcahand-touch/config.yaml
```

Then open your browser to `http://localhost:5001`

## Features

- **Connection Management**: Connect/disconnect to sensor devices
- **Sensor Status**: View which sensors are connected
- **Taxel Counts**: Display number of taxels for each sensor
- **Force Visualization**: Real-time force vectors displayed as arrows and numerical values
- **Taxel Visualization**: 2D taxel view with magnitude, direction, and arrow display modes
- **Zeroing**: Capture sensor baseline offsets
- **Auto Update**: Continuous monitoring of sensor data via WebSocket

## Dependencies

Install with: `pip install -e ".[sensing-ui]"`
