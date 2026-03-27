#!/usr/bin/env python3
"""Web-based testing UI for ORCA Sensor Client"""

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core.hardware.sensing.sensor_client import SensorClient
from orca_core.hardware.sensing.taxel_coordinates import get_all_coordinates
from orca_core.utils.utils import read_yaml, update_yaml
import argparse
import yaml
import serial.tools.list_ports
import threading
import time

SENSOR_ADAPTER_VID = 0x28E9
SENSOR_ADAPTER_PID = 0x018A

app = Flask(__name__)
app.config['SECRET_KEY'] = 'orca_sensor_secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

sensor_client = None
stream_thread = None
stream_thread_running = False
current_mode = 'resultant'  # 'resultant', 'taxels', or 'combined'
finger_to_sensor_id_config = None  # Loaded from --config if provided
config_dir = None  # Set from --config arg directory, for calibration.yaml access

def get_sensor_client():
    global sensor_client
    if sensor_client is None:
        port = request.args.get('port', '/dev/ttyACM0')
        sensor_client = SensorClient(port=port)
    return sensor_client

def stream_update_loop():
    """Background thread that reads from auto-stream and emits via websocket."""
    global stream_thread_running, sensor_client, current_mode

    while stream_thread_running:
        try:
            if sensor_client and sensor_client.is_connected:
                if current_mode == 'resultant':
                    forces, ts = sensor_client.get_auto_latest()
                    if forces is not None:
                        socketio.emit('force_update', forces)
                elif current_mode == 'taxels':
                    taxels, ts = sensor_client.get_auto_latest_taxels()
                    if taxels is not None:
                        socketio.emit('taxel_update', taxels)
                elif current_mode == 'combined':
                    forces, taxels, ts = sensor_client.get_auto_latest_all()
                    if forces is not None or taxels is not None:
                        socketio.emit('combined_update', {
                            'forces': forces,
                            'taxels': taxels
                        })
            time.sleep(0.01)  # ~100Hz update rate
        except Exception as e:
            socketio.emit('error', {'message': str(e)})
            time.sleep(0.1)

def start_stream(mode):
    """Start auto-stream with specified mode."""
    global sensor_client, stream_thread, stream_thread_running, current_mode

    # Stop existing stream
    stop_stream()

    current_mode = mode

    # Configure and start auto-stream
    if mode == 'resultant':
        sensor_client.start_auto_stream(resultant=True, taxels=False)
    elif mode == 'taxels':
        sensor_client.start_auto_stream(resultant=False, taxels=True)
    elif mode == 'combined':
        sensor_client.start_auto_stream(resultant=True, taxels=True)

    # Start websocket emission thread
    stream_thread_running = True
    stream_thread = threading.Thread(target=stream_update_loop, daemon=True)
    stream_thread.start()

def stop_stream():
    """Stop auto-stream and emission thread."""
    global sensor_client, stream_thread, stream_thread_running

    stream_thread_running = False
    if stream_thread:
        stream_thread.join(timeout=1)
        stream_thread = None

    if sensor_client and sensor_client.is_connected:
        try:
            sensor_client.stop_auto_stream()
        except Exception:
            pass

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/ports')
def list_ports():
    ports = serial.tools.list_ports.comports()
    result = []
    for p in ports:
        is_sensor = (p.vid == SENSOR_ADAPTER_VID and p.pid == SENSOR_ADAPTER_PID)
        if p.vid is not None:
            result.append({
                'device': p.device,
                'description': p.description,
                'is_sensor_adapter': is_sensor,
            })
    result.sort(key=lambda x: (not x['is_sensor_adapter'], x['device']))
    return jsonify(result)

@app.route('/api/connect', methods=['POST'])
def connect():
    try:
        data = request.json
        port = data.get('port', '/dev/ttyACM0')
        mode = data.get('mode', 'resultant')
        global sensor_client, current_mode

        if sensor_client and sensor_client.is_connected:
            stop_stream()
            sensor_client.disconnect()

        sensor_client = SensorClient(port=port, finger_to_sensor_id=finger_to_sensor_id_config)
        sensor_client.connect()

        # Load saved sensor offsets if config was provided
        if config_dir:
            calib_path = os.path.join(config_dir, 'calibration.yaml')
            calib_data = read_yaml(calib_path)
            if calib_data and 'sensor_offsets' in calib_data:
                sensor_client.set_taxel_offsets(calib_data['sensor_offsets'])

        # Start streaming with requested mode
        start_stream(mode)

        # Get configuration for response
        config = sensor_client.get_sensor_configuration()

        socketio.emit('connection_status', {'connected': True, 'mode': mode})
        return jsonify({
            'success': True,
            'message': f'Connected to {port}',
            'mode': mode,
            'config': {
                'active_sensors': config.active_sensors if config else [],
                'num_taxels': config.num_taxels if config else {}
            }
        })
    except Exception as e:
        socketio.emit('connection_status', {'connected': False, 'error': str(e)})
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    try:
        global sensor_client
        stop_stream()
        if sensor_client and sensor_client.is_connected:
            sensor_client.disconnect()
        socketio.emit('connection_status', {'connected': False})
        return jsonify({'success': True, 'message': 'Disconnected'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/api/mode', methods=['POST'])
def set_mode():
    """Change the streaming mode."""
    try:
        global sensor_client, current_mode
        data = request.json
        mode = data.get('mode', 'resultant')

        if mode not in ('resultant', 'taxels', 'combined'):
            return jsonify({'success': False, 'message': f'Invalid mode: {mode}'}), 400

        if not sensor_client or not sensor_client.is_connected:
            return jsonify({'success': False, 'message': 'Not connected'}), 400

        start_stream(mode)
        socketio.emit('mode_changed', {'mode': mode})
        return jsonify({'success': True, 'mode': mode})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/api/zero', methods=['POST'])
def zero():
    """Capture current sensor readings as zero baseline."""
    try:
        global sensor_client
        if not sensor_client or not sensor_client.is_connected:
            return jsonify({'success': False, 'message': 'Not connected'}), 400

        offsets = sensor_client.capture_taxel_offsets(num_samples=100)

        if config_dir:
            calib_path = os.path.join(config_dir, 'calibration.yaml')
            update_yaml(calib_path, 'sensor_offsets', offsets)

        return jsonify({'success': True, 'message': 'Sensor offsets captured and applied'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/api/clear_zero', methods=['POST'])
def clear_zero():
    """Clear sensor zeroing offsets."""
    try:
        global sensor_client
        if not sensor_client or not sensor_client.is_connected:
            return jsonify({'success': False, 'message': 'Not connected'}), 400

        sensor_client.clear_taxel_offsets()
        return jsonify({'success': True, 'message': 'Sensor offsets cleared'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/api/status')
def status():
    client = get_sensor_client()
    if not client.is_connected:
        return jsonify({'connected': False})

    out = {'connected': True, 'mode': current_mode}
    errors = []

    try:
        out['hardware_version'] = client.read_hardware_version()
    except Exception as e:
        errors.append(f"read_hardware_version: {e}")

    try:
        out['sensors'] = client.read_connected_sensors()
    except Exception as e:
        errors.append(f"read_connected_sensors: {e}")

    try:
        out['taxels'] = client.read_num_taxels()
    except Exception as e:
        errors.append(f"read_num_taxels: {e}")

    try:
        out['auto_data_type'] = client.read_auto_data_type()
    except Exception as e:
        errors.append(f"read_auto_data_type: {e}")

    # Get stream stats
    try:
        stats = client.get_auto_stats()
        out['stream_stats'] = {
            'frames_ok': stats.frames_ok,
            'parse_ok': stats.parse_ok,
            'parse_errors': stats.parse_errors
        }
    except Exception:
        pass

    out['status_ok'] = (len(errors) == 0)
    if errors:
        out['errors'] = errors

    return jsonify(out)


@app.route('/api/forces')
def forces():
    try:
        client = get_sensor_client()
        if not client.is_connected:
            return jsonify({'error': 'Not connected'}), 400
        forces = client.read_resulting_force()
        return jsonify(forces)
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/taxel_coordinates')
def taxel_coordinates():
    """Return taxel coordinates for all fingers."""
    return jsonify(get_all_coordinates())


@app.route('/api/refresh')
def refresh():
    try:
        client = get_sensor_client()
        if not client.is_connected:
            return jsonify({'connected': False})

        connected = client.read_connected_sensors()
        version = client.read_hardware_version()
        taxels = client.read_num_taxels()
        auto_data = client.read_auto_data_type()
        forces = client.read_resulting_force()

        return jsonify({
            'connected': True,
            'hardware_version': version,
            'sensors': connected,
            'taxels': taxels,
            'auto_data_type': auto_data,
            'forces': forces,
            'mode': current_mode
        })
    except Exception as e:
        return jsonify({'connected': False, 'error': str(e)}), 400

@socketio.on('connect')
def handle_connect():
    emit('connected', {'data': 'Connected to WebSocket'})

@socketio.on('disconnect')
def handle_disconnect():
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ORCA Sensor Testing UI')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to hand config YAML (for sensor wiring mapping)')
    args = parser.parse_args()

    if args.config:
        config_dir = os.path.dirname(os.path.abspath(args.config))
        with open(args.config) as f:
            config_data = yaml.safe_load(f)
        sensors_cfg = config_data.get('sensors', {})
        mapping = sensors_cfg.get('finger_to_sensor_id')
        if mapping:
            finger_to_sensor_id_config = mapping
            print(f"Loaded sensor mapping from {args.config}: {finger_to_sensor_id_config}")

    socketio.run(app, host='0.0.0.0', port=5001, debug=True, allow_unsafe_werkzeug=True)
