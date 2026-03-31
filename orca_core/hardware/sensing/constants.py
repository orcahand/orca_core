"""Constants for ORCA tactile sensing."""

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
VALID_SENSOR_IDS = set(range(5))

# Default hardware settings
DEFAULT_SENSOR_PORT = "/dev/ttyACM1"
DEFAULT_SENSOR_BAUDRATE = 921600
DEFAULT_FINGER_TO_SENSOR_ID = {
    "thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4,
}

# Default taxel counts per finger (must match sensor model configs)
DEFAULT_TAXEL_COUNTS = {
    "thumb": 51, "index": 87, "middle": 87, "ring": 87, "pinky": 51,
}

# Finger-to-sensor-model mapping (replaces sensor_models.yaml)
FINGER_MODELS = {
    "thumb": "touch-sensor-thumb",
    "index": "touch-sensor-finger",
    "middle": "touch-sensor-finger",
    "ring": "touch-sensor-finger",
    "pinky": "touch-sensor-pinky",
}

# Protocol constants
PROTOCOL_HEADER_REQUEST = bytes([0x55, 0xAA])
PROTOCOL_HEADER_RESPONSE = bytes([0xAA, 0x55])
PROTOCOL_HEADER_AUTO = bytes([0xAA, 0x56])
PROTOCOL_RESERVED = 0x00
FUNC_CODE_READ = 0x03
FUNC_CODE_WRITE = 0x10

# Register addresses
ADDR_RESET = 0x0022
ADDR_CONNECTED_SENSORS_START = 0x0010
ADDR_CONNECTED_SENSORS_LENGTH = 4
ADDR_NUM_TAXELS_START = 0x0030
ADDR_NUM_TAXELS_LENGTH = 56
ADDR_RESULTING_FORCE_START = 0x0500
ADDR_RESULTING_FORCE_LENGTH = 168
ADDR_AUTO_DATA_TYPE = 0x0016
ADDR_AUTO_ENABLE = 0x0017

# Auto data type bitmasks
AUTO_DATA_RESULTANT = 0x01
AUTO_DATA_TAXELS = 0x02
