"""HLS-series register map, from Feetech's HLS memory-table manual.

Two-byte registers are little-endian; BIT15 is the sign bit where noted.
Only the registers the client touches are listed.
"""


class HLS:
    # EEPROM
    ID = 5
    BAUD_RATE = 6
    PROTECTION_CURRENT = 28   # 6.5 mA/unit, 0..2047; copied into GOAL_CURRENT at power-up
    MODE = 33                 # 0 position under current limit, 1 speed, 2 current, 3 PWM

    # SRAM, read-write
    TORQUE_ENABLE = 40
    ACC = 41                  # 8.7 deg/s^2 per unit, 0 = maximum
    GOAL_POSITION = 42        # 0.087 deg/unit, BIT15 sign
    GOAL_CURRENT = 44         # 6.5 mA/unit, -2047..2047; caps the running current in modes 0-2
    GOAL_SPEED = 46           # 0.732 rpm/unit, BIT15 sign
    TORQUE_LIMIT = 48         # 0.1 %/unit, 0..1000; a different register from GOAL_CURRENT, never written
    KP = 50                   # position loop P, 0..254; loaded from EEPROM 21 at power-up
    KD = 51                   # position loop D, 0..254; loaded from EEPROM 22
    KI = 52                   # position loop I, 0..254; loaded from EEPROM 23
    LOCK = 55                 # 0: EEPROM writes persist across power-down, 1: they do not

    # SRAM, read-only
    PRESENT_POSITION = 56
    PRESENT_SPEED = 58
    PRESENT_TEMPERATURE = 63
    MOVING = 66
    PRESENT_CURRENT = 69      # 6.5 mA/unit, BIT15 sign

    GAIN_BLOCK = KP           # KP, KD, KI are contiguous and read/write as one block
    GAIN_BLOCK_LEN = 3
    GAIN_MAX = 254

    CURRENT_SCALE_MA = 6.5
    GOAL_CURRENT_MAX_RAW = 2047
    POSITION_PROFILE_LEN = 7  # ACC through GOAL_SPEED, written as one sync-write block
