#!/usr/bin/env python

from .scservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

# Baud rate definitions
HLS_1M = 0
HLS_0_5M = 1
HLS_250K = 2
HLS_128K = 3
HLS_115200 = 4
HLS_76800 = 5
HLS_57600 = 6
HLS_38400 = 7

# Memory table definitions
#-------EPROM (read-only)--------
HLS_MODEL_L = 3
HLS_MODEL_H = 4

#-------EPROM (read-write)--------
HLS_ID = 5
HLS_BAUD_RATE = 6
HLS_MIN_ANGLE_LIMIT_L = 9
HLS_MIN_ANGLE_LIMIT_H = 10
HLS_MAX_ANGLE_LIMIT_L = 11
HLS_MAX_ANGLE_LIMIT_H = 12
HLS_CW_DEAD = 26
HLS_CCW_DEAD = 27
HLS_OFS_L = 31
HLS_OFS_H = 32
HLS_MODE = 33

#-------SRAM (read-write)--------
HLS_TORQUE_ENABLE = 40
HLS_ACC = 41
HLS_GOAL_POSITION_L = 42
HLS_GOAL_POSITION_H = 43
HLS_GOAL_TORQUE_L = 44
HLS_GOAL_TORQUE_H = 45
HLS_GOAL_SPEED_L = 46
HLS_GOAL_SPEED_H = 47
HLS_LOCK = 55

#-------SRAM (read-only)--------
HLS_PRESENT_POSITION_L = 56
HLS_PRESENT_POSITION_H = 57
HLS_PRESENT_SPEED_L = 58
HLS_PRESENT_SPEED_H = 59
HLS_PRESENT_LOAD_L = 60
HLS_PRESENT_LOAD_H = 61
HLS_PRESENT_VOLTAGE = 62
HLS_PRESENT_TEMPERATURE = 63
HLS_MOVING = 66
HLS_PRESENT_CURRENT_L = 69
HLS_PRESENT_CURRENT_H = 70

class hls(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, HLS_ACC, 7)

    def WritePosEx(self, scs_id, position, speed, acc, torque):
        position = self.scs_tohost(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(torque), self.scs_hibyte(torque), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, HLS_ACC, len(txpacket), txpacket)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, HLS_PRESENT_POSITION_L)
        return self.scs_tohost(scs_present_position, 15), scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, HLS_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, HLS_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return self.scs_tohost(scs_present_position, 15), self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, HLS_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePosEx(self, scs_id, position, speed, acc, torque):
        position = self.scs_tohost(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(torque), self.scs_hibyte(torque), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.groupSyncWrite.addParam(scs_id, txpacket)

    def RegWritePosEx(self, scs_id, position, speed, acc, torque):
        position = self.scs_tohost(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(torque), self.scs_hibyte(torque), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.regWriteTxRx(scs_id, HLS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, scs_id):
        return self.write1ByteTxRx(scs_id, HLS_MODE, 1)
    
    def WriteSpec(self, scs_id, speed, acc, torque):
        speed = self.scs_toscs(speed, 15)
        txpacket = [acc, 0, 0, self.scs_lobyte(torque), self.scs_hibyte(torque), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, HLS_ACC, len(txpacket), txpacket)

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, HLS_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, HLS_LOCK, 0)

