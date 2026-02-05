#!/usr/bin/env python

from .scservo_def import *
from .protocol_packet_handler import *
from .group_sync_write import *

# Baud rate definitions
SCSCL_1M = 0
SCSCL_0_5M = 1
SCSCL_250K = 2
SCSCL_128K = 3
SCSCL_115200 = 4
SCSCL_76800 = 5
SCSCL_57600 = 6
SCSCL_38400 = 7

# Memory table definitions
#-------EPROM (read-only)--------
SCSCL_MODEL_L = 3
SCSCL_MODEL_H = 4

#-------EPROM (read-write)--------
scs_id = 5
SCSCL_BAUD_RATE = 6
SCSCL_MIN_ANGLE_LIMIT_L = 9
SCSCL_MIN_ANGLE_LIMIT_H = 10
SCSCL_MAX_ANGLE_LIMIT_L = 11
SCSCL_MAX_ANGLE_LIMIT_H = 12
SCSCL_CW_DEAD = 26
SCSCL_CCW_DEAD = 27

#-------SRAM (read-write)--------
SCSCL_TORQUE_ENABLE = 40
SCSCL_GOAL_POSITION_L = 42
SCSCL_GOAL_POSITION_H = 43
SCSCL_GOAL_TIME_L = 44
SCSCL_GOAL_TIME_H = 45
SCSCL_GOAL_SPEED_L = 46
SCSCL_GOAL_SPEED_H = 47
SCSCL_LOCK = 48

#-------SRAM (read-only)--------
SCSCL_PRESENT_POSITION_L  = 56
SCSCL_PRESENT_POSITION_H = 57
SCSCL_PRESENT_SPEED_L = 58
SCSCL_PRESENT_SPEED_H = 59
SCSCL_PRESENT_LOAD_L = 60
SCSCL_PRESENT_LOAD_H = 61
SCSCL_PRESENT_VOLTAGE = 62
SCSCL_PRESENT_TEMPERATURE = 63
SCSCL_MOVING = 66
SCSCL_PRESENT_CURRENT_L = 69
SCSCL_PRESENT_CURRENT_H = 70

class scscl(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 1)
        self.groupSyncWrite = GroupSyncWrite(self, SCSCL_GOAL_POSITION_L, 6)

    def WritePos(self, scs_id, position, time, speed):
        txpacket = [self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(time), self.scs_hibyte(time), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SCSCL_PRESENT_POSITION_L)
        return scs_present_position, scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SCSCL_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, SCSCL_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return scs_present_position, self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, SCSCL_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePos(self, scs_id, position, time, speed):
        txpacket = [self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(time), self.scs_hibyte(time), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.groupSyncWrite.addParam(scs_id, txpacket)

    def RegWritePos(self, scs_id, position, time, speed):
        txpacket = [self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(time), self.scs_hibyte(time), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.regWriteTxRx(scs_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def PWMMode(self, scs_id):
        txpacket = [0, 0, 0, 0]
        return self.writeTxRx(scs_id, SCSCL_MIN_ANGLE_LIMIT_L, len(txpacket), txpacket)

    def WritePWM(self, scs_id, time):
        return self.write2ByteTxRx(scs_id, SCSCL_GOAL_TIME_L, self.scs_toscs(time, 10))

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SCSCL_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SCSCL_LOCK, 0)
