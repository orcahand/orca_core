#!/usr/bin/env python

from .scservo_def import *

class GroupSyncRead:
    def __init__(self, ph, start_address, data_length):
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:  # len(self.data_dict.keys()) == 0:
            return

        self.param = []

        for scs_id in self.data_dict:
            self.param.append(scs_id)

    def addParam(self, scs_id):
        if scs_id in self.data_dict:  # scs_id already exist
            return False

        self.data_dict[scs_id] = []  # [0] * self.data_length

        self.is_param_changed = True
        return True

    def removeParam(self, scs_id):
        if scs_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[scs_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:

            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(self.start_address, self.data_length, self.param, len(self.data_dict.keys()))

    def rxPacket(self):
        self.last_result = True

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        result, rxpacket = self.ph.syncReadRx(self.data_length, len(self.data_dict.keys()))
        # print(rxpacket)
        if len(rxpacket) >= (self.data_length+6):
            for scs_id in self.data_dict:
                self.data_dict[scs_id], result = self.readRx(rxpacket, scs_id, self.data_length)
                if result != COMM_SUCCESS:
                    self.last_result = False
                # print(scs_id)
        else:
            self.last_result = False
        # print(self.last_result)
        return result

    def txRxPacket(self):
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def readRx(self, rxpacket, scs_id, data_length):
        # print(scs_id)
        # print(rxpacket)
        data = []
        rx_length = len(rxpacket)
        # print(rx_length)
        rx_index = 0;
        while (rx_index+6+data_length) <= rx_length:
            headpacket = [0x00, 0x00, 0x00]
            while rx_index < rx_length:
                headpacket[2] = headpacket[1];
                headpacket[1] = headpacket[0];
                headpacket[0] = rxpacket[rx_index];
                rx_index += 1
                if (headpacket[2] == 0xFF) and (headpacket[1] == 0xFF) and headpacket[0] == scs_id:
                    # print(rx_index)
                    break
            # print(rx_index+3+data_length)
            if (rx_index+3+data_length) > rx_length:
                break;
            if rxpacket[rx_index] != (data_length+2):
                rx_index += 1
                # print(rx_index)
                continue
            rx_index += 1
            Error = rxpacket[rx_index]
            rx_index += 1
            calSum = scs_id + (data_length+2) + Error
            data = [Error]
            data.extend(rxpacket[rx_index : rx_index+data_length])
            for i in range(0, data_length):
                calSum += rxpacket[rx_index]
                rx_index += 1
            calSum = ~calSum & 0xFF
            # print(calSum)
            if calSum != rxpacket[rx_index]:
                return None, COMM_RX_CORRUPT
            return data, COMM_SUCCESS 
        # print(rx_index)
        return None, COMM_RX_CORRUPT

    def isAvailable(self, scs_id, address, data_length):
        #if self.last_result is False or scs_id not in self.data_dict:
        if scs_id not in self.data_dict:
            return False, 0

        if (address < self.start_address) or (self.start_address + self.data_length - data_length < address):
            return False, 0
        if not self.data_dict[scs_id]:
            return False, 0
        if len(self.data_dict[scs_id])<(data_length+1):
            return False, 0
        return True, self.data_dict[scs_id][0]

    def getData(self, scs_id, address, data_length):
        if data_length == 1:
            return self.data_dict[scs_id][address-self.start_address+1]
        elif data_length == 2:
            return self.ph.scs_makeword(self.data_dict[scs_id][address-self.start_address+1],
                                self.data_dict[scs_id][address-self.start_address+2])
        elif data_length == 4:
            return self.ph.scs_makedword(self.ph.scs_makeword(self.data_dict[scs_id][address-self.start_address+1],
                                              self.data_dict[scs_id][address-self.start_address+2]),
                                 self.ph.scs_makeword(self.data_dict[scs_id][address-self.start_address+3],
                                              self.data_dict[scs_id][address-self.start_address+4]))
        else:
            return 0
