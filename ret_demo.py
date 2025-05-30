from orca_core import OrcaHand, Retargeter
from avp_stream import VisionProStreamer
import time

avp_ip = "192.168.1.10"
s = VisionProStreamer(ip = avp_ip, record = True)

time.sleep(5)

hand = OrcaHand('/home/ccc/orca_ws/src/orca_core/orca_core/models/orcahand_v1')
status = hand.connect()
print(status)
# hand.calibrate()

retargeter = Retargeter('/home/ccc/orca_ws/src/orca_core/orca_core/models/orcahand_v1')

while True:
    r = s.latest
    joint_angles, _ = retargeter.retarget(r)
    temp_dict = {}
    temp_dict['thumb_abd'] = joint_angles['thumb_mcp']
    temp_dict['thumb_mcp'] = joint_angles['thumb_abd']
    joint_angles['thumb_abd'] = temp_dict['thumb_abd']
    joint_angles['thumb_mcp'] = temp_dict['thumb_mcp']
    hand.set_joint_pos(joint_angles)

    
#hand.calibrate()

hand.set_joint_pos({'wrist': -20.0})
# 



# Set the desired joint positions to 0
# hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})

