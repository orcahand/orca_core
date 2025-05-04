from orca_core import OrcaHand, Retargeter
from avp_stream import VisionProStreamer
import time

avp_ip = "192.168.1.10"
s = VisionProStreamer(ip = avp_ip, record = True)

time.sleep(5)

hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')
status = hand.connect()
print(status)


retargeter = Retargeter('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')

while True:
    r = s.latest
    joint_angles = retargeter.retarget(r)
    print(joint_angles)
    
    
#hand.calibrate()

hand.set_joint_pos({'wrist': -20.0})
# 



# Set the desired joint positions to 0
# hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})

