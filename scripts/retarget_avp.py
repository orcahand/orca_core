from orca_core import OrcaHand, Retargeter
from avp_stream import VisionProStreamer
avp_ip = "10.93.181.127"   # example IP 
s = VisionProStreamer(ip = avp_ip, record = True)

# hand = OrcaHand('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')
# if not hand.connect()[0]:
#     print("Failed to connect to the hand.")

# ret = Retargeter('/Users/ccc/dev/orca/orca_core/orca_core/models/orcahand_v1_berkeley')


while True:
    r = s.latest
    # cmd_angles = ret.retarget(r)
    print(r)
