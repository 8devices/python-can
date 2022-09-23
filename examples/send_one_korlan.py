import can
from time import sleep
from importlib import reload 

#
for j in range(1):
    with (can.interface.Bus(bustype="usb2can_libusb", channel="CDE7E976", bitrate=1000000)) as bus:
        for i in range(200):
            msg = can.Message(arbitration_id=0x1, data=[0, 25, 0, 1, 3, 1, 4, 1], is_extended_id=False)
            try:
                bus.send(msg)
                #print(f"Message {i} sent on {bus.channel_info}")
            except can.CanError:
                print("Message NOT sent")
        bus.shutdown()
        print(f"closing after series {j+1}")
    #sleep(1)
    #reload(can)
