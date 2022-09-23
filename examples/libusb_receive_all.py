import can

filters= [{"can_id": 0x1, "can_mask": 0x3, "extended": False},
      {"can_id": 0x2, "can_mask": 0x3, "extended": True},
      {"can_id": 0x3, "can_mask": 0x3}]
with can.interface.Bus(bustype="usb2can_libusb", channel="C454B93C", bitrate=1000000, can_filters=filters) as bus:
    try:
        while True:
            msg = bus.recv(1)
            if msg is not None:
                print(msg)
    except KeyboardInterrupt:
        pass  # exit normally

