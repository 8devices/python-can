"""
This interface is for Windows only, otherwise use SocketCAN.
"""

import logging
import ctypes 
from typing import Optional, cast

from can import BusABC, Message, CanInitializationError, CanOperationError
from .usb2canabstractionlayer import Usb2CanAbstractionLayer, CanalMsg, CanalError
from .usb2canabstractionlayer import (
    IS_ERROR_FRAME,
    IS_REMOTE_FRAME,
    IS_ID_TYPE,
)
from .serial_selector import find_serial_devices
from can.typechecking import CanFilters, CanFilterExtended

# Set up logging
log = logging.getLogger("can.usb2can")


def message_convert_tx(msg):
    message_tx = CanalMsg()

    length = msg.dlc
    message_tx.sizeData = length

    message_tx.id = msg.arbitration_id

    for i in range(length):
        message_tx.data[i] = msg.data[i]

    message_tx.flags = 0x80000000

    if msg.is_error_frame:
        message_tx.flags |= IS_ERROR_FRAME

    if msg.is_remote_frame:
        message_tx.flags |= IS_REMOTE_FRAME

    if msg.is_extended_id:
        message_tx.flags |= IS_ID_TYPE

    return message_tx


def message_convert_rx(message_rx):
    """convert the message from the CANAL type to pythoncan type"""
    is_extended_id = bool(message_rx.flags & IS_ID_TYPE)
    is_remote_frame = bool(message_rx.flags & IS_REMOTE_FRAME)
    is_error_frame = bool(message_rx.flags & IS_ERROR_FRAME)

    return Message(
        timestamp=message_rx.timestamp,
        is_remote_frame=is_remote_frame,
        is_extended_id=is_extended_id,
        is_error_frame=is_error_frame,
        arbitration_id=message_rx.id,
        dlc=message_rx.sizeData,
        data=message_rx.data[: message_rx.sizeData],
    )


class Usb2canBus(BusABC):
    """Interface to a USB2CAN Bus.

    This interface only works on Windows.
    Please use socketcan on Linux.

    :param str channel (optional):
        The device's serial number. If not provided, Windows Management Instrumentation
        will be used to identify the first such device.

    :param int bitrate (optional):
        Bitrate of channel in bit/s. Values will be limited to a maximum of 1000 Kb/s.
        Default is 500 Kbs

    :param int flags (optional):
        Flags to directly pass to open function of the usb2can abstraction layer.

    :param str dll (optional):
        Path to the DLL with the CANAL API to load
        Defaults to 'usb2can.dll'

    :param str serial (optional):
        Alias for `channel` that is provided for legacy reasons.
        If both `serial` and `channel` are set, `serial` will be used and
        channel will be ignored.

    :param can_filters (optional):
        See :meth:`can.BusABC.set_filters`.

    """

    def __init__(
        self,
        channel=None,
        dll="usb2can.dll",
        flags=0x00000008,
        *_,
        bitrate=500000,
        can_filters: Optional[CanFilters] = None,
        **kwargs,
    ):
        self._is_filtered = False
        self.can = Usb2CanAbstractionLayer(dll)

        # get the serial number of the device
        device_id = kwargs.get("serial", channel)

        # search for a serial number if the device_id is None or empty
        if not device_id:
            devices = find_serial_devices()
            if not devices:
                raise CanInitializationError("could not automatically find any device")
            device_id = devices[0]

        # convert to kb/s and cap: max rate is 1000 kb/s
        baudrate = min(int(bitrate // 1000), 1000)

        self.channel_info = f"USB2CAN device {device_id}"

        connector = f"{device_id}; {baudrate}"
        self.handle = self.can.open(connector, flags)

        super().__init__(channel=channel, can_filters=can_filters, **kwargs)

    def send(self, msg, timeout=None):
        tx = message_convert_tx(msg)

        if timeout:
            status = self.can.blocking_send(self.handle, ctypes.byref(tx), int(timeout * 1000))
        else:
            status = self.can.send(self.handle, ctypes.byref(tx))

        if status != CanalError.SUCCESS:
            raise CanOperationError("could not send message", error_code=status)

    def _recv_internal(self, timeout):

        messagerx = CanalMsg()

        if timeout == 0:
            status = self.can.receive(self.handle, ctypes.byref(messagerx))

        else:
            time = 0 if timeout is None else int(timeout * 1000)
            status = self.can.blocking_receive(self.handle, ctypes.byref(messagerx), time)

        if status == CanalError.SUCCESS:
            rx = message_convert_rx(messagerx)
        elif status in (
            CanalError.RCV_EMPTY,
            CanalError.TIMEOUT,
            CanalError.FIFO_EMPTY,
        ):
            rx = None
        else:
            raise CanOperationError("could not receive message", error_code=status)

        return rx, self._is_filtered

    def _apply_filters(self, can_filters: Optional[CanFilters]) -> None:
        CAN_EXTENDED_FLAG = 0x80000000
        if can_filters is None:
            # Pass all messages
            # can_filters = [{"can_id": 0, "can_mask": 0}]
            self._is_filtered = False
            return
        filter_data = []
        for can_filter in can_filters:
            can_id = can_filter["can_id"]
            can_mask = can_filter["can_mask"]
            if "extended" in can_filter:
                can_filter = cast(CanFilterExtended, can_filter)
                # Match on either 11-bit OR 29-bit messages instead of both
                can_mask |= CAN_EXTENDED_FLAG
                if can_filter["extended"]:
                    can_id |= CAN_EXTENDED_FLAG
            filter_data.append(can_id)
            filter_data.append(can_mask)
        filter_len =len(filter_data)

        self.can.set_filters(self.handle,ctypes.c_int(filter_len), (ctypes.c_ulong * filter_len)(*filter_data))
        self._is_filtered = True

    def shutdown(self):
        """
        Shuts down connection to the device safely.

        :raise cam.CanOperationError: is closing the connection did not work
        """
        super().shutdown()
        status = self.can.close(self.handle)

        if status != CanalError.SUCCESS:
            raise CanOperationError("could not shut down bus", error_code=status)

    @staticmethod
    def _detect_available_configs():
        return Usb2canBus.detect_available_configs()

    @staticmethod
    def detect_available_configs(serial_matcher: Optional[str] = None):
        """
        Uses the *Windows Management Instrumentation* to identify serial devices.

        :param serial_matcher:
            search string for automatic detection of the device serial
        """
        if serial_matcher is None:
            channels = find_serial_devices()
        else:
            channels = find_serial_devices(serial_matcher)

        return [{"interface": "usb2can", "channel": c} for c in channels]
