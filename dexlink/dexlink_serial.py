"""
File: dexlink_serial.py
Author: Yifei Zhang
Email: njzhangyifei@gmail.com
Github: https://github.com/njzhangyifei
Description: Contains definition of the DexLinkSerial class
"""

import logging
import time
from threading import Lock

import serial

from .dexlink_common import *
from .dexlink_packet import *
from .dexlink_device import *


class DexLinkSerial:
    __TIMEOUT_SERIAL = 0.01  # the timeout for the first byte to arrive
    __TIMEOUT_PACKET = 0.02  # the timeout for the entire packet to be received
    __RETRY_COUNT = 3        # the retry count when experiencing timeout

    def __init__(self, portname, baudrate=115200,
                 timeout_serial=__TIMEOUT_SERIAL,
                 timeout_packet=__TIMEOUT_PACKET,
                 retry_count=__RETRY_COUNT):
        """
        Initialize the DexLinkSerial
        :param portname: name of the serial port, e.g. 'COM1' or '/dev/tty.xxx'
        :param baudrate: the baud rate to use
        :param timeout_serial: the timeout between bytes
        :param timeout_packet: the timeout for the entire packet
        :param retry_count: the retry count for timeout conditions
        :return: None
        """
        self.portname = portname
        self.baudrate = baudrate
        self.timeout_serial = timeout_serial
        self.timeout_packet = timeout_packet
        self.retry_count = retry_count
        self.lock = Lock()
        self.serial = None  # type: serial.Serial
        self.is_open = False
        self.logger = logging.getLogger(type(self).__name__)

    def refresh_timeout(self):
        self.serial.setTimeout(self.timeout_serial)

    def open(self):
        """
        Thread-safe method for opening the serial port
        :return: None
        """
        self.logger.debug("opening DexLinkSerial @ " + self.portname)
        if self.is_open:
            raise DexLinkInvalidOperationException("Serial port already opened")
        with self.lock:
            self._open()

    def _open(self):
        self.serial = serial.Serial(port=self.portname,
                                    baudrate=self.baudrate,
                                    parity=serial.PARITY_EVEN,
                                    timeout=self.timeout_serial)
        self.is_open = True

    def close(self):
        """
        Thread-safe method for closing the serial port
        :return: None
        """
        self.logger.debug("closing DexLinkSerial @ " + self.portname)
        if not self.is_open:
            raise DexLinkInvalidOperationException("Serial port already closed")
        with self.lock:
            self._close()

    def _close(self):
        try:
            self.serial.close()
            self.is_open = False
        except:
            raise DexLinkSerialException("Unable to close serial port")

    def send_request(self, packet):
        """
        Thread-safe method for sending a request
        :param packet: the DexLinkRequestPacket to be sent
        :return: None
        """
        with self.lock:
            self._send_request(packet)

    def _send_request(self, packet):
        """

        :type packet: DexLinkRequestPacket
        """
        if not self.is_open:
            raise DexLinkInvalidOperationException("Serial port not opened")
        data = packet.generate_bytes()
        self.logger.debug("sending request " + str(data))
        try:
            self.serial.flushInput()
            self.serial.write(data)
        except serial.SerialException:
            self.logger.warn("error in sending")
            raise DexLinkSerialException("Error in sending request")

    def receive_response(self):
        """
        Receive a response packet from the serial port
        :rtype:  DexLinkResponsePacket
        """
        with self.lock:
            return self._receive_response()

    def _receive_response(self):
        if not self.is_open:
            raise DexLinkInvalidOperationException("Serial port not opened")
        # we should get at least one byte during this timeout
        buffer = []
        incoming_data = None
        try:
            incoming_data = self.serial.read(self.serial.inWaiting() or 1)
        except serial.SerialException:
            self.logger.debug("error in receiving")
            raise DexLinkSerialException("Error in receiving response")
        if not incoming_data:
            self.logger.debug("time_serial expires")
            raise DexLinkTimeoutException("Timeout in receiving response")
        buffer += incoming_data

        packet_start_time = time.time()

        while True:
            # parse the buffer
            parsed_packets, bytes_consumed = DexLinkResponsePacket.parse(buffer)
            buffer = buffer[bytes_consumed:]
            if len(parsed_packets) > 0:
                return parsed_packets[0]
            # check for timeout
            time_elapsed = time.time() - packet_start_time
            if time_elapsed > self.timeout_packet:
                self.logger.debug("timeout_packet expires")
                raise DexLinkTimeoutException("Timeout in receiving response")
            incoming_data = None
            try:
                incoming_data = self.serial.read(self.serial.inWaiting() or 1)
            except serial.SerialException:
                self.logger.debug("error in receiving")
                raise DexLinkSerialException("Error in receiving response")
            if not incoming_data:
                # keep waiting
                continue
            buffer += incoming_data

    def comm(self, packet):
        """
        Begin a communication using a DexLinkRequestPacket
        :param packet: Request packet
        :type packet: DexLinkRequestPacket
        :return: Response packet
        :rtype: DexLinkResponsePacket
        """
        with self.lock:
            return self._comm(packet)

    def _comm(self, packet):
        self._send_request(packet)
        response = self._receive_response()
        op = packet.operand()
        if isinstance(op, DexLinkOperand):
            op = op.value
        if (response.operand() != DexLinkOperand.response) or \
                (len(response.payload()) < 2) or \
                (response.payload()[0] != op):
            raise DexLinkTimeoutException("Invalid response packet")
        return response

    def comm_retry(self, packet):
        """
        Communicate with retry
        :param packet: the request packet
        :type packet: DexLinkRequestPacket
        :return: response packet if received
        :rtype: DexLinkResponsePacket
        :raises DexLinkTimeoutException: on timeout
        :raises DexLinkSerialException: when serial port error occurs
        """
        with self.lock:
            i = 0
            while True:
                rtnval = None
                i += 1
                try:
                    rtnval = self._comm(packet)
                except DexLinkTimeoutException as e:
                    if i > self.retry_count:
                        raise e
                    self.logger.debug("timeout, retrying " + str(i))
                    continue
                except DexLinkSerialException as e:
                    if i > self.retry_count:
                        raise e
                    self.logger.debug("serial error")
                    continue
                if rtnval:
                    return rtnval

    def scan(self, device_type, start_address=1, end_address=255):
        """
        Scan the whole bus for a given device type using their ping method
        :param start_address: the first address to be scanned
        :param end_address: the last address to be scanned
        :param device_type: the type of the device
        :return: a list of found device_type device
        """
        if not issubclass(device_type, DexLinkDevice):
            raise DexLinkInvalidOperationException("device_type must be a "
                                                   "DexLinkDevice")
        if (start_address < 0 or start_address > 255) or (
                    (end_address < 0 or end_address > 255) or (
                            start_address > end_address)):
            raise DexLinkInvalidOperationException("Invalid address bounds")
        available_device = []
        for i in range(start_address-1, end_address):
            if i in INVALID_ADDRESS:
                continue
            device_info = device_type.ping_address(self, i)
            if device_info:
                available_device.append(
                    device_type(self, i, device_info['model'],
                                device_info['unique_id']))

        return available_device
