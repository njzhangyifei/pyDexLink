"""
File: dexlink_packet.py
Author: Yifei Zhang
Email: njzhangyifei@gmail.com
Github: https://github.com/njzhangyifei
Description: This file contains definitions for DexLinkPacket and its subclasses
"""

from .util import crc16_ccitt
from .util.boyer_moore_search import BoyerMoore, boyer_moore

from .util import byte_bit_util
from .dexlink_constants import *
from .dexlink_operand import DexLinkOperand

__author__ = 'Yifei'

_p_bm = BoyerMoore(HEADER_BYTES)


class DexLinkPacket(object):
    """Packet for dexlink"""

    def __init__(self, address, operand, payload):
        """Initialize the packet"""
        if isinstance(operand, DexLinkOperand):
            operand = operand.value
        self._address = address
        self._length = len(payload) + OPERAND_LEN + ADDRESS_LEN
        self._operand = operand
        self._payload = payload
        self._checksum = [0x00, 0x00]
        self.update_checksum()
        self._bytes = None

    @staticmethod
    def create_packet(t, address, operand, payload):
        """
        Create a packet with the arguments provided
        :param t: the type of the packet
        :param address: the address field
        :param operand: the operand field
        :param payload: the payload field
        :return: a type t packet
        """
        packet = t(address, operand, payload)
        return packet

    @staticmethod
    def verify_checksum(address, operand, payload, checksum):
        """
        Verify the checksum
        :param address: the address field of the packet
        :param operand: the operand field of the packet
        :param payload: the payload field of the packet
        :param checksum: the checksum to be verified
        :return: True or False
        :rtype: bool
        """
        return checksum == DexLinkPacket.get_checksum(address, operand, payload)

    def address(self):
        """
        Get the address field in this packet
        :return: address field
        :rtype: int
        """
        return self._address

    def operand(self):
        """
        Get the operand field in this packet
        :return: operand field
        :rtype: DexLinkOperand
        """
        try:
            return DexLinkOperand(self._operand)
        except:
            return self._operand

    def payload(self):
        """
        Get the payload in this packet
        :return: the packet payload as a list of int
        :rtype: list[int]
        """
        return self._payload

    @staticmethod
    def parse(packet_type, buffer):
        """
        Parse a buffer for a given packet type
        :param packet_type: the type of the packet to be parsed
        :param buffer: the buffer from which to parse
        :return: a tuple containing the parsed packets list and the byte
        consumed
        """
        packet_list = []
        original_buffer_length = len(buffer)
        byte_consumed = 0

        # header structure of possible packet
        index_header = 0
        address = 0
        index_header_iter = 0
        payload = None
        operand = None

        found = False

        if len(buffer) < MIN_PACKET_LENGTH:
            return packet_list, byte_consumed

        index_header_occurrences = boyer_moore(
            HEADER_BYTES, _p_bm,
            buffer)
        index_header_occurrences_num = len(index_header_occurrences)

        while True:
            # Find next header
            if index_header_iter == index_header_occurrences_num:
                # we do not have an header next
                byte_consumed = original_buffer_length
                break
            index_header = index_header_occurrences[index_header_iter]
            index_header_iter += 1

            # We have a header now
            if original_buffer_length - index_header < MIN_PACKET_LENGTH:
                byte_consumed = index_header
                break

            # Check length
            # expected packet length
            # = len(op+payload) + header + 1 (len) + 2 (checksum)
            expected_packet_length = \
                buffer[
                    index_header + LENGTH_OFFSET] \
                + MIN_PACKET_LENGTH
            # Packet has a valid header, check payload length
            if original_buffer_length - index_header < expected_packet_length:
                # The packet is not long enough, keep these bytes
                byte_consumed = index_header
                break
            elif expected_packet_length == MIN_PACKET_LENGTH:
                # Zero length packet, drop
                byte_consumed = index_header + MIN_PACKET_LENGTH
                continue

            # Check checksum
            payload = [0] * (
                buffer[index_header + LENGTH_OFFSET] -
                (ADDRESS_LEN + OPERAND_LEN))
            payload = buffer[
                      index_header + PAYLOAD_OFFSET:
                      index_header + PAYLOAD_OFFSET +
                      len(payload)]
            # checksum is two bytes
            checksum = [0] * CHECKSUM_LEN
            checksum[0] = buffer[index_header +
                                 CHECKSUM_OFFSET]
            checksum[1] = buffer[index_header +
                                 CHECKSUM_OFFSET + 1]
            operand = buffer[index_header + OPERAND_OFFSET]
            address = buffer[index_header + ADDRESS_OFFSET]
            checksum_correct = DexLinkPacket.verify_checksum(address, operand,
                                                             payload, checksum)
            # If the checksum is not correct, consume the header and move on
            if not checksum_correct:
                byte_consumed = index_header + HEADER_LEN
                continue

            # Check for invalid address
            if address in INVALID_ADDRESS:
                byte_consumed = index_header + HEADER_LEN
                continue

            found = True;

            # Found a valid packet
            if found:
                packet = DexLinkPacket.create_packet(packet_type, address,
                                                     operand, payload)
                packet_list.append(packet)
                found = False
                byte_consumed = index_header + len(packet.generate_bytes())
        return packet_list, byte_consumed

    def update_checksum(self):
        """Calculate checksum for this packet
        :returns: The checksum for this packet
        """
        self._checksum = self.get_checksum(self._address,
                                           self._operand,
                                           self._payload)
        return self._checksum

    @staticmethod
    def get_checksum(address, operand, payload):
        """
        Calculate the checksum with given arguments
        :param address: the address field
        :param operand: the operand field
        :param payload: the payload field
        :return: the checksum calculated with CRC16-CCITT
        """
        data = [address, operand] + payload
        checksum = crc16_ccitt.calculate_CRC16CCITT(data, len(data))
        return [(checksum & 0xFF00) >> 8, checksum & 0x00FF]

    def generate_bytes(self):
        """
        Generate bytes for the DexLinkPacket
        :returns: The byte array for this DexLinkPacket
        """
        self.update_checksum()
        self._bytes = HEADER_BYTES + \
                      [self._length] + \
                      self._checksum + \
                      [self._address] + \
                      [self._operand] + \
                      self._payload
        return self._bytes

    def __str__(self):
        return byte_bit_util.byte_list_to_string(self.generate_bytes())

    def __eq__(self, other):
        if not (type(other) is type(self)):
            return False
        if str(self) == str(other):
            return True
        return False

    def __ne__(self, other):
        return not (self.__eq__(other))


class DexLinkRequestPacket(DexLinkPacket):
    def __init__(self, address, operand, payload):
        super().__init__(address, operand, payload)

    @staticmethod
    def parse(buffer):
        """
        Override, parse the DexLinkRequestPacket from a given buffer
        :param buffer: the buffer to be parsed from
        :return: a tuple containing a list of packet parsed and the number of
        bytes consumed
        """
        return DexLinkPacket.parse(DexLinkRequestPacket, buffer)

    pass


class DexLinkResponsePacket(DexLinkPacket):
    def __init__(self, address, operand, payload):
        super().__init__(address, operand, payload)

    @staticmethod
    def parse(buffer):
        """
        Override, parse the DexLinkResponsePacket from a given buffer
        :param buffer: the buffer to be parsed from
        :return: a tuple containing a list of packet parsed and the number of
        bytes consumed
        """
        return DexLinkPacket.parse(DexLinkResponsePacket, buffer)

    def read_int(self, payload_start_index, payload_end_index, signed=False):
        """
        Read a int from given positions from payload
        :param payload_start_index: the begin index
        :param payload_end_index: the end index
        :param signed: if the int is signed
        :return: the integer parsed
        :rtype: int
        """
        p = self._payload[payload_start_index:payload_end_index]
        t = int.from_bytes(
            p,
            byteorder='big', signed=signed)
        return t

    def request_operand(self):
        """
        Get the request operand from the response packet
        :return: the first payload byte
        """
        return self._payload[0]

    def status(self):
        """
        Get the status byte from the response packet
        :return: the second payload byte
        """
        return self._payload[1]

    pass
