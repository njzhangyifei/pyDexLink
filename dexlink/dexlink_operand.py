"""
File: dexlink_operand.py
Author: Yifei Zhang
Email: njzhangyifei@gmail.com
Github: https://github.com/njzhangyifei
Description: Contains all the operand for DexLink
"""

from enum import Enum

__author__ = 'Yifei'


class DexLinkOperand(Enum):
    response = 0x00
    ping = 0x01
    ping_physical_address = 0x02
    wait = 0x03
    execute = 0x04

    read_torque_output = 0x10
    read_mode_selection = 0x11
    read_sensor = 0x12
    read_setpoint = 0x13
    read_motor = 0x14
    read_led = 0x15

    config_torque_output = 0x20
    config_mode_selection = 0x21
    config_led = 0x22
    config_address = 0x23
    config_baud_rate = 0x24
    config_position_direction = 0x25

    position_request = 0x30
    position_direction_request = 0x31
    position_velocity_request = 0x32

    cont_rot_velocity_request = 0x40

    openloop_ab = 0x60
    openloop_a = 0x61
    openloop_b = 0x62
