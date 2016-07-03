#!/usr/bin/env python3

import time
from dexlink.dexlink_packet import *
from dexlink.dexlink_operand import DexLinkOperand
from dexlink.dexlink_common import *
from dexlink.dexlink_serial import DexLinkSerial
from dexlink.dexlink_device import DexLinkServo

######################################################
# open the serial port
d = DexLinkSerial('COM3')
d.open()

######################################################
# generate Ping packet
ping_packet = DexLinkRequestPacket(address=0x01,
                                   operand=DexLinkOperand.ping,
                                   payload=[0x00])

######################################################
# scan throw address 1 through 20
print("Searching for Servo...")
device_list = d.scan(DexLinkServo, 1, 20)


######################################################
# [ Example for Ping-ing device ]
device_alive = False
try:
    resp = d.comm(ping_packet)
    if resp:
        device_alive = True
except DexLinkTimeoutException:
    print("Timeout!")
except DexLinkSerialException:
    print("Serial Exception!")

print("Servo Found!")
for servo in device_list:
    print(" - " + str(servo))

print("Configuring first servo...")
# We've got a servo alive
servo = device_list[0]  # type: DexLinkServo
# Print out the description for the servo

address = int(input("Please enter the address:"))
num_retry = 0
while num_retry < 5:
    try:
        num_retry += 1
        servo.config_address(address)
        if servo.address == address:
            print("Success!")
            break
    except DexLinkTimeoutException:
        print(str(num_retry)+"/5 - "+"Timeout!")
    except DexLinkSerialException:
        print(str(num_retry)+"/5 - "+"Serial Exception!")
        break

if num_retry > 5:
    print("Failed!")

