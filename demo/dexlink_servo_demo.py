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
# scan throw address 1 through 20
device_list = d.scan(DexLinkServo, 1, 20)

######################################################
# [ Examples for generating packets ]
ping_packet = DexLinkRequestPacket(address=0x01,
                                   operand=DexLinkOperand.ping,
                                   payload=[0x00])

led_on_packet = DexLinkRequestPacket(address=0x01,
                                     operand=DexLinkOperand.config_led,
                                     payload=[0x00, 0x01])

led_off_packet = DexLinkRequestPacket(address=0x01,
                                      operand=DexLinkOperand.config_led,
                                      payload=[0x00, 0x00])


######################################################
# [ Example for Ping-ing device ]
device_alive = False
try:
    resp = d.comm(ping_packet)
    if resp:
        print(str(resp))
        device_alive = True
except DexLinkTimeoutException:
    print("Timeout!")
except DexLinkSerialException:
    print("Serial Exception!")
# We've got a servo alive
servo = device_list[0]  # type: DexLinkServo
# Print out the description for the servo
print(str(servo))


######################################################
# [ Examples for Basic Config ]
# Enable Torque Output
servo.config_torque_output(True)
print(str(servo.torque_output))
# Config the servo into position control mode
servo.config_mode_selection(servo.Mode.position_control)
print(str(servo.mode_selection.name))

######################################################
# Do something
if device_alive:
    # [ Example for Reading from Sensors]
    # for i in range(100):
    #     try:
    #         servo.update_sensor_readings()
    #         readings = servo.get_sensor_readings_str(servo.Sensor.position)
    #         print(readings)
    #         time.sleep(0.05)
    #     except Exception as e:
    #         print("Comm error" + str(e))
    #         continue

    # [ Example for WAIT/EXECUTE command (synchronize all servo) ]
    # for i in range(0):
    #     try:
    #         servo.wait()
    #         print("Wait Sent")
    #         servo.set_position_deg(90)
    #         print("Position Request Sent")
    #         time.sleep(3)
    #         print("Sending Execute")
    #         resp = servo.execute()
    #         print(resp)
    #         servo.config_led(True)
    #         time.sleep(3)
    #         servo.config_led(False)
    #         print("Sending New Position Request")
    #         servo.set_position_deg(0)
    #         time.sleep(1)
    #     except Exception as e:
    #         print("Comm error" + str(e))
    #         continue

    # [ Example for LED On/off ]
    # servo.config_led(False)
    # time.sleep(2)
    # servo.config_led(True)

    # [ Example for Position Control Mode ]
    # resp = servo.set_position_velocity_rpm(0) # disable velocity limit
    # time.sleep(2)
    # resp = servo.set_position_deg(0, direction=servo.Direction.cw)
    # time.sleep(2)
    # servo.set_position_deg(20,direction=servo.Direction.cw)
    # time.sleep(2)
    # resp = servo.set_position_velocity_rpm(0.5) # enable velocity limit
    # servo.set_position_deg(80)
    # time.sleep(10)

    # [ Example for Continuous Rotation ]
    # servo.config_mode_selection(servo.Mode.continuous_rotation)
    # servo.set_velocity_rpm(1)
    # time.sleep(3)
    pass

d.close()
