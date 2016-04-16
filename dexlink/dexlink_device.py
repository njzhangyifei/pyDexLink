"""
File: dexlink_device.py
Author: Yifei Zhang
Email: njzhangyifei@gmail.com
Github: https://github.com/njzhangyifei
Description: Contains all the definitions for the DexLinkDevice and DexLinkServo
"""

import math
import time

from .util import byte_bit_util
from .dexlink_common import *
from .dexlink_operand import *
from .dexlink_packet import DexLinkRequestPacket, \
    DexLinkResponsePacket

DEG_P_S2RPM = (1.0 / 360.0) * 60.0


class DexLinkDevice(object):
    @staticmethod
    def ping_address(dexlink_serial, address):
        """
        Ping a specific address with a DexLinkSerial object
        Subclass should override this
        :param dexlink_serial: the DexLinkSerial with which to communicate
        :type dexlink_serial: DexLinkSerial
        :param address: the address of device
        :return: None
        """
        return None

    def __init__(self, dexlink_serial, address, model, unique_id):
        """
        Initialize the DexLinkDevice
        :param dexlink_serial:
        :type dexlink_serial: DexLinkSerial
        :param address:
        :param model:
        :param unique_id:
        :return:
        """
        self.broadcast_address = 0xFF
        self._serial = dexlink_serial
        self.address = address
        self.model = model
        self.id = unique_id

    def comm(self, operand, payload, broadcast=False):
        """
        Communicate with the device with operand and payload
        :param broadcast: if we are broadcasting
        :param operand: the Operand
        :param payload: the payload/parameter
        :return: the DexLinkResponsePacket from the device
        :rtype: DexLinkResponsePacket
        """
        if not broadcast:
            packet = DexLinkRequestPacket(self.address, operand, payload)
            return self._serial.comm_retry(packet)
        else:
            packet = DexLinkRequestPacket(self.broadcast_address, operand,
                                          payload)
            self._serial.send_request(packet)

    def __str__(self):
        return str(self.model) + " @ [" + hex(self.address) + "] with " \
                                                              "unique id " + \
               byte_bit_util.byte_list_to_string(self.id)


class DexLinkServo(DexLinkDevice):
    class Status(Enum):
        ok = 8
        packet_invalid = 7
        other_error = 5
        flash_error = 4
        encoder_error = 3
        overcurrent = 2
        overvoltage = 1
        overheat = 0

    class Mode(Enum):
        idle = 0x00
        position_control = 0x01
        continuous_rotation = 0x02
        openloop = 0x0F

    class Direction(Enum):
        nearest = 0x00
        clockwise = 0x01
        cw = 0x01
        counterclockwise = 0x02
        ccw = 0x02

    class Motor(Enum):
        motor_a = 1
        motor_b = 0

    class Sensor(Enum):
        position = 0
        velocity = 1
        odometer = 2
        current = 3
        temperature = 4
        voltage = 5

    class Setpoint(Enum):
        openloop_a = 0
        openloop_b = 1
        position = 2
        velocity = 3

    motors_bound = {
        'min': 0,
        'max': 1000
    }

    conversion_factor = {
        'position': (360.0 / 4095),
        # 'current': (8.2 / 4095),
        'current': (3.3 / 4095 / 50 * 100),
        'velocity': (360.0 / 4095) * 25 * DEG_P_S2RPM,  # 25 hz
        'odometer': (360.0 / 4095),
        'temperature': (4095.0 / 4095),
        # 'voltage': (4095.0 / 4095),
        'voltage': (3.3 * 5.7 / 4095.0),
    }
    sensor_readings_unit = {
        'position': 'deg',
        'current': 'A',
        'temperature': 'C',
        'voltage': 'V',
        'velocity': 'rpm',
        'odometer': 'deg',
    }

    @staticmethod
    def ping_address(dexlink_serial, address):
        """
        Ping a specific address with a dexlink_serial
        :param dexlink_serial: the DexLinkSerial with which to communicate
        :type dexlink_serial: DexLinkSerial
        :param address: the address of the device to ping
        :return: a dictionary containing unique id of dexervos and model/version
        :rtype: dict
        """
        ping_packet = DexLinkRequestPacket(address, DexLinkOperand.ping, [0x01])
        try:
            response = dexlink_serial.comm_retry(ping_packet)
        except DexLinkTimeoutException:
            return None
        if response:
            unique_id = response.payload()[2:14]
            model = "DexLinkServo"
            return {'unique_id': unique_id, 'model': model}
        return None

    def __init__(self, dexlink_serial, address, model, unique_id):
        """
        Initialize the DexLinkServo with a DexLinkSerial and address/model
        :param dexlink_serial: the DexLinkSerial with which to communicate
        :type dexlink_serial: DexLinkSerial
        :param address: the address of the servo
        :param model: the model of the servo
        :param unique_id: the unique id of the servo
        """
        self.status = []
        self.status_update_callback = []
        self.torque_output = False
        self.mode_selection = self.Mode.idle
        self.last_sensor_update = 0
        self.last_motors_update = 0
        self.led_status = False
        self.sensor_readings_raw = {}
        self.setpoint_raw = {}
        self.sensor_readings = {}
        self.setpoint = {}
        self.motors = {}
        self.motors_raw = {}
        for key in self.Sensor:
            self.sensor_readings_raw[key.name] = 0
            self.sensor_readings[key.name] = 0
        for key in self.Motor:
            self.motors[key.name] = 0
            self.motors_raw[key.name] = 0
        super().__init__(dexlink_serial, address, model, unique_id)

    def status(self):
        """
        Get the status of the servo
        :return: a list containing the status of the servo
        """
        return self.status

    def has_status(self, status=None):
        """
        Check if a given status exists
        :param status: the status to be checked
        :return: True or False
        """
        if not status:
            return status in self.status

    def register_status_update_callback(self, callback):
        """
        Register the status update callback function which will be called
        synchronously when a new status byte is received
        :param callback: the callback function
        :return: None
        """
        if not (callback in self.status_update_callback):
            self.status_update_callback.append(callback)

    def deregister_status_update_callback(self, callback):
        """
        Deregister the callback function
        :param callback: function to be deregistered
        :return: None
        """
        if callback in self.status_update_callback:
            self.status_update_callback.remove(callback)

    def _match_dict_with_enum_key(self, enum_dict, enum_list, enum_type):
        if not enum_list:
            return enum_dict
        if isinstance(enum_list, list):
            d = {}
            for element in enum_list:
                r = self._match_dict_with_enum_key(enum_dict, element,
                                                   enum_type)
                d.update(r)
            return d
        if isinstance(enum_list, str):
            return {enum_list: enum_dict[enum_list]}
        if isinstance(enum_list, enum_type):
            return {enum_list.name: enum_dict[enum_list.name]}
        return None

    def get_motor_raw(self, motor_type=None):
        """
        Get the raw status of motor
        :param motor_type: the motor channel to read
        :return: a dictionary containing the motor status with motor_type as
        keys and int as values
        """
        return self._match_dict_with_enum_key(self.motors_raw,
                                              motor_type, self.Sensor)

    def get_sensor_readings(self, sensor_type=None):
        """
        Get the human-readable form of sensor reading, scaled
        :param sensor_type: the sensor to read
        :return: a dictionary containing the sensor status with sensor_type as
        keys and float as values
        """
        return self._match_dict_with_enum_key(self.sensor_readings,
                                              sensor_type, self.Sensor)

    def get_sensor_readings_raw(self, sensor_type=None):
        """
        Get the raw form of sensor reading, unscaled
        :param sensor_type: the sensor to read
        :return: a dictionary containing the sensor status with sensor_type as
        keys and int as values
        """
        return self._match_dict_with_enum_key(self.sensor_readings_raw,
                                              sensor_type, self.Sensor)

    def get_sensor_readings_str(self, sensor_type=None):
        """
        Get the human-readable form of sensor reading in string
        :param sensor_type: the sensor to read
        :return: a dictionary containing the sensor status with sensor_type as
        keys and str as values
        """
        d = self._match_dict_with_enum_key(self.sensor_readings,
                                           sensor_type, self.Sensor)
        return {k: '{:.2f} {:s}'.format(v, self.sensor_readings_unit[k]) for
                k, v in d.items()}

    def comm(self, operand, payload, broadcast=False):
        """
        Communicate with this device using the operand and payload
        :param broadcast:
        :param operand: the Operand field in the DexLinkRequestPacket
        :param payload: the payload field in the DexLinkRequestPacket
        :return: the DexLinkResponse packet read from the device
        :raises DexLinkSerialException: when serial error occurs
        :raises DexLinkTimeoutException: when timeout occurs
        """
        if not broadcast:
            response = super().comm(operand, payload, broadcast)
            self.update_status(response.status())
            return response
        else:
            super().comm(operand, payload, broadcast)

    def update_status(self, status_byte):
        """
        Update the status field of the device using the status byte
        :param status_byte: the original status byte from the
        DexLinkResponsePacket
        :return: None
        """
        new_status = []
        for e in self.Status:
            if status_byte & (1 << e.value):
                # append error to list
                new_status.append(e)
        if len(new_status) == 0:
            new_status.append(self.Status.ok)
        self.status = new_status
        for f in self.status_update_callback:
            f(self.status)

    def config_torque_output(self, new_state):
        """
        Configure the torque output status
        :param new_state: True => enable, False => disable
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        if type(new_state) != bool and type(new_state) != int:
            raise DexLinkInvalidOperationException(
                "torque output can only be bool or int")
        param = 0
        if type(new_state) == bool and new_state:
            param = 1
        if type(new_state) == int and new_state > 0:
            param = 1
        response = self.comm(DexLinkOperand.config_torque_output, [param])
        self.torque_output = (param == 1)
        return response

    def config_mode_selection(self, new_mode):
        """
        Switch the control mode of the servo
        :param new_mode: the mode to switch to
        :type new_mode: DexLinkServo.Mode
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        if type(new_mode) != self.Mode:
            raise DexLinkInvalidOperationException(
                "torque output can only be DexLinkServo.Mode")
        response = self.comm(DexLinkOperand.config_mode_selection,
                             [new_mode.value])
        self.mode_selection = new_mode
        return response

    def config_led(self, led_status):
        """
        Configure the LED status
        :param led_status: True => Enable, False => Disable
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        if type(led_status) != bool:
            raise DexLinkInvalidOperationException('led status can only be '
                                                   'bool')
        param = [0x00, 0x01 if led_status else 0x00]
        response = self.comm(DexLinkOperand.config_led, param)
        self.led_status = led_status
        return response

    def update_torque_output(self):
        """
        Update the torque_output field with the device
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        response = self.comm(DexLinkOperand.read_torque_output, [])
        temp = response.read_int(2, 3)
        self.torque_output = (temp == 1)
        return response

    def update_mode_selection(self):
        """
        Update the mode_selection field
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        response = self.comm(DexLinkOperand.read_mode_selection, [])
        temp = response.read_int(2, 3)
        self.mode_selection = self.Mode(temp)
        return response

    def update_led_status(self):
        """
        Update the LED status field
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        response = self.comm(DexLinkOperand.read_led, [0x00])
        temp = response.read_int(2, 3)
        self.led_status = False if temp == 0 else True
        return response

    def update_sensor_readings_raw(self):
        """
        Update the sensor_readings_raw field
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        param = 0
        for key in self.Sensor:
            param |= (1 << key.value)
        response = self.comm(DexLinkOperand.read_sensor, [param])
        # first two bytes are [ request_operand, status_byte ]
        self.sensor_readings_raw[self.Sensor.voltage.name] = \
            response.read_int(2, 4)
        self.sensor_readings_raw[self.Sensor.temperature.name] = \
            response.read_int(4, 6)
        self.sensor_readings_raw[self.Sensor.current.name] = \
            response.read_int(6, 8)
        self.sensor_readings_raw[self.Sensor.odometer.name] = \
            response.read_int(8, 12, signed=True)
        self.sensor_readings_raw[self.Sensor.velocity.name] = \
            response.read_int(12, 14, signed=True)
        self.sensor_readings_raw[self.Sensor.position.name] = \
            response.read_int(14, 16)
        return response

    def update_sensor_readings(self):
        """
        Update the sensor_readings field and the sensor_readings_raw field
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        response = self.update_sensor_readings_raw()
        self.sensor_readings.update(
            {k: self.convert_from_lsb(v, k) for k, v in
             self.sensor_readings_raw.items()})
        self.last_sensor_update = time.time()
        return response

    def update_motors_raw(self):
        """
        Update the motors_raw field
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        param = 0
        for key in self.Motor:
            param |= (1 << key.value)
        response = self.comm(DexLinkOperand.read_motor, [param])
        # first two bytes are [ request_operand, status_byte ]
        self.motors_raw[self.Motor.motor_a.name] = response.read_int(2, 4)
        self.motors_raw[self.Motor.motor_b.name] = response.read_int(4, 6)
        self.last_motors_update = time.time()
        return response

    def update_motors(self):
        """
        Update the motors_raw and the motors field
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        self.update_motors_raw()
        self.motors[self.Motor.motor_a.name] = \
            self.motors_raw[self.Motor.motor_a.name] / \
            float(self.motors_bound['max'] - 1)
        self.motors[self.Motor.motor_b.name] = \
            self.motors_raw[self.Motor.motor_b.name] / \
            float(self.motors_bound['max'] - 1)

    def set_openloop_ab_raw(self, openloop_a, openloop_b, broadcast=False):
        """
        Set the openloop_a and openloop_b value for openloop control
        :param openloop_a: the value for motor channel a
        :param openloop_b: the value for motor channel b
        :param broadcast: if this is a broadcast message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        openloop_a = int(openloop_a)
        openloop_a = min(self.motors_bound['max'] - 1, openloop_a)
        openloop_a = max(self.motors_bound['min'], openloop_a)
        openloop_b = int(openloop_b)
        openloop_b = min(self.motors_bound['max'] - 1, openloop_b)
        openloop_b = max(self.motors_bound['min'], openloop_b)
        param = [(openloop_a >> 8), (openloop_a & 0xFF),
                 (openloop_b >> 8), (openloop_b & 0xFF)]
        return self.comm(DexLinkOperand.openloop_ab, param, broadcast)

    def set_position_deg(self, deg,
                         direction=Direction.nearest, broadcast=False):
        """
        Set the position setpoint to a given position in degrees
        :param direction: in which direction should the servo rotate
        :param deg: the position in degrees
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        if deg < 0 or deg >= 360:
            raise DexLinkInvalidOperationException(
                "Position can only be [0 - 360) degrees")
        raw = self.convert_to_lsb(deg, "position")
        return self.set_position_raw(raw, direction, broadcast)

    def set_position_rad(self, rad,
                         direction=Direction.nearest, broadcast=False):
        """
        Set the position setpoint to a given position in radians
        :param direction: in which direction should the servo rotate
        :param rad: the position in radians
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        return self.set_position_deg(math.degrees(rad), direction, broadcast)

    def set_position_velocity_rpm(self, velocity, broadcast=False):
        raw = self.convert_to_lsb(velocity, "velocity")
        return self.set_position_velocity_raw(raw, broadcast)

    def set_position_velocity_deg_per_s(self, velocity, broadcast=False):
        return self.set_position_velocity_rpm(velocity * DEG_P_S2RPM,
                                              broadcast)

    def set_position_velocity_rad_per_s(self, velocity, broadcast=False):
        return self.set_position_velocity_deg_per_s(math.degrees(velocity),
                                                    broadcast)

    def set_position_velocity_raw(self, raw, broadcast=False):
        if raw > 2047:
            raw = 2047
        elif raw < -2048:
            raw = -2048
        param = [(raw >> 8), (raw & 0xFF)]
        return self.comm(DexLinkOperand.position_velocity_request, param,
                         broadcast)

    def set_position_raw(self, raw,
                         direction=Direction.nearest, broadcast=False):
        """
        Set the position setpoint to a given position in raw data
        :param direction: in which direction should the servo rotate
        :param raw: the position in radians
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        if raw > 4095:
            raw = 4095
        elif raw < 0:
            raw = 0
        param = [direction.value, (raw >> 8), (raw & 0xFF)]
        return self.comm(DexLinkOperand.position_direction_request,
                         param, broadcast)

    def set_velocity_rpm(self, velocity, broadcast=False):
        """
        Set the velocity setpoint to a given velocity in degrees per second
        :param velocity: the velocity in degrees per second
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        raw = self.convert_to_lsb(velocity, "velocity")
        return self.set_velocity_raw(raw, broadcast)

    def set_velocity_deg_per_s(self, velocity, broadcast=False):
        """
        Set the velocity setpoint to a given velocity in degrees per second
        :param velocity: the velocity in degrees per second
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        return self.set_velocity_rpm(velocity * DEG_P_S2RPM, broadcast)

    def set_velocity_rad_per_s(self, velocity, broadcast=False):
        """
        Set the velocity setpoint to a given velocity in radians per second
        :param velocity: the velocity in radians per second
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        return self.set_velocity_deg_per_s(math.degrees(velocity), broadcast)

    def set_velocity_raw(self, raw, broadcast=False):
        """
        Set the velocity setpoint to a given velocity in raw data
        :param raw: the velocity in raw data
        :param broadcast: the flag whether to broadcast this message
        :return: the DexLinkResponsePacket received
        :rtype: DexLinkResponsePacket
        """
        if raw > 2047:
            raw = 2047
        elif raw < -2048:
            raw = -2048
        param = [(raw >> 8), (raw & 0xFF)]
        return self.comm(DexLinkOperand.cont_rot_velocity_request, param,
                         broadcast)

    def wait(self, broadcast=False):
        """
        Enable the wait mode for the servo/broadcast
        :param broadcast: if we are broadcasting
        :return: the response Packet or None if we are broadcasting
        """
        return self.comm(DexLinkOperand.wait, [], broadcast)

    def execute(self, broadcast=False):
        """
        Enable the execute mode for the servo/broadcast
        :param broadcast: if we are broadcasting
        :return: the response Packet or None if we are broadcasting
        """
        return self.comm(DexLinkOperand.execute, [], broadcast)

    def convert_from_lsb(self, data, data_type_name):
        """
        Convert a data type from raw form
        :param data: the data in raw form
        :param data_type_name: the name of the data type
        :return: the data converted
        """
        if data_type_name in self.conversion_factor.keys():
            return float(data) * self.conversion_factor[data_type_name]
        else:
            raise DexLinkInvalidOperationException("Not a valid data type")

    def convert_to_lsb(self, data, data_type_name):
        """
        Convert a data type to raw form
        :param data: the data
        :param data_type_name: the name of the data type
        :return: the data converted
        """
        if data_type_name in self.conversion_factor.keys():
            return int(float(data) / self.conversion_factor[data_type_name])
        else:
            raise DexLinkInvalidOperationException("Not a valid data type")
