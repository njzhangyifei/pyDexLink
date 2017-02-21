```
Title:        pyDexlink Tutorial (Simplified Chinese)
Date:         2016-06-22
Version:      0.0.1 (Subject to Change)
Prepared For: github.com/njzhangyifei/pyDexLink
```

# 导航

协议标准请见 [DexLink 协议标准](https://static.frank.moe/dexlink_docs/)

舵机硬件相关问题请见 [DexLink Servo Hardware Documentation](https://github.com/njzhangyifei/pyDexLink/tree/master/docs_hardware)

疑难解答请见 [pyDexlink 疑难解答](https://github.com/njzhangyifei/pyDexLink/tree/master/tutorial/troubleshoot.md)

# pyDexlink 使用教程

pyDexlink 是Dexlink协议的 **python3**实现。

## Step 0. 安装 pyDexlink

**请确认您在使用 python3，下列步骤中的 pip 指的是 python3 所附带的 pip。**

Linux 用户请确认您有权限访问 `/dev/tty*`。

### pyDexlink 依赖库
- pySerial

### 安装方法
Windows 用户请确认您的PATH环境变量已经包含python3及其pip
```sh
pip install pyserial
cd pyDexlink               # make sure you are in the correct directory
python3 setup.py install
```

## Step 1. "import pyDexlink"

运行以下 python 代码

```python
from dexlink.dexlink_packet import *
from dexlink.dexlink_common import *
from dexlink.dexlink_operand import DexLinkOperand
from dexlink.dexlink_serial import DexLinkSerial
from dexlink.dexlink_device import DexLinkServo
```

## Step 2. 打开串口

通过创建一个DexLinkSerial的实例，然后调用open方法。您可以创建一个用于与舵机通讯
的DexLinkSerial对象。

 - Example 1 - 打开 COM3，默认波特率 115200

```python
d = DexLinkSerial('COM3')         # COM3, 115200
d.open()
```

 - Example 2 - 打开 COM3，波特率 9600

```python
d = DexLinkSerial('COM3', 9600)   # COM3, 9600
d.open()
```

 - Example 3 - 打开 /dev/ttyUSB0，默认波特率 115200

```python
d = DexLinkSerial('/dev/ttyUSB0') # /dev/ttyUSB0, 115200
d.open()
```

## Step 3. 扫描总线上的舵机

如果您不知道舵机地址，pyDexlink 可以自动帮您发现总线上的舵机。

由于485总线的限制，一次性扫描整个总线会需要一些时间。
我们建议您一次性只扫描一个范围的地址。

关于设备地址的定义，请参阅 DexLink 协议文档。

以下的例子都需要已经打开的 DexLinkSerial 对象，请参见 Step 2。

 - Example 1 - 扫描总线上地址 0x01 (1) 到地址 0x14 (20) 的舵机设备

```python
# scan throw address 1 through 20
device_list = d.scan(DexLinkServo, 1, 20)
```

## (Optional) Step 4. 手动生成ping数据包并检测舵机是否存在

如果您已经知道舵机的地址，您可以直接使用舵机地址来生成 ping 数据包来检测舵机是否在线。

 - Example 1 - 生成目标地址为 0x01 的 ping 数据包，然后发送。

```python
# Example for generating ping packets
ping_packet = DexLinkRequestPacket(address=0x01,                 # address
                                   operand=DexLinkOperand.ping,  # operand
                                   payload=[0x00])               # payload

# Example for Ping-ing device 
device_alive = False
try:
    resp = d.comm(ping_packet)     # Send the packet and wait for response
    if resp:
        print(str(resp))
        device_alive = True        # Ping success
except DexLinkTimeoutException:
    print("Timeout!")              # Timeout, servo is not alive
except DexLinkSerialException:
    print("Serial Exception!")     # Serial port error
```

## Step 5: 获取舵机实例

从 Step 3 中，我们获取了总线上的舵机列表 `device_list` 。我们从中取出一个舵机
实例就可以控制舵机设备了。

您可以将 `device_list` 打印出来以查看每个舵机的信息

 - Example 1 - 取出列表中第一个舵机，存为 `servo` 。

```python
# We've got a servo alive
servo = device_list[0]  # type: DexLinkServo
print(str(servo))       # print servo information
```

## Step 6: 打开/关闭舵机扭矩输出

 - Example 1 - 打开舵机扭矩输出

```python
# Enable Torque Output
servo.config_torque_output(True)
print(str(servo.torque_output))  # print torque output status
```

 - Example 2 - 关闭舵机扭矩输出

```python
# Disable Torque Output
servo.config_torque_output(False)
print(str(servo.torque_output))  # print torque output status
```


## Step 7: 改变舵机控制模式

 - Example 1 - 设定为位置控制模式

```python
# Config the servo into position control mode
servo.config_mode_selection(servo.Mode.position_control)
print(str(servo.mode_selection.name))     # print current control mode
```

 - Example 2 - 设定为连续旋转控制模式

```python
# Config the servo into cont. rotation control mode
servo.config_mode_selection(servo.Mode.continuous_rotation)
print(str(servo.mode_selection.name))     # print current control mode
```

## Step 8: 控制舵机！

 - **位置控制模式**，请先启用舵机扭矩输出，并设置为位置控制模式

顺时针转向**到**0度

```python
resp = servo.set_position_deg(0, direction=servo.Direction.cw)
```

顺时针转向**到**20度

```python
resp = servo.set_position_deg(20, direction=servo.Direction.cw)
```

逆时针顺时针转向**到**80度

```python
resp = servo.set_position_deg(80, direction=servo.Direction.ccw)
```

逆时针转向**到**0度

```python
resp = servo.set_position_deg(0, direction=servo.Direction.ccw)
```

向更近方向转向**到**30度

```python
resp = servo.set_position_deg(30)
```

向更近方向转向**到**0度

```python
resp = servo.set_position_deg(0)
```

设定位置控制时运行的速度上限到 10 rpm

```python
resp = servo.set_position_velocity_rpm(10) # enable velocity limit
```

设定位置控制时运行的速度上限到 30 deg/s

```python
resp = servo.set_position_velocity_deg_per_s(10) # enable velocity limit
```

取消位置控制时的运行速度上限

```python
resp = servo.set_position_velocity_rpm(0)  # disable velocity limit
```

 -  **连续旋转模式**，请先启用舵机扭矩输出，并设置为连续旋转控制模式

设定旋转速度为 10 rpm

```python
servo.set_velocity_rpm(10)
```

设定旋转速度为 30 deg/s

```python
servo.set_velocity_deg_per_s(30)
```

停止旋转

```python
servo.set_velocity_rpm(0)
```

## Step 9: 读取舵机传感器数值

读取舵机当前的位置，速度，电压以及里程计。单位分别为 deg，rpm，volt，deg。

```python
servo.update_sensor_readings()
position = servo.get_sensor_readings_str(servo.Sensor.position)
velocity = servo.get_sensor_readings_str(servo.Sensor.velocity)
voltage  = servo.get_sensor_readings_str(servo.Sensor.voltage)
odometer = servo.get_sensor_readings_str(servo.Sensor.odometer)
```

## Step 10: 控制舵机的LED

打开 LED

```python
# Example for LED On/off
servo.config_led(True)     # LED on
```

关闭 LED

```python
# Example for LED On/off
servo.config_led(False)     # LED off
```

## (Important!) Step 11: 关闭串口（重要！）

关闭串口

```
d.close()
```

## 其他

请参阅 `dexlink/dexlink_device` 文件中的 `DexLinkServo` 类。

文件内有对各个方法的注释。


## LICENSE
Yifei Zhang (c) Copyright 2016, All Rights Reserved

