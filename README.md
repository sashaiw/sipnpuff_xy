# sipnpuff_xy
This package allows you to control two axes with an [openSip+Puff](https://github.com/lazzuri/openSip-Puff) sensor.
It is intended for control of a two-axis gimbal with a mounted laser.

## Setup
`rossipnpuf.ino` will need to be flashed to the 32u4 on the openSip+Puff board. If it is your first time using the
board, it is likely necessary to first flash the Arduino Leonardo bootloader to the chip using an external AVR
programmer connected to the built-in ICSP header. Once this program is uploaded, the board will utilize 
[http://wiki.ros.org/rosserial](http://wiki.ros.org/rosserial) to publish a single Float64 topic, `sipnpuff_sensor`.
You will need to install rosserial and run a rosserial node. For example:
``rosrun rosserial_python serial_node.py /dev/ttyACM0``

`sipnpuff.py`, when run, will subscribe to `sipnpuff_sensor`, and publish two other topics, by default they are
`/pan_controller/command` and `/tilt_controller/command`. The X topic will be a value between -pi to pi radians, and
the Y topic will be a value between -1 and 1 radians.

`laser_visualizer.py` can also be run in lieu of an actual gimbal to visualize the position of the laser. It relies on
matplotlib for the visualization.

`sipnpuff.py` uses several rosparam parameters:
 + `zero`: The nominal value of the sensor, when only atmospheric pressure is registered. Default: 377
 + `deadzone`: The deadzone of the sensor. Default: 3
 + `timeout`: The delay before modes are changed in seconds. It is expected that this can be raised as the user becomes more
 skilled. Default: .2
 + `movement_rate`: The rate at which the laser will move. This can also be increased as the user builds skill with the
 sensor. Default: 1
 + `sel_topic`: The topic that the selection status will be published to. Default: /sipnpuff/select
 + `x_topic`: The topic that the gimbal's X position will be published to. Default: /pan_controller/command
 + `y_topic`: The topic that the gimbal's Y position will be published to. Default: /tilt_controller/command

## Usage:
In order to provide control of two axes with only a single axis, `sipnpuff.py` will allow selection of several modes:
 + **Menu** mode will be activated if no activity is read from the sensor during the timeout duration. This is the mode
 that you can select other modes from.
 + **Y adjustment** mode is activated from **menu** mode with a light puff. This mode allows you to adjust the gimbal in the
 Y direction using both sipping and puffing.
 + **X adjustment** mode is activated from **menu** mode with a light sip. This mode allows you to adjust the gimbal in the
 X direction using both sipping and puffing.
 + **Select** is activated with a hard puff.