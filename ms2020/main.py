#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor
 
 
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
 
 
WHEEL_DIAMETER_MM=89
AXLE_TRACK_MM=157
SOUND_VOLUME=7
 
left_motor=Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor=Motor(Port.B, Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
 
brick.sound.beep(700, 80, SOUND_VOLUME)
robot.drive_time(0, 1, 10000)
robot.stop(stop_type=Stop.BRAKE)
left_motor.run_angle(90,  180, Stop.BRAKE)