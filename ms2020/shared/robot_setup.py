#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor

 
SOUND_VOLUME=7
WHEEL_DIAMETER_MM=89
AXLE_TRACK_MM=135
SENSOR_TO_AXLE=118

# Get wheel circumference
WHEEL_CIRCUM_MM=3.149*89
# 360 degrees -> WHEEL_CIRCUM_MM so   1 degree -> ?
DEGREES_PER_MM=360/WHEEL_CIRCUM_MM
 
#drive motors
left_motor=Motor(Port.C, Direction.CLOCKWISE)
right_motor=Motor(Port.D, Direction.CLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)
crane_motor=Motor(Port.B, Direction.CLOCKWISE, [8,24])
rack_motor=None ####  Motor(Port.A, Direction.CLOCKWISE)

gyro=GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)
color_sensor_left = None ### ColorSensor(Port.S1)
color_sensor_right = ColorSensor(Port.S4)
touch_sensor= TouchSensor(Port.S3)
