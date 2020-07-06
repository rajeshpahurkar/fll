#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor

##### Do not change above this line ##########################################

WHEEL_DIAMETER_MM=89
AXLE_TRACK_MM=135
 
 
#drive motors
left_motor=Motor(Port.B, Direction.CLOCKWISE)
right_motor=Motor(Port.C, Direction.CLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)

def move_straight(duration, speed_mm_s):
    robot.drive_time(speed_mm_s, 0, duration)
    robot.stop(stop_type=Stop.BRAKE)

move_straight(duration=5000, speed_mm_s=300) 
## move_straight(5000, 300)

def turn(angle):
    robot.drive_time(0, angle, 1000)
    turn(angle=60)
    ## turn(60)

def turn_arc(distance,angle):
   robot.drive_time(distance, angle, 1000)
   turn_arc(distance=200,angle=90)