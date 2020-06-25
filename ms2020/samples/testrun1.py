#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor
 
import testcode

from testcode import robot
from testcode import gyro
from testcode import crane_motor
from testcode import left_motor
from testcode import right_motor

def returnhome():
    testcode.turn_to_angle(gyro=gyro, target_angle=0)
    testcode.move_straight(distance=1000, speed_mm_s=-500)

def traffic():

    # testcode.move_straight(450, 400)
    # testcode.move_crane_to_floor(crane_motor)
    # testcode.move_crane_up(crane_motor, 30)
    # testcode.turn_to_angle( gyro, 20)
 
    testcode.drive_raising_crane(duration_ms=900, 
        robot_distance_mm=30, 
        robot_turn_angle=25, 
        crane_motor=crane_motor, 
        crane_angle=90)



def swing():
    # testcode.move_straight(distance=1000, speed_mm_s=500)

    testcode.move_straight_target_direction(gyro=gyro, distance_mm=1000, speed_mm_s=400, target_angle=0)

def tower():
    # testcode.move_crane_up( crane_motor=crane_motor, degrees=30)
    # testcode.drive_raising_crane(duration_ms=1000, 
    #     robot_distance_mm=100, 
    #     robot_turn_angle=-3, 
    #     crane_motor=crane_motor, 
    #     crane_angle=70)
    # testcode.drive_raising_crane(duration_ms=1000, 
    #     robot_distance_mm=100, 
    #     robot_turn_angle=0, 
    #     crane_motor=crane_motor, 
    #     crane_angle=-30)
    testcode.drive_raising_crane(duration_ms=1000, 
        robot_distance_mm=100, 
        robot_turn_angle=0, 
        crane_motor=crane_motor, 
        crane_angle=-50)
    # testcode.move_crane_down( crane_motor=crane_motor, degrees=30)
    # testcode.turn(-40)

def bridge():
     testcode.move_straight_target_direction(gyro=gyro, 
     distance_mm=550, 
     speed_mm_s=300, 
     target_angle=0)
   

testcode.calibrate_gyro(0)    
# traffic()
# swing()
# returnhome()
tower()
# bridge()