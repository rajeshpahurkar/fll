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


def traffic():
    testcode.move_straight(450, 300)

    # testcode.calibrate_gyro(0)    
    # traffic()
    # testcode.turn_to_angle( gyro, 0)
    # testcode.move_crane_to_floor(crane_motor)
    # testcode.turn(25)
    testcode.turn(5)


traffic()