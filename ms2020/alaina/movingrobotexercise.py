#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor

#REMINDERS# 

#do not erase above#
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


gyro=GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)
# color_sensor_left = ColorSensor(Port.S1)
color_sensor_right = ColorSensor(Port.S4)


def move_to_color(
    color_sensor,
    stop_on_color,
    speed_mm_s):
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)

    robot.stop(stop_type=Stop.BRAKE)

def move_straight(distance, speed_mm_s):
    left_motor.reset_angle(0)
    motor_target_angle = int(DEGREES_PER_MM * distance)
    robot.drive(speed_mm_s, 0)

    while (abs(left_motor.angle()) < abs(motor_target_angle)):
        wait(20)

    robot.stop(stop_type=Stop.BRAKE)


def turn(angle):
    robot.drive_time(0, angle, 1000)


#move_to_color(color_sensor=color_sensor_right, stop_on_color=Color.RED, speed_mm_s=100)
#move_to_color(color_sensor=color_sensor_right, stop_on_color=Color.BLUE, speed_mm_s=150)
turn(90)
move_straight(400, 450)