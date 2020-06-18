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
 
SOUND_VOLUME=7
WHEEL_DIAMETER_MM=89
AXLE_TRACK_MM=157
SENSOR_TO_AXLE=60
 
 
#drive motors
left_motor=Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor=Motor(Port.C, Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)


def move_straight(duration, speed_mm_s):
    robot.drive_time(speed_mm_s, 0, duration)
    robot.stop(stop_type=Stop.BRAKE)

def move_straight(distance, speed_mm_s):

    # calculate the time (duration) for which robot needs to run
    duration = abs(int(1000 * max_distance / speed_mm_s))
    robot.drive_time(speed_mm_s, 0, duration)
    robot.stop(stop_type=Stop.BRAKE)

# move_straight(5000, 300)


def turn(angle):
    robot.drive_time(0, angle, 1000)
    robot.stop(stop_type=Stop.BRAKE)


## turn(60)

color_sensor_left = ColorSensor(Port.S2)
color_sensor_right = ColorSensor(Port.S3)

def move_to_color(
    color_sensor,
    stop_on_color,
    speed_mm_s):
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)


##move_to_color(color_sensor_left, COLOR.RED, 300)

obstacle_sensor = UltrasonicSensor(Port.S4)

def move_to_obstacle(
    obstacle_sensor,
    stop_on_obstacle_at,
    speed_mm_s):

    robot.drive(speed_mm_s, 0)
    # Check if obstacle too close.
    while obstacle_sensor.distance() > stop_on_obstacle_at:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)
    
# move_to_obstacle(obstacle_sensor, 50, 300)

gyro=GyroSensor(Port.S1)



def turn_to_angle( gyro, target_angle):

    error = target_angle - gyro.angle()
    while ( abs(error) > 5):
        adj_angular_speed = error * 1.5
        robot.drive(0, adj_angular_speed)
        wait(100)
        error=target_angle - gyro.angle()

    robot.stop(stop_type=Stop.BRAKE)


turn_to_angle( gyro, 65)


def calibrate_gyro(new_angle=0):
    current_speed=gyro.speed()
    current_angle=gyro.angle()
    wait(50)
    gyro.reset_angle(new_angle)
    wait(50)

calibrate_gyro(0)    
# turn_to_angle( gyro, 65)
# move_straight(5000, 300)

crane_motor=Motor(Port.D)

 
def move_crane_up( crane_motor, degrees):
   crane_motor.run_angle(90,  degrees, Stop.BRAKE)
 
def move_crane_down( crane_motor, degrees):
   crane_motor.run_angle(90,    -1 * degrees, Stop.BRAKE)


# move_crane_up(crane_motor, 60)

def move_crane_to_floor(crane_motor):
   crane_motor.run_until_stalled(-180, Stop.COAST, 50)
   move_crane_up( crane_motor, degrees = 5)

# move_crane_to_floor(crane_motor)


def turn_to_color(color_sensor, stop_on_color, angular_speed_deg_s):
 
    robot.drive(0, angular_speed_deg_s)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)

# turn_to_color(color_sensor_left, COLOR.RED, 45)

def turn_to_color_right(color_sensor, stop_on_color, angular_speed_deg_s):
 
    robot.drive(0, angular_speed_deg_s)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)

def turn_to_color_left(color_sensor, stop_on_color, angular_speed_deg_s):
 
    robot.drive(0, -1 * angular_speed_deg_s)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)


turn_to_color(color_sensor_left, COLOR.RED, 45)


# Used by line follower to align with the general direction of the line

def align_with_line_to_left(color_sensor, line_color):

    #Find left white border of line
    move_to_color( color_sensor, line_color, 300)
 
    #move forward half the length of tank and rotate
    move_straight( SENSOR_TO_AXLE, 300)    
    turn_to_color_right( color_sensor, line_color, 45) 

def align_with_line_to_right(color_sensor, line_color):

    #Find left white border of line
    move_to_color( color_sensor, line_color, 300)
 
    #move forward half the length of tank and rotate
    move_straight( SENSOR_TO_AXLE, 300)    
    turn_to_color_left( color_sensor, line_color, 45) 

align_with_line_to_left(color_sensor_left, COLOR.RED)



