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
AXLE_TRACK_MM=157
SENSOR_TO_AXLE=60

WHEEL_DIAMETER_MM=89
AXLE_TRACK_MM=135
SENSOR_TO_AXLE=135
 
#drive motors
left_motor=Motor(Port.B, Direction.CLOCKWISE)
right_motor=Motor(Port.D, Direction.CLOCKWISE)
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER_MM, AXLE_TRACK_MM)

 

def move_straight(duration, speed_mm_s):
     robot.drive_time(speed_mm_s, 0, duration)
     robot.stop(stop_type=Stop.BRAKE)

#move_straight(duration=4000, speed_mm_s=300) 
## move_straight(5000, 300)

def move_straight(distance, speed_mm_s):
 
   # calculate the time (duration) for which robot needs to run
   duration = abs(int(1000 * distance / speed_mm_s))
   robot.drive_time(speed_mm_s, 0, duration)
   robot.stop(stop_type=Stop.BRAKE)

def turn(angle):
    robot.drive_time(0, angle, 1000)

#turn(angle=360)

def turn_arc(distance,angle):
   robot.drive_time(distance, angle, 1000)

#turn_arc(distance=200,angle=90)

color_sensor_left = ColorSensor(Port.S4)
color_sensor_right =None# ColorSensor(Port.S4)


def move_to_color(
    color_sensor,
    stop_on_color,
    speed_mm_s):
 
    robot.drive(speed_mm_s, 0)
    # Check if color reached.
    while color_sensor.color() != stop_on_color:
        wait(10)
    robot.stop(stop_type=Stop.BRAKE)

#move_to_color(color_sensor=color_sensor_right, stop_on_color=Color.RED, speed_mm_s=100)

touch_sensor=None# TouchSensor(Port.S3)

def move_to_wall(touch_sensor, speed_mm_s):
   robot.drive(speed_mm_s, 0)
   # Check if touch sensor pressed.
   while touch_sensor.pressed() == False :
       wait(10)
   robot.stop(stop_type=Stop.BRAKE)

#move_to_wall(touch_sensor=touch_sensor, speed_mm_s=100)

gyro=GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)

def turn_to_angle( gyro, target_angle):
 
    error = target_angle - gyro.angle()
    while ( abs(error) > 5):
        adj_angular_speed = error * 1.5
        robot.drive(0, adj_angular_speed)
        wait(100)
        error=target_angle - gyro.angle()
 
    robot.stop(stop_type=Stop.BRAKE)

#turn_to_angle( gyro=gyro, target_angle=65)

def calibrate_gyro(new_angle):
    current_speed=gyro.speed()
    current_angle=gyro.angle()
    wait(50)
    gyro.reset_angle(new_angle)
    wait(50)

calibrate_gyro(new_angle=0)

def turn_to_color(color_sensor, stop_on_color, angular_speed_deg_s):
   robot.drive(0, angular_speed_deg_s)
   # Check if color reached.
   while color_sensor.color() != stop_on_color:
       wait(10)
   robot.stop(stop_type=Stop.BRAKE)

#turn_to_color(color_sensor_left, Color.RED, 45)


def turn_to_color_right(color_sensor, stop_on_color, angular_speed_deg_s):
   robot.drive(0, angular_speed_deg_s)
   # Check if color reached.
   while color_sensor.color() != stop_on_color:
       wait(10)
   robot.stop(stop_type=Stop.BRAKE)

#turn_to_color(color_sensor_right, Color.RED, 45)




def move_crane_to_floor(crane_motor):
   crane_motor.run_until_stalled(-180, Stop.COAST, 35)
   move_crane_up( crane_motor, degrees = 5)

#move_crane_to_floor(crane_motor)


def log_string(message):
   print(message)
   brick.display.text(message)


#log_string('Running robot now done')

# Get wheel circumference
WHEEL_CIRCUM_MM=3.149*89
# 360 degrees -> WHEEL_CIRCUM_MM so   1 degree -> ?
DEGREES_PER_MM=360/WHEEL_CIRCUM_MM


def color_test(color_sensor, distance_mm, speed_mm_s):
 
   left_motor.reset_angle(0)
   motor_target_angle = int(DEGREES_PER_MM * distance_mm)
 
   while (abs(left_motor.angle()) < abs(motor_target_angle)):
       log_string('Color :' + str(color_sensor.color()) + ' Intense:' + str(color_sensor.reflection()))
 
       robot.drive(speed_mm_s, 0)
       wait(100)
 
   robot.stop(stop_type=Stop.BRAKE)

#color_test(color_sensor=color_sensor_left, distance_mm=150, speed_mm_s=50

def move_straight(duration, speed_mm_s):
     robot.drive_time(speed_mm_s, 0, duration)
     robot.stop(stop_type=Stop.BRAKE)

move_straight(duration=4000, speed_mm_s=300) 
move_straight(5000, 300)






