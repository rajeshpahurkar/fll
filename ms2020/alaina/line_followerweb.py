# line_follower.py
#!/usr/bin/env pybricks-micropython
# from pybricks.hubs import EV3Brick
import sys
import os
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                               InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.ev3devices import Motor
 
sys.path.append(os.path.abspath('../shared'))
import robot_setup
 
from robot_setup import left_motor
from robot_setup import right_motor
from robot_setup import robot
from robot_setup import rack_motor
from robot_setup import crane_motor
from robot_setup import gyro
from robot_setup import touch_sensor
from robot_setup import color_sensor_left
from robot_setup import color_sensor_right
from robot_setup import color_sensor_center
from robot_setup import touch_sensor
 
from robot_setup import SOUND_VOLUME
from robot_setup import WHEEL_DIAMETER_MM
from robot_setup import AXLE_TRACK_MM
from robot_setup import SENSOR_TO_AXLE
from robot_setup import WHEEL_CIRCUM_MM
from robot_setup import DEGREES_PER_MM
 
##### Do not change above this line ##########################################

# Import the EV3-robot library
#old import ev3dev.ev3 as ev3
#old from time import sleep


class LineFollower:
    # Constructor
    def __init__(self):
        #old self.btn = ev3.Button()
        self.shut_down = False

    # Main method
    def run(self):

        # sensors
        #old cs = ev3.ColorSensor();      assert cs.connected  # measures light intensity
        #old us = ev3.UltrasonicSensor(); assert us.connected  # measures distance
        cs = color_sensor_center
        #old cs.mode = 'COL-REFLECT'  # measure light intensity
        #old us.mode = 'US-DIST-CM'   # measure distance in cm

        # motors
        #old lm = ev3.LargeMotor('outB');  assert lm.connected  # left motor
        #old rm = ev3.LargeMotor('outC');  assert rm.connected  # right motor
        #old mm = ev3.MediumMotor('outD'); assert mm.connected  # medium motor
        lm = left_motor
        rm = right_motor

        speed = 360/4  # deg/sec, [-1000, 1000]
        dt = 500       # milliseconds
        stop_action = "coast"

        # PID tuning
        Kp = 1  # proportional gain
        Ki = 0  # integral gain
        Kd = 0  # derivative gain

        integral = 0
        previous_error = 0

        # initial measurment
        #old target_value = cs.value()
        target_value = color_sensor.reflection()

        # Start the main loop
        while not self.shut_down:

            # deal with obstacles
            #old distance = us.value() // 10  # convert mm to cm

            #old if distance <= 5:  # sweep away the obstacle
            #old     mm.run_timed(time_sp=600, speed_sp=+150, stop_action="hold").wait()
            #old     mm.run_timed(time_sp=600, speed_sp=-150, stop_action="hold").wait()

            # Calculate steering using PID algorithm
            #old error = target_value - cs.value()
            error = target_value - color_sensor.reflection()
            integral += (error * dt)
            derivative = (error - previous_error) / dt

            # u zero:     on target,  drive forward
            # u positive: too bright, turn right
            # u negative: too dark,   turn left

            u = (Kp * error) + (Ki * integral) + (Kd * derivative)

            # limit u to safe values: [-1000, 1000] deg/sec
            if speed + abs(u) > 1000:
                if u >= 0:
                    u = 1000 - speed
                else:
                    u = speed - 1000

            # run motors
            if u >= 0:
                #old lm.run_timed(time_sp=dt, speed_sp=speed + u, stop_action=stop_action)
                #old rm.run_timed(time_sp=dt, speed_sp=speed - u, stop_action=stop_action)
                #old sleep(dt / 1000)
                lm.run_time(speed=speed + u)
                rm.run_time(speed=speed - u)
                wait(dt)
            else:
                #old lm.run_timed(time_sp=dt, speed_sp=speed - u, stop_action=stop_action)
                #old rm.run_timed(time_sp=dt, speed_sp=speed + u, stop_action=stop_action)
                #old sleep(dt / 1000)
                lm.run_time(speed=speed - u)
                rm.run_time(speed=speed + u)
                wait(dt)

            previous_error = error

            # Check if buttons pressed (for pause or stop)
            #old if not self.btn.down:  # Stop
            #old     print("Exit program... ")
            #old     self.shut_down = True
            #old elif not self.btn.left:  # Pause
            #old     print("[Pause]")
            #old     self.pause()

    # 'Pause' method
#old     def pause(self, pct=0.0, adj=0.01):
#old         while self.btn.right or self.btn.left:  # ...wait 'right' button to unpause
#old             ev3.Leds.set_color(ev3.Leds.LEFT, ev3.Leds.AMBER, pct)
#old             ev3.Leds.set_color(ev3.Leds.RIGHT, ev3.Leds.AMBER, pct)
#old             if (pct + adj) < 0.0 or (pct + adj) > 1.0:
#old                 adj = adj * -1.0
#old             pct = pct + adj

#old         print("[Continue]")
#old         ev3.Leds.set_color(ev3.Leds.LEFT, ev3.Leds.GREEN)
#old         ev3.Leds.set_color(ev3.Leds.RIGHT, ev3.Leds.GREEN)


# Main function
if __name__ == "__main__":
    robot = LineFollower()
    robot.run()