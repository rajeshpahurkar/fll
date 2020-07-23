#!/usr/bin/env pybricks-micropython
# import ev3dev.ev3 as ev3
import pybricks.nxtdevices as nxt
# from pybricks.nxtdevices import UltrasonicSensor
import pybricks.ev3devices as ev3dev
from pybricks.tools import wait, StopWatch
from pybricks import ev3brick as brick
from pybricks.ev3devices import UltrasonicSensor,TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color

# us = ev3.UltrasonicSensor()
us = nxt.UltrasonicSensor(Port.S1)
color_sensor_center = None##ColorSensor(Port.S4)
touch_sensor= None## TouchSensor(Port.S2)
message='Staring ultrasomic'
brick.sound.beep(1100, 80, 7)
print(message)
brick.display.text(message)

def test_sensor():
    # while touch_sensor.pressed() == False :
    while us.distance() > 10:
        message='Distance ' + str(us.distance())
        print(message)
        brick.display.text(message)
        wait(1000)

# us = ev3dev.UltrasonicSensor(Port.S1)
test_sensor()
