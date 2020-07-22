#!/usr/bin/env pybricks-micropython
import ev3dev.ev3 as ev3
import pybricks.nxtdevices as nxt
# from pybricks.nxtdevices import UltrasonicSensor
import pybricks.ev3devices as ev3dev
from pybricks.tools import wait, StopWatch
from pybricks import ev3brick as brick
from pybricks.ev3devices import UltrasonicSensor,TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color

# us = ev3.UltrasonicSensor()
# us = nxt.UltrasonicSensor(Port.S1)
message='Staring ultrasomic'

def test_sensor():
    touch_sensor= TouchSensor(Port.S2)
    while touch_sensor.pressed() == False :
        wait(10)
    brick.sound.beep(1100, 80, 7)

# us = ev3dev.UltrasonicSensor(Port.S1)
