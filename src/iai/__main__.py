from time import sleep
from iai.Controller import CONController
from iai.Axis import CONAxis
from iai.Space import Space
from iai.utilities import *
import serial

def self_test():
    controller = CONController("/dev/ttyUSB1", 38400, 0.04)
    axis1 = CONAxis(controller, 0)
    axis1.reset_error()
    axis1.servo = True
    print(axis1.is_moving())
    axis1.go_home()
    print(axis1.is_moving())
    axis1.wait_while_axis_is_using()
    axis1.move_absolute(80, 30, 0.3)
    print(axis1.get_current_position())
    axis1.wait_while_axis_is_using()
    axis1.move_absolute(0, 180, 0.3)
    print(axis1.get_current_position())
    axis1.wait_while_axis_is_using()
    for i in range(10):
        axis1.move_absolute(0, 30, 0.3)
        print(axis1.get_current_position())
        axis1.wait_while_axis_is_using()
        axis1.move_absolute(50, 30, 0.3)
        print(axis1.get_current_position())
        axis1.wait_while_axis_is_using()
    axis1.move_absolute(0,100,0.3)
    axis1.wait_while_axis_is_using()    

if __name__ == "__main__":
    self_test()

