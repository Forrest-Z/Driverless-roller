from .Axis import *
from .Controller import Controller


class Space(object):
    @staticmethod
    def init_helper(port, baudrate, axis_number, axis_class = CONAxis):
        controller = Controller(port, baudrate)
        axises = list(map(lambda x: axis_class(controller, x), axis_number))
        return Space(axises)

    def __init__(self, axises):
        self.axises = axises

        self.go_home = lambda: list(map(lambda x: x.go_home(), self.axises))
        self.wait = lambda: list(map(lambda x: x.wait_while_axis_is_using(), self.axises))

    def move(self, points, speed=100, accelerate=0.3, decelerate=0.3):
        list(map(lambda x, y: x.move_absolute(y, accelerate, decelerate, speed), self.axises, points))
