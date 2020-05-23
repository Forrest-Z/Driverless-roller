from time import sleep
from iai.Controller import Controller
from iai.utilities import *
import re
import time

class Axis(object):
    def __init__(self, controller, number_in_controller=1):
        """
        @type controller: Controller
        """
        self._controller = controller
        self.axis_mode = 1 << (number_in_controller - 1)
        self.number = number_in_controller

    def go_home(self, search_speed=0, creep_speed=0):
        raise NotImplementedError

    def wait_for_going_home(self):
        wait(lambda: self.get_status()["axis status"]["Origin return"] == "Returning to Origin")

    def move_absolute(self, position, acceleration, deceleration, speed):
        raise NotImplementedError

    def wait_while_axis_is_using(self):
        wait(lambda: self.get_status()["axis status"]["Servo axis in use"])

    def get_status(self):
        raise NotImplementedError

    def send_command(self, command):
        self._controller.send_command(command)

    def receive_command(self):
        return self._controller.receive_command()


class CONAxis(Axis):
    def __init__(self, controller, number_in_controller=0):
        self._controller = controller
        self.number = number_in_controller + 1
        self.disable_PIO()
        self.servo = True
        pass

    def send_and_receive_command(self, command):
        x = {}
        x[0] = command
        def _impl():
            command = x[0]
            command = int_to_upper_hex(self.number, 2) + command
            self._controller.send_command(command)
            data = self.receive_command()
            payload = data[1:-4]
            checksum = int_to_upper_hex(self._controller.calculate_checksum(payload), 2)
            if checksum != data[-4:-2]:
                raise IOError()
            return data
        return retry(_impl, 3)

    def disable_PIO(self):
        self.send_and_receive_command("050427FF00")  # Disable PIO

    def get_status(self):
        raw_status = self.send_and_receive_command("039000000A")
        raw_status = raw_status[7:-4]
        raw_status = hex_to_int_list(raw_status)
        return raw_status

    def is_moving(self):
        raw_status = ""
        while raw_status == "":
            raw_status = self.send_and_receive_command("0390070001")
        raw_status = raw_status[7:-4]
        raw_status = hex_to_int_list(raw_status)
        return (raw_status[1] & 0b00100000) > 0

    def get_current_position(self):
        data = self.send_and_receive_command("0390000002")
        data = data[7:15]
        data = int(data,16)
        data = data * 0.01
        return data

    def wait_while_axis_is_using(self):
        while self.is_moving():
            sleep(0.001)


    @property
    def servo(self):
        raw_status = self.send_and_receive_command("0390080002")
        raw_status = raw_status[7:-4]
        raw_status = hex_to_int_list(raw_status)
        return (raw_status[-1] & 0b00000100) > 0

    @servo.setter
    def servo(self, value):
        if value:
            self.send_and_receive_command("050403FF00")
        else:
            self.send_and_receive_command("0504030000")

    def disable_safety_speed(self):
        self.send_and_receive_command("0504010000")

    def go_home(self):
        self.send_and_receive_command("05040B0000")
        self.send_and_receive_command("05040BFF00")
        pass

    def move_absolute(self, position, speed, acceleration):
        command = "10" + "9900" + "0009" + "12"  # The absolute command header
        command += int_to_upper_hex(int(position / 0.01), 8)
        command += int_to_upper_hex(int(0.1 / 0.01), 8)
        command += int_to_upper_hex(int(speed / 0.01), 8)
        command += int_to_upper_hex(int(acceleration / 0.01), 4)
        command += int_to_upper_hex(0, 4)
        command += int_to_upper_hex(0, 4)
        self.send_and_receive_command(command)


    def reset_error(self):
        self.send_and_receive_command("050407FF00")
        self.send_and_receive_command("0504070000")
