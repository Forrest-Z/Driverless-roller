from iai.utilities import *
import platform
import sys

is_py3 = sys.version[0] == '3'

class Controller(object):
    def __init__(self, port=None, baudrate=None, timeout=5, station=0x99):
        self.station = station
        self.is_ironpython = platform.python_implementation() == "IronPython"
        if port and baudrate:
            if self.is_ironpython:
                import clr
                clr.AddReference('System')
                import System
                self._serial = System.IO.Ports.SerialPort(port)
                self._serial.BaudRate = baudrate
                self._serial.DataBits = 8
                self._serial.Open()
                self._serial.write = self._serial.Write
                self._serial.readline = self._serial.ReadLine
            else:
                import serial
                self._serial = serial.Serial(port, baudrate, timeout=timeout)
            self.send_init_command()

    def add_controller_info(self, command):
        """
        :type command: str
        """
        return command.replace("[station]", int_to_upper_hex(self.station, 2))

    def send_init_command(self):
        self.send_command("?[station]VERM0")
        self.receive_command()

    def send_command(self, command):
        command = self.add_controller_info(command)
        command += int_to_upper_hex(calculate_check_sum(command), 2)
        command += "\r\n"
        command = command.encode()
        self._serial.write(command)

    def receive_command(self, timeout=None):
        if timeout:
            self._serial.timeout = timeout
        received = self._serial.readline()

        if received[-1] != '\n' and received[-1] != 10:
            raise IOError()

        if is_py3:
            return received.decode("UTF-8")
        else:
            return str(received)

    def close(self):
        self._serial.close()

class CONController(Controller):
    def __init__(self, port=None, baudrate=38400, timeout=0.3):
        super(CONController, self).__init__(port, baudrate, timeout)

    def calculate_checksum(self, command):
        raw_check_sum = 0
        for i in range(int(len(command) / 2)):
            raw_check_sum += int(command[2*i] + command[2*i+1] ,16)
        raw_check_sum = 0x100 - (raw_check_sum & 0xff)
        return raw_check_sum

    def send_command(self, command):
        raw_check_sum = self.calculate_checksum(command)
        command = ":" + command
        command += int_to_upper_hex(raw_check_sum, 2)
        command += "\r\n"
        command = command.encode()
        self._serial.write(command)

    def send_init_command(self):
        pass

    @property
    def timeout(self):
        return self._serial.timeout

    @timeout.setter
    def timeout(self, value):
        self._serial.timeout = value

