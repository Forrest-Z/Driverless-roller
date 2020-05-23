# -*- coding: utf-8 -*-

import time
import os

import serial

import serial.tools.list_ports

def check_ttyusb_num(port_location='2-1.1'):
	"""
	"""
	ports = [port for port in serial.tools.list_ports.comports() if port[2] != 'n/a']
	
	for port in ports:
		temp_port_location = port[2].split(' ')[-1]
		if temp_port_location.startswith('LOCATION='):
			if temp_port_location.split('=')[-1] == port_location:
				print("get")
				return port[0]
	raise Exception('USB port is not used!!!')

def get_port_loaction():
#	print [port[0] for port in serial.tools.list_ports.comports() if port[2] != 'n/a']
    ports = [port for port in serial.tools.list_ports.comports() if port[2] != 'n/a']
    for port in ports:
        print port[0], port[1], port[2]


if __name__ == "__main__":

#    port_name = check_ttyusb_num('3-4')
#    print port_name
	get_port_loaction()