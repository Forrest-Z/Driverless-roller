#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time
import serial
import sys
from iai.Controller import CONController
from iai.Axis import CONAxis
from iai.Space import Space
from iai.utilities import *
from campus_driving.msg import stm32TX
import vehicle_model

class Actuator(object):
    def __init__(self):
        # for stm32
        self.ser = serial.Serial('/dev/ttyUSB4',9600 ,timeout = 1000)
        # for IAI
        self.controller = CONController("/dev/ttyUSB3", 38400, 0.04)
        self.axis1 = CONAxis(self.controller, 0)
        self.axis1.reset_error()
        self.axis1.servo = True
        self.axis1.is_moving()
        self.axis1.go_home()
        self.axis1.wait_while_axis_is_using()

    def publish(self):
        temp = self.ser.read(1)
        start = ord(temp)
        if start == 0x0D:
            sensor_data = self.ser.readline()
            data_str = sensor_data.split(',')
            steer = float(data_str[0])
            vel = float(data_str[1])
            brake_stat = float(data_str[2])
            shift_stat = float(data_str[3])

            msg = stm32TX()
            msg.steer_angle = steer
            msg.velocity_CAN = vel
            msg.brake_stat = int(brake_stat)
            msg.shift_stat = int(shift_stat)
            stm32Pub.publish(msg)

    def action(self,data):
        getDaIAI(data)
        # for IAI control
        self.axis1.get_current_position()
        self.axis1.move_absolute(recv['iai_s'], recv['iai_v'], 0.5)
        #steer and da control
        temp0 = str(recv['steerAngle'])
        temp1 = str(recv['da'])
        send_data = '\r' + temp0 + ',' + temp1 + '\n'
        self.ser.write(send_data)

def getDaIAI(data):
    recv['da'] = data.da
    recv['iai_s'] = data.iai_s
    recv['iai_v'] = data.iai_v

def getSteerAngle(data):
    recv['steerAngle'] = data.steer_angle

if __name__ == '__main__':
    rospy.init_node('driver', anonymous = True)
    recv = {'da':vehicle_model.MIN_DA, 'iai_s':vehicle_model.MIN_IAI_S, \
            'iai_v':vehicle_model.MAX_IAI_V, 'steerAngle':0.}
    state = Actuator()
    rospy.Subscriber('daIAI_cmd', stm32TX, state.action)
    rospy.Subscriber('steerAngleCmd', stm32TX, getSteerAngle)

    stm32Pub = rospy.Publisher('stm32TX_info', stm32TX, queue_size =1)

    rate = rospy.Rate(50)
    try:
        while not rospy.is_shutdown():
            state.publish()
    #        state.action()
            rate.sleep()
    except:
        # go_home = Actuator() # let iai go home by class initialization
        state.axis1.go_home()
        state.axis1.wait_while_axis_is_using()
        state.ser.close()
        rospy.spin()
        sys.exit()
