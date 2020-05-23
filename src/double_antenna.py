#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import serial
import time
import sys

from campus_driving.msg import INSPVAX

class Double_Antenna(object):
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS0',115200 ,timeout = 1000)

    def publish(self):
        try:
            line = self.ser.readline()
        except:
            self.ser.close()
            line = 'no info'
        if line:
            line = line.strip()
            if line.startswith('#HEADINGA'):
                INSPV = line.split(',')
                msg = INSPVAX()
                msg.header.stamp = rospy.Time.now()
                msg.azimuth = float(INSPV[12])
                azimuthPub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('Double_Antenna', anonymous = True)
    azimuthPub = rospy.Publisher("Double_Antennainsx", INSPVAX, queue_size =10)

    state = Double_Antenna()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        state.publish()
        rate.sleep()
    state.ser.close()
    rospy.spin()
    sys.exit()
