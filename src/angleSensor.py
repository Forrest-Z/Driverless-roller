#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy as np
import sys
import time
import re
from campus_driving.msg import INSPVAX

countPub = 0
def update():
    global countPub
    rate = rospy.Rate(200) # hz !< 200
    while not rospy.is_shutdown():
        countPub += 1
        try:
            data = ser.read(1)
        except :
            ser.close()
            data = 'no info'
            print data
        if data == 'R':
            line = ser.read(12)
            print 'line', line
            GPG = re.findall(r"\d+",line)  #
            try:
                startQuanshu=int(float(GPG[0]))
                Quanshu=int(startQuanshu-32767)
                startAngle=int(float(GPG[1]))
                angle=float(startAngle/10.)
                if startQuanshu==32767:
                    angle = angle - 11.4 # positive: 左转,max = 149.7-118.6 = 31.1; negative: 右转，min = -33.1
                else:
                    angle = -(angle + 11.4)
                # publish
                if countPub >= 10:
                    countPub = 0
                    msg = INSPVAX()
                    msg.startQuanshu=startQuanshu
                    msg.Quanshu=Quanshu
                    msg.startAngle=startAngle
                    msg.angle=angle
                    msg.header.stamp = rospy.Time.now()
                    INSPVAX_pub.publish(msg)
                print(Quanshu,angle)
            except Exception:
                ser.close()
                print(data)
        rate.sleep()
    ser.close()

if __name__ == '__main__':
    rospy.init_node('angleSensor', anonymous = True)
    INSPVAX_pub = rospy.Publisher("angleSensor", INSPVAX, queue_size =5)
    baud = 9600
    # ser = serial.Serial('/dev/ttyS1',baud ,timeout = 1000)
    ser = serial.Serial('/dev/ttyS1',baud ,timeout = 1000)
    try:
        update()
    except rospy.ROSInterruptException:
        ser.close()
        rospy.spin()
        sys.exit()
