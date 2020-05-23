#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy as np
import sys
import time
from campus_driving.msg import INSPVAX
from  gps_coordinate import gps_2_xy

xLast = 0
yLast = 0
flag = 0

def M30_pub():
    global xLast,yLast,flag
    rate = rospy.Rate(50) # hz
#    cmd = bytes([2, 0, 100, 13, 0, 0, 0, 3 ,0, 1, 0, 7, 4, 6, 0, 1, 0, 135, 3])
    #ser.write(cmd)
    while not rospy.is_shutdown():
        try:
            line = ser.readline()
        except :
            ser.close()
            line = 'no info'
        if line:
            line = line.strip()
            if line.startswith('$GPGGA'):
                GPG = line.split(',')
                msg = INSPVAX()
                try:
                    latitude = float(GPG[2])
                    longitude = float(GPG[4])
                    msg.latitude = int(latitude/100)+(latitude - int(latitude/100)*100.)/60.
                    msg.longitude = int(longitude/100)+(longitude - int(longitude/100)*100.)/60.
                    msg.x, msg.y = gps_2_xy(msg.longitude, msg.latitude)
                    msg.header.stamp = rospy.Time.now()
                    print msg.header.stamp
                    dist = np.sqrt((msg.x-xLast)**2 + (msg.y-yLast)**2)
#                print dist
                    if dist>5. and flag==1:
                        pass
                    else:
                        flag = 1
                        xLast = msg.x
                        yLast = msg.y

                        GPGGA_pub.publish(msg)
                        print(latitude,longitude,msg.latitude,msg.longitude )
                except Exception:
                    ser.close()
                    print(line)
        rate.sleep()

    ser.close()

if __name__ == '__main__':
    rospy.init_node('M30', anonymous = True)
    GPGGA_pub = rospy.Publisher("M30_gps", INSPVAX, queue_size =10)
    ser = serial.Serial('/dev/ttyS4',115200 ,timeout = 1000)
    try:
        M30_pub()
    except rospy.ROSInterruptException:
        ser.close()
        sys.exit()
