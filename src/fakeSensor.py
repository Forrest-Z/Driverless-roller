#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import sys
from campus_driving.msg import INSPVAX



def M30_pub():
    global xLast,yLast,flag
    rate = rospy.Rate(50) # hz
    while not rospy.is_shutdown():
        msg = INSPVAX()
#        latitude = float(GPG[2])
#        longitude = float(GPG[4])
#        msg.latitude = int(latitude/100)+(latitude - int(latitude/100)*100.)/60.
#        msg.longitude = int(longitude/100)+(longitude - int(longitude/100)*100.)/60.
#        msg.x, msg.y = gps_2_xy(msg.longitude, msg.latitude)
        msg.x = 35.
        msg.y = 10.
        msg.header.stamp = rospy.Time.now()
        GPGGA_pub.publish(msg)

        data = INSPVAX()
        data.header.stamp = rospy.Time.now()
        data.azimuth = 193.
        azimuth_pub.publish(data)

        cptmsg = INSPVAX()
        cptmsg.x = 26
        cptmsg.y = 5.
        cpt_pub.publish(cptmsg)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('fakeSensor', anonymous = True)
    GPGGA_pub = rospy.Publisher("M30_gps", INSPVAX, queue_size =10)
    azimuth_pub = rospy.Publisher("Double_Antennainsx", INSPVAX, queue_size =10)
    cpt_pub = rospy.Publisher("cpt_ins", INSPVAX, queue_size =10)

    try:
        M30_pub()
    except rospy.ROSInterruptException:
        sys.exit()
