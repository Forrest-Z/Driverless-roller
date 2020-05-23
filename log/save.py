#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import os
from campus_driving.msg import VehState
import numpy as np
import sys

cur_path = os.path.dirname(__file__)
def saveData(data):
    global actualPosition
    xVibr = data.xVibr
    yVibr = data.yVibr
    # y = data.y
    position = np.array([xVibr,yVibr])
    actualPosition.append(position)
    print 'is saving'



if __name__ == '__main__':
    rospy.init_node('savedata', anonymous = True)
    rospy.Subscriber("vehState", VehState, saveData)
    actualPosition = []
    rate = rospy.Rate(50) # hz
    while True:
        if rospy.is_shutdown():
            np.save(os.path.join(cur_path,'cal/position.npy'),actualPosition)
            rospy.spin()
            sys.exit()
        else:
            pass
        rate.sleep()
