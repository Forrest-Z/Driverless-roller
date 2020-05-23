#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
rosbag used in py
ref: http://blog.csdn.net/songrotek/article/details/50789450

author: qidong
'''
import rospy
import rosbag
import os
import numpy as np
import sys
from nav_msgs.msg import OccupancyGrid
from campus_driving.msg import VehState

cur_path = os.path.dirname(__file__)
bag = rosbag.Bag(os.path.join(cur_path,'../log/rosbag/vehState.bag'),'w')

def logVehState(data):
    bag.write('vehState',data)

def logLidar(data):
    bag.write('lidar3d_grid',data)

if __name__ == '__main__':
    rospy.init_node('savedata', anonymous = True)
    rospy.Subscriber("vehState", VehState, logVehState)
    # rospy.Subscriber("/lidar3d_grid", OccupancyGrid, logLidar)

    rate = rospy.Rate(10) # hz
    while not rospy.is_shutdown():
        rate.sleep()
    bag.close()
    rospy.spin()
