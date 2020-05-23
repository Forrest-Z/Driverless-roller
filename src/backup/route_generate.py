#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
文件功能：
接收M30信息，记录行驶路径，保存成地图文件route.txt
运行本文件，开始记录
停止运行，结束记录并保存
每次运行会覆盖之前的文件
'''
import rospy
#import serial

from campus_driving.msg import INSPVAX

#from campus_driving.msg import M30
#from ctypes import *
import os 

import time
#data = 10

def callback(msg):

    latitude = msg.latitude 
    longitude = msg.longitude
    
    curr_time = time.time()
    
    wirte_line = "$GPGGA,{0:.3f},{1:.9f},N,{2:.9f},E,4,08,1.1,1281.636,M,-31.992,M,1.0,0000*59\n".format(curr_time,latitude,longitude)
    f.write(wirte_line)
       
if __name__ == '__main__':
    rospy.init_node('gps_transmit', anonymous = True)
    cur_path = os.path.dirname(__file__)
    route_file_path = os.path.join(cur_path,'../map/route.txt')
#    open("/home/catkin_ws/src/campus_driving/map/route.txt")

    if not os.path.exists(route_file_path):
        os.mknod(route_file_path)
    else:
        os.remove(route_file_path)
        os.mknod(route_file_path)
        
    f = open(route_file_path,'w+') 
    
    pos = rospy.Subscriber('/M30_gps', INSPVAX, callback)
    
    rospy.spin()
    f.close()
    
#    M30_pub = rospy.Publisher("Double_Antennainsx", M30, queue_size =10)   
 
#    try:
#        Double_Antenna_pub()
#    except rospy.ROSInterruptException:
#        pass





