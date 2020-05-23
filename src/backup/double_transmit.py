#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import serial
import numpy as np

from campus_driving.msg import INSPVAX, Route
#from campus_driving.msg import M30
from ctypes import *
import os 
from message_filters import  Subscriber, ApproximateTimeSynchronizer
from  gps_coordinate import gps_2_xy
from dam_map_draw import gnss_route_xy
#data = 10

libtest = cdll.LoadLibrary('/home/ai/advcan_source_v2.0.18.0/examples/transmit_gps.so') 

data_type = {"PC1":500, "PC2":501, "PC3":502}  #can口传输的ID号
x_ori = 3380400.6653839252   #车库前
y_ori = 245529.56042564771  #车库前

def Callback(msg1,msg2):

    latitude = msg1.latitude 
    longitude = msg1.longitude
    
    azimuth = msg2.azimuth
    

    desire_route = Route()
#    current_x, current_y = gps_2_xy(longitude, latitude, 3380400.6653839252, 245529.56042564771 )  # 按需求更改初始位置
    current_x, current_y = gps_2_xy(longitude, latitude, x_ori, y_ori )  # 按需求更改初始位置
    desire_route.current_x = current_x
    desire_route.current_y = current_y
#    
    
    
    
    current_x = (current_x+3700)*100     
    current_y = (current_y+3700)*100
    
    current_x = int(current_x)
    current_y = int(current_y)
    
    azimuth *= 100
    azimuth = int(azimuth) 

    data = (c_char*8)()
    data[0] = chr((current_x & 0x0F0000) >> 16)
    data[1] = chr((current_x & 0x00FF00) >> 8)
    data[2] = chr(current_x & 0x0000FF) 
    data[3] = chr((current_y & 0x0F0000) >> 16)
    data[4] = chr((current_y & 0x00FF00) >> 8)
    data[5] = chr(current_y & 0x0000FF) 
    data[6] = chr((azimuth & 0x00FF00) >> 8)
    data[7] = chr(azimuth & 0x0000FF)
    
    libtest.trans_can_msg(fd, data, data_type["PC1"])
#    print("trans ok")
        
#    route1_x, route1_y = gnss_route_xy()
    route1_x,route1_y,route2_x,route2_y = gnss_route_xy()
    desire_route.endpoint_x = route2_x
    desire_route.endpoint_y = route2_y
    
    desire_route.startpoint_x = route1_x
    desire_route.startpoint_y = route1_y
    
    route1_x = (route1_x+3700)*100     
    route1_y = (route1_y+3700)*100
    route1_x = int (route1_x)
    route1_y = int (route1_y)    
        
    data = (c_char*8)()
    data[0] = chr((route1_x & 0x0F0000) >> 16)
    data[1] = chr((route1_x & 0x00FF00) >> 8)
    data[2] = chr(route1_x & 0x0000FF) 
    data[3] = chr((route1_y & 0x0F0000) >> 16)
    data[4] = chr((route1_y & 0x00FF00) >> 8)
    data[5] = chr(route1_y & 0x0000FF) 
    data[6] = chr((0 & 0x00FF00) >> 8)
    data[7] = chr(0 & 0x0000FF)
      
#    for d in data:
#        print hex(ord(d))
    libtest.trans_can_msg(fd, data, data_type["PC2"])
#    print("trans2 ok")
    
#    route2_x, route2_y = gnss_route_xy()
    route2_x = (route2_x+3700)*100     
    route2_y = (route2_y+3700)*100
    route2_x = int (route2_x)
    route2_y = int (route2_y)    
        
    data = (c_char*8)()
    data[0] = chr((route2_x & 0x0F0000) >> 16)
    data[1] = chr((route2_x & 0x00FF00) >> 8)
    data[2] = chr(route2_x & 0x0000FF) 
    data[3] = chr((route2_y & 0x0F0000) >> 16)
    data[4] = chr((route2_y & 0x00FF00) >> 8)
    data[5] = chr(route2_y & 0x0000FF) 
    data[6] = chr((0 & 0x00FF00) >> 8)
    data[7] = chr(0 & 0x0000FF)
     
    
    pub_route.publish(desire_route)
#    for d in data:
#        print hex(ord(d))
    libtest.trans_can_msg(fd, data, data_type["PC3"])
#    print("trans3 ok")    
       
if __name__ == '__main__':
    rospy.init_node('gps_transmit', anonymous = True)
    fd = libtest.open_can("can1",500)
#    sub = rospy.Subscriber("/Double_Antennainsx",INSPVAX, callback)
    ang = Subscriber('/Double_Antennainsx', INSPVAX)
    pos = Subscriber('/M30_gps', INSPVAX)
    
    pub_route = rospy.Publisher("/desire_route", Route, queue_size=10)
    tss = ApproximateTimeSynchronizer([pos, ang],10,0.01)       
    tss.registerCallback(Callback)
    rospy.spin()
    libtest.close_can(fd)
    
#    M30_pub = rospy.Publisher("Double_Antennainsx", M30, queue_size =10)   
 
#    try:
#        Double_Antenna_pub()
#    except rospy.ROSInterruptException:
#        pass





