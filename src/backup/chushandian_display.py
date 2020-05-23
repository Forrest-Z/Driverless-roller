#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np

#import math
#import struct
import time
#import cv2


#import cv2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,Point
from campus_driving.msg import LocalPath
from campus_driving.msg import INSPVAX, Route
        

def callback(msg):
   
    
    start_x = msg.startpoint_x
    start_y = msg.startpoint_y
    
    end_x = msg.endpoint_x
    end_y = msg.endpoint_y
    
    curr_x = msg.current_x
    curr_y = msg.current_y
    
    marker_v = Marker()
    marker_v.header.frame_id = "gps_marker"
    marker_v.header.stamp = rospy.Time.now()
    marker_v.ns = "campus_driving"
    marker_v.id = 1
    
    marker_v.type = Marker.POINTS
    p = Point()
    p.x = curr_x
    p.y = curr_y
    p.z = 0.
    marker_v.points.append(p)
    marker_v.action = Marker.ADD
    marker_v.pose.orientation.w = 1.0
    
    marker_v.scale.x = 1.
    marker_v.scale.y = 1.
    marker_v.scale.z = 0.1
    marker_v.color.r = 1.0
    marker_v.color.g = 0.0
    marker_v.color.b = 0.0
    marker_v.color.a = 1.0
    marker_v.lifetime = rospy.Duration()
    
    current_pub.publish(marker_v)
    
    c_line = np.polyfit([start_x,end_x],[start_y,end_y],1)
    x_line = np.linspace(start_x,end_x,100)
    y_line = np.polyval(c_line,x_line)
    
    marker = Marker()
    marker.header.frame_id = "gps_marker"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "campus_driving"
    marker.id = 0
    
    marker.type = Marker.POINTS
#    time1 = time.time()
    for x,y in x_line, y_line:
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.
        marker.points.append(p)
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    
    route_pub.publish(marker)

    
if __name__ == "__main__":
    rospy.init_node('chushandian_display')
    
    route_pub = rospy.Publisher('route_marker', Marker, queue_size=10)  
    current_pub = rospy.Publisher('current_marker', Marker, queue_size=10)  
 
    sub = rospy.Subscriber('desire_route', Route, callback) 
     
    rospy.spin()
