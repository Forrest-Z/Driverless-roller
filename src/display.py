#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
import sys
import time
import yaml
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,Point
from campus_driving.msg import VehState, PathDisplay, PlanOP
from mapEndPointDraw import findMapEndPoint, genMapPoint
import vehicle_model
import commonFcn


position = {'x':0., 'y':0., 'azimuth':0.}
countMap = 0
GLBstepDispMap = 10
cur_path = os.path.dirname(__file__)
GPSmap = np.array([])
def globalMapRaw():
    global GPSmap
    if taskTypeGLB == 'csdCircle':
        for i in range(4):
            GPS_all = np.load(os.path.join(cur_path,''.join(['../map/curveMap', str(i), '.npy'])))
            if GPSmap.size == 0:
                GPSmap = GPS_all
            else:
                GPSmap = np.concatenate((GPSmap, GPS_all), axis=0)
        gps_points = GPSmap
    else:
        gps_points = np.load(os.path.join(cur_path,'../map/rawMap.npy'))
    marker = Marker()
    marker.header.frame_id = "gps_marker"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "campus_driving"
    marker.id = 0

    marker.type = Marker.POINTS
    for x,y in gps_points:
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

    pubMap.publish(marker)

def globalMap():

    mapEndPoint, numGPSallPoint = findMapEndPoint(numAllLaneGLB,distBetweenLaneGLB) # (numAllLaneGLB,distOffsetStep=1.5)
    marker = Marker()
    marker.header.frame_id = "gps_marker"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "campus_driving"
    marker.id = 0
    marker.type = Marker.LINE_LIST

    for i in xrange(numAllLaneGLB):
        p = Point()
        route_x_st,route_y_st = mapEndPoint[i][0]
        p.x = route_x_st
        p.y = route_y_st
        p.z = 0.
        marker.points.append(p)

        p = Point()   # note: without it, line_list can not display in rviz
        route_x_end,route_y_end = mapEndPoint[i][1]
        p.x = route_x_end
        p.y = route_y_end
        p.z = 0.
        marker.points.append(p)

    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1
#    marker.scale.y = 0.1
#    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()

    pubMap.publish(marker)

def curPoint(msg):
    global countMap, GLBstepDispMap

    if countMap>=GLBstepDispMap:  # callback period is 20 ms, that is, the period is 2s
        countMap = 0
        GLBstepDispMap = np.amin([10000, GLBstepDispMap*10])  # 不采用这种机制会被优化掉
        if mapTypeGLB == 'raw' or taskTypeGLB == 'csdCircle':
            globalMapRaw()
        else:
            globalMap()
    countMap = countMap + 1

    point = [msg.x, msg.y, 0.]
    ID = 1
    color = [1., 0., 0., 1.] # r g b a
    scale = [0.8, 0.8, 0.1] # x y z
    argsCurPoint = (ID, pubCurPoint, color, scale)
    plotPoint(point, argsCurPoint)

    point = [msg.xVibr, msg.yVibr, 0.]
    ID = 10
    color = [0., 1., 0., 1.] # r g b a
    scale = [0.8, 0.8, 0.1] # x y z
    argsCurPoint = (ID, pubCurPoint, color, scale)
    plotPoint(point, argsCurPoint)

    position['x'] = msg.x
    position['y'] = msg.y
    position['azimuth'] = msg.azimuth

def prePoint(msg):
    point = [msg.prePoint.x, msg.prePoint.y, msg.prePoint.z]
    IDPrePoint = 2
    colorPrePoint = [0., 0., 1., 1.] # r g b a
    scalePrePoint = [1., 1., 0.1]
    argsPrePoint = (IDPrePoint, pubPrePoint, colorPrePoint, scalePrePoint)
    plotPoint(point, argsPrePoint)


def plotPoint(point, args):
    ID = args[0]
    pub = args[1]
    color = args[2]
    scale = args[3]

    marker_p = Marker()
    marker_p.header.frame_id = "gps_marker"
    marker_p.header.stamp = rospy.Time.now()
    marker_p.ns = "campus_driving"
    marker_p.id = ID

    marker_p.type = Marker.POINTS
    p = Point()
    p.x = point[0]
    p.y = point[1]
    p.z = point[2]
    marker_p.points.append(p)
    marker_p.action = Marker.ADD
    marker_p.pose.orientation.w = 1.0

    marker_p.scale.x = scale[0]
    marker_p.scale.y = scale[1]
    marker_p.scale.z = scale[2]
    marker_p.color.r = color[0]
    marker_p.color.g = color[1]
    marker_p.color.b = color[2]
    marker_p.color.a = color[3]
    marker_p.lifetime = rospy.Duration()

    pub.publish(marker_p)

def BasePath(msg):
    '''
    base path include global path and optimal path
    '''
    marker_globalPath = Marker()
    marker_globalPath.header.frame_id = "gps_marker"
    marker_globalPath.header.stamp = rospy.Time.now()
    marker_globalPath.ns = "campus_driving"
    marker_globalPath.id = 4

    marker_globalPath.type = Marker.POINTS
    x_S = np.array(msg.xGlbPath)
    y_S = np.array(msg.yGlbPath)
#    print("DISPLAY",x_S,y_S)
    for i in xrange(0,x_S.shape[0],1):
        p = Point()
        p.x = x_S[i]
        p.y = y_S[i]
        p.z = 0.
#        print("p",p)
        marker_globalPath.points.append(p)

    marker_globalPath.action = Marker.ADD
    marker_globalPath.pose.orientation.w = 1.0

    marker_globalPath.scale.x = 0.1
    marker_globalPath.scale.y = 0.1
    marker_globalPath.scale.z = 0.1
    marker_globalPath.color.r = 1.
    marker_globalPath.color.g = 1.
    marker_globalPath.color.b = 1.
    marker_globalPath.color.a = 1.0
    marker_globalPath.lifetime = rospy.Duration()
    pubCenterPath.publish(marker_globalPath)

    # plot all local path
    IDOptimal = 5
    colorOptimal = [0., 0., 1., 1.0] # r g b a
    scale = [0.04, 0.04, 0.1]
    argsOptimal = (IDOptimal, pubLocalPath, colorOptimal, scale)
    plotLocalPath(msg, argsOptimal)
    # plot optimal path

    IDOptimal = 6
    colorOptimal = [1., 0., 0., 1.0] # r g b a
    scale = [0.2, 0.2, 1]
    argsOptimal = (IDOptimal, pubOptimalPath, colorOptimal, scale)
    msg.xLocPath.data = np.array(msg.xLocPath.data).reshape \
        (msg.xLocPath.layout.dim[0].size,-1)[msg.xLocPath.layout.dim[0].stride]
    msg.yLocPath.data = np.array(msg.yLocPath.data).reshape \
        (msg.yLocPath.layout.dim[0].size,-1)[msg.yLocPath.layout.dim[0].stride]
    plotLocalPath(msg, argsOptimal)

def plotLocalPath(msg, args):
    '''
    how to add arguments to callback
    ref: https://answers.ros.org/question/231492/passing-arguments-to-callback-in-python/
    '''
    ID = args[0]
    pub = args[1]
    color = args[2]
    scale = args[3]
    marker_localPath = Marker()
    marker_localPath.header.frame_id = "gps_marker"
    marker_localPath.header.stamp = rospy.Time.now()
    marker_localPath.ns = "campus_driving"
    marker_localPath.id = ID

    marker_localPath.type = Marker.POINTS
    x_S = np.array(msg.xLocPath.data)
    y_S = np.array(msg.yLocPath.data)

    #先将轨迹点局部坐标转换为全局坐标
    point = np.array([x_S,y_S])
    pointGlobal = commonFcn.local_2_global_vect(position,point)
    x_S = pointGlobal[0]
    y_S = pointGlobal[1]


    for i in xrange(0,x_S.shape[0],1):
        p = Point()
        p.x = x_S[i]
        p.y = y_S[i]
        p.z = 0.
        marker_localPath.points.append(p)

    marker_localPath.action = Marker.ADD
    marker_localPath.pose.orientation.w = 1.0
    marker_localPath.scale.x = scale[0]
    marker_localPath.scale.y = scale[1]
    marker_localPath.scale.z = scale[2]
    marker_localPath.color.r = color[0]
    marker_localPath.color.g = color[1]
    marker_localPath.color.b = color[2]
    marker_localPath.color.a = color[3]
    marker_localPath.lifetime = rospy.Duration()

    pub.publish(marker_localPath)

if __name__ == "__main__":
    # Loading Parameters
    base_path = os.path.dirname(__file__)
    param_file = base_path + '/../params/param.yaml'
    with open(param_file,'r') as f:
        param = yaml.load(f)
    mapTypeGLB = param["map"]["mapType"]
    taskTypeGLB = param["map"]["taskType"]
    if mapTypeGLB == 'raw':
        # 校园正常测试，而不是模拟出山店
        numAllLaneGLB = 1
        distBetweenLaneGLB = 1.  # 可随意设置，不再用到
        timesSingleLaneGLB = 1
    else:
        numAllLaneGLB = param["map"]["numAllLane"]
        distBetweenLaneGLB = param["map"]["distBetweenLane"]
        timesSingleLaneGLB = param["map"]["timesSingleLane"]

    rospy.init_node('display')
    pubMap = rospy.Publisher('gps_diplay', Marker, queue_size=1)
    pubCurPoint = rospy.Publisher('gps_diplay_v', Marker, queue_size=1)
    pubPrePoint = rospy.Publisher('gps_diplay_p', Marker, queue_size=1)
    pubCenterPath = rospy.Publisher('CenterPath_display', Marker, queue_size=1)
    pubOptimalPath = rospy.Publisher('optimalPath_display', Marker, queue_size=1)
    pubLocalPath = rospy.Publisher('localPath_display', Marker, queue_size=1)

    sub1 = rospy.Subscriber("vehState", VehState, curPoint)
    sub3 = rospy.Subscriber('basePath', PathDisplay, BasePath)
    subPrePoint = rospy.Subscriber('planOutput', PlanOP, prePoint)

    rospy.spin()
#    sys.exit()
