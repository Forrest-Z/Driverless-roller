#!/usr/bin/python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest("view_controller_msgs")

import rospy
import sys
import numpy as np
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3
from campus_driving.msg import VehState, PlanOP

rate_float = 1   # hz
class rvizCamera(object):
    def __init__(self):
        pass

    def publish(self):
        preX = recv['preX']
        preY = recv['preY']
        x = recv['x']
        y = recv['y']
        cp = CameraPlacement()
        # 眼睛，也就是人所在的点。朝focus点看
        # 设为预瞄-当前连线的延长线上，延长d [m]
        d = 15.
        distPre = np.sqrt((preX-x)**2 + (preY-y)**2)
        if distPre < 0.001:
            scale = 0.
        else:
            scale = d / distPre
        eye = Point()
        eye.x = x - scale * (preX-x)
        eye.y = y - scale * (preY-y)
        eye.z = 10.   # 因为大地水平坐标系方向和常规不一样，所以为负
        cp.eye.point = eye
        cp.eye.header.frame_id = "base_link"

        # 设置视觉焦点
        focus = Point()
        focus.x = recv['preX']
        focus.y = recv['preY']
        focus.z = 0.
        cp.focus.point = focus
        cp.focus.header.frame_id = "base_link"

        up = Vector3(0, 0, 1)  # 因为大地水平坐标系方向和常规不一样，所以为负
        cp.up.vector = up
        cp.up.header.frame_id = "base_link"

        cp.time_from_start = rospy.Duration(1.0/rate_float)
        pub.publish(cp)

def getVehState(data):
    recv['x'] = data.x
    recv['y'] = data.y
    recv['z'] = data.z
    recv['roll'] = data.roll
    recv['pitch'] = data.pitch
    recv['azimuth'] = data.azimuth

def getModeDrive(data):
    global flagNewDateGLB
    flagNewDateGLB = True
    recv['preX'] = data.prePoint.x
    recv['preY'] = data.prePoint.y
    recv['preZ'] = data.prePoint.z

if __name__ == '__main__':
    rospy.init_node("rvizSet", anonymous = True)
    recv = {'x':0., 'y':0., 'z':0., 'roll':0., 'pitch':0., 'azimuth':0., \
    'preX':0., 'preY':0., 'preZ':0.}
    rospy.Subscriber("vehState", VehState, getVehState)
    rospy.Subscriber("planOutput", PlanOP, getModeDrive)

    pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)

    state = rvizCamera()
    rate = rospy.Rate(rate_float)
    while not rospy.is_shutdown():
        state.publish()
        rate.sleep()
    rospy.spin()
    sys.exit()
