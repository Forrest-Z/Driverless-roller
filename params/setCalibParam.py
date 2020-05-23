#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import os
import sys
if "../" not in sys.path:
    sys.path.append("../src")
import vehicle_model

if __name__ == '__main__':
    while 1:
        # manu_vel = float(raw_input("Please input manu_vel:"))
        manu_vel = 10.
        # kp = float(raw_input("Please input kp:"))  # 3.  0315
        # ki = float(raw_input("Please input ki:"))  # 0.1 0315
        distPreviewForward = float(raw_input("Please input distPreviewForward:"))  # 5
        # distPreviewBack = float(raw_input("Please input distPreviewBack:"))  # 5
        distPreviewBack = distPreviewForward
        distPreviewCurve = float(raw_input("Please input distPreviewCurve:"))  # 5
        # gainPlus = float(raw_input("Please input gainPlus:"))  # 1.7

        # kd = float(raw_input("Please input kd:"))  # 0.001
        kp = 3.
        ki = 0.1
        # distPreviewForward = 5.  # 5 0315
        # distPreviewBack = 5.  # 5 0315
        gainPlus = 0.
        kd = 0.0  # 1. 0.001
        print 'all param have been set'
        # kd = float(raw_input("Please input manu_vel:"))

        # set calibrated parameters
        # rospy.set_param('test/private_name', {'p':18, 'i':20, 'd':13})
        # for local_planning node, anonymous = False
        rospy.set_param('local_planning/GLBflagOpenCalib', 1) # 控制标定量接收程序
        rospy.set_param('local_planning/CALIBRT', \
            {'distIsEnd':5., 'distPreview':10., \
            'widthObstDetect':2.5, 'distObstDetect':9.})

        # distIsEnd: 判断是否到达路径终点的距离 (m)
        # distSearchClose: 最近点搜索范围 (m)
        # distPreview: 规划和避障预瞄距离 (m)， 区别于下面的控制预瞄距离
        # widthObstDetect: 障碍物检测单侧横向宽度(m)，相对于路径candidate。
                        #  校园：1.1；驾校：2.2;工程车2.5,左右总宽4.6m,振动碾对角宽6.6m
                        # A60: 1.72/2 + 0.5 = 1.4    0.8
        # distObstDetect: 障碍物检测纵向长度(m)，相对于M30原点.忽略区域外的障碍物

        # for steer_control node, anonymous = False
        rospy.set_param('steer_control/GLBflagOpenCalib', 1) # 控制标定量接收程序
        if vehicle_model.name == 'A60':
            # 前进增益
            # Ki: 1s增加的转角度数; Ts_steer: 执行周期
            rospy.set_param('steer_control/gainsForward', \
                {'Kp':kp, 'Ki':0., 'Kd':kd, 'Ts_steer':0.1, \
                 'pure':1., 'distPreview':7.5})  # 0.6
            # 后退增益
            rospy.set_param('steer_control/gainsBack', \
                {'Kp':kp, 'Ki':0., 'Kd':kd, 'Ts_steer':0.1, \
                 'pure':3., 'distPreview':9.5}) # 1.
        else:
            # csd parameters
            # 前进增益
            # Ki: 1s增加的转角度数; Ts_steer: 执行周期
            rospy.set_param('steer_control/gainsForward', \
                {'Kp':kp, 'Ki':ki, 'Kd':kd, 'Ts_steer':0.1, \
                 'pure':gainPlus, 'distPreview':distPreviewForward, \
                 'distPreviewCurve':distPreviewCurve})  # 0.6
            # 后退增益
            rospy.set_param('steer_control/gainsBack', \
                {'Kp':kp, 'Ki':ki, 'Kd':kd, 'Ts_steer':0.1, \
                 'pure':gainPlus, 'distPreview':distPreviewBack, \
                 'distPreviewCurve':distPreviewCurve}) # 1.

        # for velocity_control.py node, anonymous = False
        # node name should be identical to the name in launcah file
        rospy.set_param('velocity_control/GLBflagOpenCalib', 1) # 控制标定量接收程序
        rospy.set_param('velocity_control/CALIBRTVEL', \
            {'manu_vel':manu_vel})
        # save calibrated data
        cur_path = os.path.dirname(__file__)
        file_name = os.path.join(cur_path,'calibParam.yaml')
        cmdDump = "rosparam dump " + file_name
        os.system(cmdDump)



'''
for campus
# 前进增益
# KiErrY: 1s增加的转角度数; Ts_steer: 执行周期
rospy.set_param('steer_control/gainsForward',
    {'KpErrY':60., 'KiErrY':1., 'KpErrTheta':60., 'KdErrTheta':0.,
     'sPre':5.5, 'Ts_steer':0.1})
# 后退增益
rospy.set_param('steer_control/gainsBack',
    {'KpErrY':25., 'KiErrY':1., 'KpErrTheta':4., 'KdErrTheta':0.,
     'sPre':7.5, 'Ts_steer':0.1})
'''
# 'KpErrY':60.  'KpErrY':20.   'KiErrY':0.1, A60前进后退无超调 P控制 10 4 20 4 60 4
'''
    rospy.set_param('steer_control/gainsForward',
        {'KpErrY':70., 'KiErrY':4, 'KpErrTheta':0., 'KdErrTheta':0., 'sPre':7., 'Ts_steer':0.1})
    # 后退增益
    rospy.set_param('steer_control/gainsBack',
        {'KpErrY':150., 'KiErrY':5, 'KpErrTheta':0., 'KdErrTheta':0., 'sPre':7.})
'''
