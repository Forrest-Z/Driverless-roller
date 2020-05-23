#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import yaml
import time
import os
from ctypes import *
from message_filters import  Subscriber, ApproximateTimeSynchronizer
from campus_driving.msg import PlanOP, mcuFlag,stm32TX,VehState
from mapEndPointDraw import findMapEndPoint

libtest = cdll.LoadLibrary('/home/ai/advcan_source_v2.0.18.0/examples/transmit_gps.so')
data_type = {"PC1":503, "PC2":504, "PC3":505}  #can口传输的ID号
allow_route = False
CAN_is_normal = True
flagCANTest = False
def transRoute(route_x,route_y,fistID,index,numAllID):
    flagStartEnd = int(index)%2 # 0:起点； 1：终点
    indCurrentLane = int(index/2)
############ can test ########################################
    if flagCANTest == True:
        flagStartEnd = 1 # 0:起点； 1：终点
        indCurrentLane = 2 #
        numAllID = 16 # hex: 10
        route_x = 45. #hex: 5 B6 E4 --> dec: 5 182 228
        route_y = 32. #hex: 5 B1 D0 --> dec: 5 177 208
############  end  ###########################################

    route_x = int((route_x+3700)*100)
    route_y = int((route_y+3700)*100)

    data = (c_char*8)()
    data[0] = chr(((flagStartEnd & 0x000001) << 7) | (indCurrentLane & 0x00007F))
    data[1] = chr(numAllID & 0x0000FF)
    data[2] = chr((route_x & 0x0F0000) >> 16)
    data[3] = chr((route_x & 0x00FF00) >> 8)
    data[4] = chr(route_x & 0x0000FF)
    data[5] = chr((route_y & 0x0F0000) >> 16)
    data[6] = chr((route_y & 0x00FF00) >> 8)
    data[7] = chr(route_y & 0x0000FF)

    ID = fistID + index
    libtest.trans_can_msg(fd, data, ID)

def Callback(msg):
    print('test can')
    global allow_route, CAN_is_normal
    global mapEndPoint, numGPSallPoint
    current_x = msg.xVibr
    current_y = msg.yVibr
    azimuth = msg.azimuth
######## can test ###################################
    if flagCANTest == True:
        current_x = 89.23 #hex: 5 c8 2B -->5 200 43
        current_y = 13.56 #hex: 5 AA 9C -->5 170 156
        azimuth = 355.11 #hex: 8A B7 --> 138 183
########  end    ####################################

    current_x = int((current_x+3700)*100)
    current_y = int((current_y+3700)*100)

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
    libtest.trans_can_msg(fd, data, data_type["PC2"])
    if (allow_route == True) & (CAN_is_normal == True):
#        mapEndPoint, numGPSallPoint = findMapEndPoint(numAllLane,distBetweenLane) # (numAllLane,distOffsetStep=1.5)
        fistID = data_type["PC3"]
        for i in xrange(numAllLane):
            route_x_st,route_y_st = mapEndPoint[i][0]
            route_x_end,route_y_end = mapEndPoint[i][1]
            transRoute(route_x_st,route_y_st,fistID,2*i,numAllLane)    # (route_x,route_y,fistID,index,numAllID)
            transRoute(route_x_end,route_x_end,fistID,2*i+1,numAllLane)

def CallbackSteer(msg1,msg2):
    modeDrive = msg1.modeDrive
    desireDirDrive = msg1.desireDirDrive
    steerAngle_rx = msg2.steer_angle

############ can test ############################################
    if flagCANTest == True:
        modeDrive = 2 # 0:initial; 1: drive; 2:stop; 4:STOP_SAFE
        steerAngle_rx = 126  # hex: 24 f9-->dec: 36 249
        desireDirDrive = 1 # 0:forward; 1:back
############   end  #############################################
    steerAngle = steerAngle_rx*6.  #7560=3.5*360*6,暂时直接按A60比例放大，应该能绕过工程车回差和死区问题
    steerAngle = int(steerAngle+7560)

    steerAngle_deg = int(steerAngle_rx+500)

    data = (c_char*8)()
    data[0] = chr(modeDrive & 0x0F)
    data[1] = chr((steerAngle & 0xFF00) >> 8)
    data[2] = chr(steerAngle & 0xFF)
    data[3] = chr(desireDirDrive & 0x0F)
    data[4] = chr((steerAngle_deg & 0xFF00) >> 8)
    data[5] = chr(steerAngle_deg & 0xFF)
    data[6] = chr((0 & 0x00FF00) >> 8)
    data[7] = chr(0 & 0x0000FF)
    libtest.trans_can_msg(fd, data, data_type["PC1"])

def callbackCAN_RX(msg):
    global allow_route, CAN_is_normal
    allow_route = msg.allow_route
    CAN_is_normal = msg.is_normal

if __name__ == '__main__':
    # Loading Parameters
    base_path = os.path.dirname(__file__)
    param_file = base_path + '/../params/param.yaml'
    with open(param_file,'r') as f:
        param = yaml.load(f)
    numAllLane = param["map"]["numAllLane"]
    distBetweenLane = param["map"]["distBetweenLane"]

    mapEndPoint, numGPSallPoint = findMapEndPoint(numAllLane,distBetweenLane) # (numAllLane,distOffsetStep=1.5)
    rospy.init_node('CAN_Driver', anonymous = True)
    fd = libtest.open_can("can1",250)

    rospy.Subscriber("vehState", VehState, Callback)
    rospy.Subscriber("/mcu_flag", mcuFlag, callbackCAN_RX)

    mode = Subscriber('/planOutput', PlanOP)
    steer = Subscriber('/steerAngleCmd', stm32TX)
    tss1 = ApproximateTimeSynchronizer([mode, steer],10,0.1)
    tss1.registerCallback(CallbackSteer)

    rospy.spin()
    libtest.close_can(fd)
