#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np

from campus_driving.msg import mcuFlag
from ctypes import *
import os 

global count
global shifting_last_value
global allow_last_value
libtest = cdll.LoadLibrary('/home/ai/advcan_source_v2.0.18.0/examples/python_can.so') 
libtest.receive_can_msg.restype = c_longlong
#libtest.receive_can_msg.argtypes = (c_int, c_int)
data_type = {"MCU1":502, "PC1":503, "PC2":504, "PC3":505}  #can口传输的ID号
def mcuflag_pub():
    global count
    global shifting_last_value
    global allow_last_value
    global start_trace_last_value
    rate = rospy.Rate(50) # hz
    while not rospy.is_shutdown():
        ree = libtest.receive_can_msg(fd,data_type["MCU1"])
        flag = ree & 0xFF0000
        mcu_flag = mcuFlag()
        mcu_flag.is_shifting = shifting_last_value
        mcu_flag.allow_route = allow_last_value
        mcu_flag.is_normal = True
        mcu_flag.start_trace = start_trace_last_value
        
        if not flag == 0xFF0000:  
            count = 0
            is_shifting = ree & 0x01 == 1
            allow_route = ree & 0x02 == 2
            start_trace = ree & 0x04 == 4
#            mcu_flag = mcuFlag()
            mcu_flag.is_shifting = is_shifting
            mcu_flag.allow_route = allow_route
            mcu_flag.start_trace = start_trace
            shifting_last_value = is_shifting
            allow_last_value = allow_route
            start_trace_last_value = start_trace
            mcu_flag.is_normal = True
            pub.publish(mcu_flag)
        else:
            count += 1
            if count >= 500:
                mcu_flag.is_normal = False
            pub.publish(mcu_flag)
                                
        rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('CAN_receive_test', anonymous = True)
    pub = rospy.Publisher("/mcu_flag", mcuFlag, queue_size=10)
    fd = libtest.open_can("can1",250)
#    fd1 = libtest.open_can("can1",250)
    count = 0
    shifting_last_value = False
    allow_last_value = False
    start_trace_last_value = False
    mcuflag_pub()
    libtest.close_can(fd)





