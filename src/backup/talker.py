#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import threading
import time
import sys
from std_msgs.msg import Float32

global vel_manualset
vel_manualset = 8.

class talker(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def stop(self):
        self.timeToQuit.set()       
    def run(self):
        pub = rospy.Publisher('vel_desire', Float32, queue_size=1)
        rate = rospy.Rate(10) # 50hz
        while not rospy.is_shutdown():
            lock.acquire()            
            vel_set = vel_manualset
            lock.release()
            rospy.loginfo(vel_set)
            #print('vel_set:', vel_set)
            pub.publish(vel_set)
            rate.sleep()

class readSetCmd(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def stop(self):
        self.timeToQuit.set()       
    def run(self):
        rospy.Subscriber("vel_manualSet", Float32, self._call_read)
        rospy.spin()
    def _call_read(self, data):
        global vel_manualset
        
        lock.acquire()
        vel_manualset = data.data
        lock.release()
    
if __name__ == '__main__':
        rospy.init_node('talker', anonymous = True)
        lock = threading.RLock()    #RLock对象  

        vel_set_task = readSetCmd()    #create a new thread
        vel_set_task.setDaemon(True)  #主线程A启动了子线程B，调用b.setDaemaon(True)，则主线程结束时，会把子线程B也杀死
        vel_set_task.start()

        vel_pub_task = talker()
        vel_pub_task.setDaemon(True)
        vel_pub_task.start()

        while True:
            if rospy.is_shutdown():
                rospy.loginfo("stop")
                sys.exit()
            else:
                pass
            time.sleep(0.02)

'''
def talker():
    rospy.init_node('talker', anonymous=True)    
    pub = rospy.Publisher('vel_desire', Float32, queue_size=10)
    rospy.Subscriber("vel_set", Float32, self.CPTbestx_read)
    #rospy.spin() 
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vel_set = 8.
        rospy.loginfo(vel_set)
        pub.publish(vel_set)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
'''
