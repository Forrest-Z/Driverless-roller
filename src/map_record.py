#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy as np
import sys
import time
import os
# from campus_driving.msg import INSPVAX
from  gps_coordinate import gps_2_xy

xLast = 0
yLast = 0
flag = 0

def start_record():
    global xLast,yLast,flag
    rate = rospy.Rate(10) # hz
#    cmd = bytes([2, 0, 100, 13, 0, 0, 0, 3 ,0, 1, 0, 7, 4, 6, 0, 1, 0, 135, 3])
    #ser.write(cmd)
    while not rospy.is_shutdown():
        try:
            line = ser.readline()
            record_file_handler.write(line)
            print line
        except :
            ser.close()
            line = 'no info'
            print line
        """
        if line:
            print line
            line = line.strip()
            if line.startswith('$GPGGA'):
                GPG = line.split(',')
                # msg = INSPVAX()
                try:
                    latitude = float(GPG[2])
                    longitude = float(GPG[4])
                    msg.latitude = int(latitude/100)+(latitude - int(latitude/100)*100.)/60.
                    msg.longitude = int(longitude/100)+(longitude - int(longitude/100)*100.)/60.
                    msg.x, msg.y = gps_2_xy(msg.longitude, msg.latitude)
                    msg.header.stamp = rospy.Time.now()
                    print msg.header.stamp
                    dist = np.sqrt((msg.x-xLast)**2 + (msg.y-yLast)**2)
#                print dist
                    if dist>5. and flag==1:
                        pass
                    else:
                        flag = 1
                        xLast = msg.x
                        yLast = msg.y

                        record_file_handler.write(line + '\n')

                        # GPGGA_pub.publish(msg)
                        # print(latitude,longitude,msg.latitude,msg.longitude )
                except Exception:
                    ser.close()
                    record_file_handler.close()
           """ #         print(line)
        rate.sleep()

    ser.close()

if __name__ == '__main__':
    rospy.init_node('map_record', anonymous = True)
    # GPGGA_pub = rospy.Publisher("M30_gps", INSPVAX, queue_size =10)
    ser = serial.Serial('/dev/ttyS4',115200 ,timeout = 1000)
    cur_path = os.path.dirname(__file__)
    #     param_file = cur_path + '/../params/calibParam.yaml'
    # record_file = cur_path + '/home/ai/map.txt'
    record_file_name = "map_" + time.strftime("%Y%m%d_%H%M%S",time.localtime())+".txt"
    record_file = os.path.join(cur_path,"../map/mapAutoLog/"+record_file_name)
    if not os.path.exists(record_file):
        os.mknod(record_file)
    record_file_handler = open(record_file,'w')
    try:
        start_record()
    except rospy.ROSInterruptException:
        ser.close()
        record_file_handler.close()
        sys.exit()
