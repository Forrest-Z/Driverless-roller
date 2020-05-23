#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import pynmea2
import numpy as np
import os
import yaml

import sys  
reload(sys)  
sys.setdefaultencoding('utf8') 

import matplotlib.pyplot as plt
from  gps_coordinate import gps_2_xy

#def gnss_route_xy():
    

dAll = ()
file_nameAll = ()
k_all = ()


def gnss_route_xy():
    # Loading Parameters
    cur_path = os.path.dirname(__file__)
    with open(os.path.join(cur_path,'../params/param.yaml'),'r') as f:
        param = yaml.load(f)
    globalMapName = param["map"]["globalMapName"]
    
    latitudeAll = ()
    longitudeAll = ()
    GPS_all = ((0,0,0),)  #GPS_all = ()#在此处定义，取所有txt文件数据
    GPSfile = open(os.path.join(cur_path,''.join(['../map/',globalMapName])), "r")
#    GPSfile = open(os.path.join(cur_path,'../map/M30_dynamic_qianjin_20171103.txt'), "r")
    while True:
        line = GPSfile.readline()
        if line:
            line = line.strip()
            if line.startswith('$GPGGA'):
                GPGGA = line.split(',')
                latitude = float(GPGGA[2])
                longitude = float(GPGGA[4])
                altitude = float(GPGGA[9])
    
#                statusTemp1 = GPGGA[12]
#                statusTemp2 = statusTemp1.split('*')
#                ins_status = statusTemp2[0]
    
                route1_x, route1_y = gps_2_xy(longitude,latitude)
                
                GPS_all += ((route1_x,route1_y,altitude),)
                
                latitudeAll += (latitude,) 
                longitudeAll += (longitude,) 
            else:
                continue
        else:
            break
    
    x1=GPS_all[0][0]  #GPS起点x,y换算值
    y1=GPS_all[0][1]    
    
    x2=GPS_all[-1][0]   #GPS终点x,y换算值
    y2=GPS_all[-1][1]   
        
#    return x1,y1    
#    result = np.array([x1,y1,x2,y2])
    return   x1,y1,x2,y2    
     
         
#    GPSfile.close()
             
#print 

#GPS_all_arr = np.array(GPS_all)
#plt.figure(2)
#plt.plot(GPS_all_arr[:,1],GPS_all_arr[:,0],'*')     #绘图时，将x y互换位置以匹配地图视角
#
#np.save(os.path.join(cur_path,'../../map/gnss.npy'),GPS_all_arr)
#np.save("/home/catkin_ws/src/truck_driving/map/gnss.npy",GPS_all_arr)
