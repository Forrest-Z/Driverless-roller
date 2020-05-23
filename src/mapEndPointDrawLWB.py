#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import pynmea2
import numpy as np
#import sys
import os
import yaml
import matplotlib.pyplot as plt
import time
import vehicle_model

#import sys
#sys.path.append(os.path.dirname(__file__)+'/..')

# if "../" not in sys.path:
#   sys.path.append("../")

from commonFcn import findOffsetLinePoint
from  gps_coordinate import gps_2_xy

def genMapPoint(startPointNew,endPointNew,numGPSallPoint):
    c_line = np.polyfit([startPointNew[0],endPointNew[0]],[startPointNew[1],endPointNew[1]],1)
    x_line = np.linspace(startPointNew[0],endPointNew[0],numGPSallPoint)
    y_line = np.polyval(c_line,x_line)
    GPSallPoint = np.column_stack((x_line,y_line))

    return GPSallPoint

def findMapEndPoint(numAllLane,distOffsetStep=1.5,flagPlot=0):
    # Loading Parameters
    cur_path = os.path.dirname(__file__)
    with open(os.path.join(cur_path,'../params/paramLWB.yaml'),'r') as f:
        param = yaml.load(f)
    globalMapName = param["map"]["globalMapName"]
    simu = param["map"]["simu"]

    GPS_all = ()
    GPSfile = open(os.path.join(cur_path,''.join(['../map/',globalMapName])), "r")
    xLast = 0
    yLast = 0
    flag = 0
    while True:
        line = GPSfile.readline()
        if line:
            line = line.strip()
            if vehicle_model.name == 'A60':
                if line.startswith('%INSPVASA'):
                    try:
                        INSPV = line.split(',')
                        latitude = float(INSPV[4])
                        longitude = float(INSPV[5])
                        altitude = float(INSPV[6])
                        latitude_deg = latitude
                        longitude_deg = longitude

                        x_offset, y_offset = gps_2_xy(longitude_deg, latitude_deg)  #x指向正北 #y指向正东
                        dist = np.sqrt((x_offset-xLast)**2 + (y_offset-yLast)**2)
                        if flag == 0:
                            flag = 1
                            xLast = x_offset
                            yLast = y_offset
                        else:
                            if dist > 5. : # 剔除异常数据
                                pass
                            elif dist > 0.2 : # 间隔0.2m取一个点
                                xLast = x_offset
                                yLast = y_offset
                                GPS_all += ((x_offset,y_offset),)
                            else:
                                pass
                    except Exception:
#                            print(line)
                            pass
                else:
                    continue
            else:
                if line.startswith('$GPGGA'):
                    GPGGA = line.split(',')
                    try:
                        latitude = float(GPGGA[2])
                        longitude = float(GPGGA[4])
    #                    altitude = float(GPGGA[9])
                        altitude = 0.
                        latitude_deg=int(latitude/100)+(latitude - int(latitude/100)*100)/60.
                        longitude_deg=int(longitude/100)+(longitude - int(longitude/100)*100)/60.

                        x_offset, y_offset = gps_2_xy(longitude_deg, latitude_deg)  #x指向正北 #y指向正东
                        dist = np.sqrt((x_offset-xLast)**2 + (y_offset-yLast)**2)
                        if dist>5. and flag==1:
                            pass
                        else:
                            flag = 1
                            xLast = x_offset
                            yLast = y_offset

                            GPS_all += ((x_offset,y_offset),)
                    except Exception:
        #                    print(line)
                            pass
                else:
                    continue
        else:
            break
    GPSfile.close()
    if simu == True:
        GPS_all = np.load(os.path.join(cur_path,'../log/cal/map_gen/simuMap.npy'))  # 用仿真生成的覆盖实际地图
    GPS_all_arr = np.array(GPS_all)
    numGPSallPoint = len(GPS_all)
    np.save(os.path.join(cur_path,'../map/rawMap.npy'),GPS_all_arr)

    start_x=GPS_all[0][0]  #GPS起点x,y换算值  1229
    start_y=GPS_all[0][1]
    end_x=GPS_all[-1][0]   #GPS终点x,y换算值
    end_y=GPS_all[-1][1]
    c_line = np.polyfit([start_x,end_x],[start_y,end_y],1)
    x_line = np.linspace(start_x,end_x,len(GPS_all))
    y_line = np.polyval(c_line,x_line)
    GPS_all_line = np.column_stack((x_line,y_line))

    if flagPlot == 1:
        # 绘制原始点
#        plt.figure(1)
        plt.plot(GPS_all_arr[:,0],GPS_all_arr[:,1],'*')     #绘图时，将x y互换位置以匹配地图视角

        # 绘制原始直线和起止点
        plt.plot(GPS_all_line[:,0],GPS_all_line[:,1],'-')     #绘图时，将x y互换位置以匹配地图视角
        plt.plot(start_x,start_y,'ro')
        plt.plot(end_x,end_y,'bo')
        #np.save(os.path.join(cur_path,'../map/xueyuanLine.npy'),GPS_all_line)

    startPoint = np.array([start_x,start_y])
    endPoint = np.array([end_x,end_y])
    distOffset = 0
    mapEndPoint = []
    for i in range(param["map"]["bigCycle"])
        mapEndPoint.append([startPoint,endPoint])
        for i in xrange(numAllLane-1):
            distOffset += distOffsetStep
            startPointNew,endPointNew  = findOffsetLinePoint(startPoint,endPoint,distOffset)       # findOffsetLinePoint(startPoint,endPoint,distOffset)
            mapEndPoint.append([startPointNew,endPointNew])

        setPlanTrajectory  = param["map"]["planTrajectory"]
        if setPlanTrajectory == 'zhi':
        mapEndPoint.append([startPointNew,endPointNew])
        for l in xrange(numAllLane-1):
            distOffset -= distOffsetStep
            startPointNew,endPointNew  = findOffsetLinePoint(startPoint,endPoint,distOffset)
            mapEndPoint.append([startPointNew,endPointNew])   # “之”字型路线，如果走六条道的话，需要设置的总线路数为 3



        if flagPlot == 1:
            # 绘制偏移直线和起止点
            GPS_all_lineOffset = genMapPoint(startPointNew,endPointNew,numGPSallPoint)
            plt.plot(GPS_all_lineOffset[:,0],GPS_all_lineOffset[:,1],'c-')     #绘图时，将x y互换位置以匹配地图视角
            plt.plot(startPointNew[0],startPointNew[1],'ro')
            plt.plot(endPointNew[0],endPointNew[1],'bo')
            #print (startPointNew[0]-start_x)*(end_x-start_x)+(startPointNew[1]-start_y)*(end_y-start_y)
            #np.save(os.path.join(cur_path,'../map/xueyuanLineOffset.npy'),GPS_all_lineOffset)

#    numGPSallPoint = (endPointNew[0]-startPointNew[0])/len(GPS_all)

    return mapEndPoint, numGPSallPoint
if __name__ == "__main__":
    a,num=findMapEndPoint(2,10.5,1)
    print a
    print 'x',a[1][0]
