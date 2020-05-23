#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import pynmea2
import numpy as np
#import sys
import os
import yaml
import matplotlib.pyplot as plt
import time
import commonFcn

#import sys
#sys.path.append(os.path.dirname(__file__)+'/..')

# if "../" not in sys.path:
#   sys.path.append("../")
from mapEndPointDraw import findMapEndPoint, genMapPoint

cur_path = os.path.dirname(__file__)
numGPSallPoint = 200

# Loading Parameters
# Constant Parameters
param_file = cur_path + '/../params/param.yaml'
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
# 首先获取线段圆弧顶点坐标
def getMapPrimePoint():
    # 临时试验直走一次跑道,即两条直线
    mapEndPoint, numGPSallPoint = findMapEndPoint(numAllLaneGLB, distBetweenLaneGLB) # (numAllLaneGLB,distOffsetStep=1.5)
    point0 = mapEndPoint[0][0]
    point1 = mapEndPoint[0][1]
    point2 = mapEndPoint[1][1]
    point3 = mapEndPoint[1][0]
    MapPrimePoint = [point0, point1, point2, point3]
    return MapPrimePoint

def genSingleCurveMap(LastLineStartPoint, startPoint, endPoint, numGPSallPoint):
    # 已知半圆两端点坐标，绘制半圆
    # 首先求圆心坐标
    center = np.array([(endPoint[0]-startPoint[0])/2.+startPoint[0] , (endPoint[1]-startPoint[1])/2.+startPoint[1]])
    # 求半圆两端点向量顺时针旋转到平面坐标系的角度(deg)
    angle = commonFcn.twoVectAngle(startPoint, endPoint)
    print 'angle',angle
    R = np.linalg.norm(endPoint-startPoint)/2.
    # 求圆弧路点的相对坐标位置
    # 坐标系： 原点-圆心，y轴方向起点到终点, x轴方向-y轴右手边
    # 判断圆弧相对于之前的直线是往右还是往左，方法：第二个端点在原直线左侧还是右侧
    PA = LastLineStartPoint
    PB = startPoint
    curPosition = endPoint
    pointSide = commonFcn.checkSidePoint2Line(PA, PB, curPosition) # 左正右负
    # theata采样
    if pointSide >= 0.:
        # 圆弧往左，-1/2*pi -> 1/2*pi
        theata = np.linspace(-1./2.*np.pi, 1./2.*np.pi, numGPSallPoint)
    else:
        theata = np.linspace(3./2.*np.pi, 1./2.*np.pi, numGPSallPoint)
    xMapLocal = R*np.cos(theata)
    yMapLocal = R*np.sin(theata)
    # 转换到全局坐标
    position = {'x':center[0], 'y':center[1], 'azimuth':angle}
    mapLocal = np.array([xMapLocal, yMapLocal])
    xMap,yMap = commonFcn.local_2_global_vect(position,mapLocal)
    GPSallPoint = np.column_stack((xMap,yMap))
    return GPSallPoint

# 绘制线段-半圆-线段
def genCurveMap():
    point0, point1, point2, point3 = getMapPrimePoint()
    map0 = genMapPoint(point0,point1,numGPSallPoint)
    map1 = genSingleCurveMap(point0, point1, point2, numGPSallPoint)
    map2 = genMapPoint(point2,point3,numGPSallPoint)
    map3 = genSingleCurveMap(point2, point3, point0, numGPSallPoint)
    plt.axis('equal')
#    plt.plot(map0[:,0],map0[:,1],'r')
#    plt.plot(map1[:,0],map1[:,1],'b')
#    plt.plot(map2[:,0],map2[:,1],'r')
#    plt.plot(map3[:,0],map3[:,1],'b')
    np.save(os.path.join(cur_path,'../map/curveMap0.npy'),map0)
    np.save(os.path.join(cur_path,'../map/curveMap1.npy'),map1)
    np.save(os.path.join(cur_path,'../map/curveMap2.npy'),map2)
    np.save(os.path.join(cur_path,'../map/curveMap3.npy'),map3)


if __name__ == "__main__":
    genCurveMap()
    for i in range(4):
#        indexMap = str(i)
#        mapName = ['../map/curveMap', indexMap, '.npy']
#        mapName = ''.join(mapName)
#        GPS_all = np.load(os.path.join(cur_path,mapName))

        GPS_all = np.load(os.path.join(cur_path,''.join(['../map/curveMap', str(i), '.npy'])))
#        plt.xlim(-100,-40)
#        plt.ylim(10,50)
        plt.axis('equal')
        plt.plot(GPS_all[:,0],GPS_all[:,1],'r')
