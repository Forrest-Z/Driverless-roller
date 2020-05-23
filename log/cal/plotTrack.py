# -*- coding: utf-8 -*-
"""
Created on Fri Dec  8 21:51:16 2017

@author: lwb
"""

import numpy as np
import os
import yaml
import sys
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
if "../../" not in sys.path:
    sys.path.append("../../src")
    sys.path.append("../../params")
from mapEndPointDraw import findMapEndPoint
import commonFcn

def plotMapLine():
    # 绘制直线地图
    mapEndPoint, numGPSallPoint = findMapEndPoint(numAllLaneGLB,distBetweenLaneGLB) # (numAllLaneGLB,distOffsetStep=1.5)
    for i in range(len(mapEndPoint)):
        start_x, start_y = mapEndPoint[i][0]
        end_x, end_y = mapEndPoint[i][1]
        c_line = np.polyfit([start_x,end_x],[start_y,end_y],1)
        x_line = np.linspace(start_x,end_x,100)
        y_line = np.polyval(c_line,x_line)
        plt.plot(x_line, y_line, 'b')
        plt.plot(x_line[0],y_line[0],'ro') # 绘制起点
def plotActualTrack():
    position = np.load(os.path.join(cur_path,'position.npy'))
    xVibr = position[:,0]
    yVibr = position[:,1]
    plt.plot(xVibr, yVibr, 'r')

if __name__ == '__main__':

    cur_path = os.path.dirname(__file__)
    param_file = cur_path + '/../../params/param.yaml'
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

    # 设置坐标刻度
    #plt.xticks(np.linspace(0,2,4))
    #plt.yticks(np.linspace(0,10,10))
    # 设置坐标范围
    #plt.xlim(0,2)
    #plt.ylim(0,10)
    # 设置坐标轴纵横比
    plt.axis('equal')
    plotActualTrack()
    plotMapLine()

    
