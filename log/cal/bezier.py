# -*- coding: utf-8 -*-
"""
Created on Sat Mar 10 22:51:27 2018

@author: li
"""

import numpy as np
import matplotlib.pyplot as plt

def plotBezier(point0,point1,point2,point3):
    pointRoadBessel = np.array([point0,point1,point2,point3]) #4行2列
    ARoadBessel = np.array([[-1,3,-3,1],[3,-6,3,0],[-3,3,0,0],[1,0,0,0]])
    polyCoeffiCandidate = np.dot(ARoadBessel,pointRoadBessel) #t**3 t**2 t 1,note: the variable is neither x nor y, but t
    #        pYRoadBessel = np.dot(ARoadBessel,pointRoadBessel[:,1]) #Look
    t = np.linspace(0,1,np.max([100,50]))
    x_S = np.polyval(polyCoeffiCandidate[:,0],t)
    y_S = np.polyval(polyCoeffiCandidate[:,1],t)
    
    return x_S, y_S
#point0 = np.array([0,0])
#point1 = np.array([0,3])
#point2 = np.array([2,2])
#point3 = np.array([2,5])

# 设置坐标刻度
#plt.xticks(np.linspace(0,2,4))
#plt.yticks(np.linspace(0,10,10))
# 设置坐标范围
#plt.xlim(0,2)
#plt.ylim(0,10)
# 设置坐标轴纵横比
# ref: 知乎-我的收藏
plt.axis('equal')
x, y = plotBezier([0,0],[0,3],[2,2],[2,5])
plt.plot(x, y, 'r')
x, y = plotBezier([0,0],[0,4],[2,2],[2,6])
#plt.plot(x, y, 'b')