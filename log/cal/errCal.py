# -*- coding: utf-8 -*-
"""
Created on Fri Dec  8 21:51:16 2017

@author: lwb
"""

import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
if "../../" not in sys.path:
    sys.path.append("../../src")
import commonFcn

def errorCalculation(startPoint,endPoint,acualTimeData):
#    startPoint = np.array([0,0])
#    endPoint = np.array([3,4])
#    dist = np.linalg.norm(endPoint-startPoint)
#    a = [1,2,3,4,5,6,7,8]
#    b = np.amax(a)
#    c = np.argmax(a)

    """
    求规划路径斜率和截距
    """
    slopeExpectation = (endPoint[1]-startPoint[1])/(endPoint[0]-startPoint[0])
    interceptExpectation = startPoint[1] - startPoint[0]*slopeExpectation
    """
    求第一条路径误差
    """
    dataLength = np.shape(acualTimeData)
    dataLength = dataLength[0]
    err = []
    for i in xrange(dataLength):
        acualTimeDataSingle = acualTimeData[i]
        trackExcettation = slopeExpectation*acualTimeDataSingle[0]+ interceptExpectation
        errCal = np.abs(trackExcettation - acualTimeDataSingle[1])
        err.append(errCal)

    errMax = np.max(err)
    errMean = np.mean(err)
    angle = np.arctan(slopeExpectation)
    errMax = errMax*np.cos(angle)
    errMean = errMean*np.cos(angle)
    return errMax,errMean

if __name__ == '__main__':

    cur_path = os.path.dirname(__file__)
    logPositionAll = np.load(os.path.join(cur_path,'logPositionAll.npy'))
    mapEndPoint = np.load(os.path.join(cur_path,'mapEndPoint.npy'))
    '''
    第一条路径误差计算
    '''
    firstPathEnd = mapEndPoint[0]
    startPoint1 = firstPathEnd [0]
    endPoint1 = firstPathEnd  [1]
    acualTimeData1 = logPositionAll[0]
    errMax1,errMean1 = errorCalculation(startPoint1,endPoint1,acualTimeData1)
    print("first path: ","errMax",errMax1,"errMean",errMean1)
    '''
#    第二条路径误差计算
#    '''
    secondPathEnd = mapEndPoint[1]
    startPoint2 = secondPathEnd [0]
    endPoint2 = secondPathEnd  [1]
    acualTimeData2 = logPositionAll[1]
    errMax2,errMean2 = errorCalculation(startPoint2,endPoint2,acualTimeData2)
    print("second path: ","errMax",errMax2,"errMean",errMean2)

    '''
    统计跟踪误差
    '''
    logDistErr = np.load(os.path.join(cur_path,'logDistErr.npy'))
    logEta = np.load(os.path.join(cur_path,'logEta_Delta.npy'))[:,0]
    logDelta = np.load(os.path.join(cur_path,'logEta_Delta.npy'))[:,1]
    rawMap = np.load(os.path.join(cur_path,'rawMap.npy'))

    maxDistErr = np.amax(logDistErr)
    minDistErr = np.amin(logDistErr)
    print 'minDistErr->maxDistErr: ', minDistErr,'->', maxDistErr
    maxEta = np.amax(logEta)
    minEta = np.amin(logEta)
    print 'minEta->maxEta: ', minEta,'->', maxEta
    maxDelta = np.amax(logDelta)
    minDelta = np.amin(logDelta)
    print 'minDelta->maxDelta: ', minDelta,'->', maxDelta

    pitch = np.load(os.path.join(cur_path,'pitch.npy'))
    x = np.arange(pitch.size)
    maxPitch = np.amax(pitch)
    minPitch = np.amin(pitch)
    print 'maxPitch,minPitch',maxPitch,minPitch
#    plt.plot(x,pitch,'r')

    a = commonFcn.meanFilter(50)
    pitchMean = [a.update(i) for i in pitch]
    pitchMean = np.array([pitchMean]).T
#    plt.plot(x,pitchMean)

    logVelCtr = np.load(os.path.join(cur_path,'logVelCtr.npy'))
    AX = logVelCtr[:,4]
    MAXAX = np.amax(AX)
    MINAX = np.amin(AX)
    print 'MAXAX,MINAX',MAXAX,MINAX

    xMap = np.linspace(0,100,1000)
    yMap = np.zeros_like(xMap)
#    yMap = np.random.randn(1000)
    # 或直接np.load
    # B spline构造密集路点
    spl = UnivariateSpline(xMap,yMap)
    spl.set_smoothing_factor(1.) # 1:最大平滑值 0:直接插值
    xCenterGlobal = np.linspace(xMap[0],xMap[-1],100) #100./10000. = 0.01m
    yCenterGlobal = spl(xCenterGlobal)
    plt.plot(xCenterGlobal,yCenterGlobal)
    zCenterGlobal = np.zeros_like(xCenterGlobal)
    GPSallPoint = np.column_stack((xCenterGlobal,yCenterGlobal,zCenterGlobal))
    np.save(os.path.join(cur_path,'map_gen/simuMap.npy'),GPSallPoint)
