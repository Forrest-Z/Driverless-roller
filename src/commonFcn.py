# -*- coding: utf-8 -*-
"""
Created on Wed Nov 15 16:27:24 2017

@author: qidong
"""
import numpy as np
import functools
import time
import math
from collections import deque
import matplotlib.pyplot as plt
import bisect
# import rospy

def timeit(func):
    """
    """
    @functools.wraps(func)
    def _wrapper(*args, **kwargs):
        time1 = time.time()
        ret = func(*args, **kwargs)
        time2 = time.time()
        print(func.__name__ + " cost time: {}".format(time2-time1))
        return ret
    return _wrapper

# def findOffsetLinePoint(startPoint,endPoint,distOffset):
#     '''
#     coordinate: global
#     input:
#         startPoint: point in global map
#         endPoint: point in global map
#         distOffset: the offset distance between two path planned
#     output:
#         startPointNew: the point that has been offseted
#         endPointNew: the point that has been offseted
#     '''
#     p1 = np.array([0.,distOffset])
#     #直线斜率
#     k =(endPoint[1] - startPoint[1])/(endPoint[0] - startPoint[0])
#     thetaAtrans = np.arctan(k) #
#
#     Atrans = np.array([[np.cos(thetaAtrans),-np.sin(thetaAtrans)],[np.sin(thetaAtrans),np.cos(thetaAtrans)]])
#     p2 = np.dot(Atrans,p1)
#     startPointNew = startPoint +p2
#     endPointNew = endPoint +p2
#     return startPointNew,endPointNew

def findOffsetLinePoint(startPoint,endPoint,distOffset):
    '''
    coordinate: global
    input:
        startPoint: point in global map
        endPoint: point in global map
        distOffset: the offset distance between two path planned
    output:
        startPointNew: the point that has been offseted
        endPointNew: the point that has been offseted
    正值往右偏，负值往左偏
    '''
    p1 = np.array([distOffset,0.])
    thetaAtrans = twoVectAngle(startPoint,endPoint)
    position = {'x':0., 'y':0., 'azimuth':thetaAtrans}
    p2 = local_2_global_point(position,p1)
    startPointNew = startPoint +p2
    endPointNew = endPoint +p2
    return startPointNew,endPointNew

'''
坐标转换：
针对右手坐标系，即常规xy坐标系，ｙ在x左边:
    假如有一个点(dX,dY).这个点绕原点逆时针旋转角度dAngle(弧度)。运行到新位置的坐标是：
    (iNewX, iNewY)。
    或者等价的说，保持这个点不动，但是旧坐标系绕原点顺时针转动dAngle到达新坐标系,(dX,dY)在
    新坐标系的坐标是(iNewX, iNewY).
左手坐标系，即之前的:
    转换矩阵内部上下互换，即xy互换
ref: http://blog.csdn.net/liji_digital/article/details/73136297
'''
def local_2_global_vect(position,point):
    '''
    局部坐标点转换为全局坐标,适用于一系列点向量输入,参考display.py
    input:
        position: 当前位置
        point: 需要转换的局部坐标点,形式为两行即point = np.array([x_S,y_S])
    ref: http://blog.csdn.net/linshanxian/article/details/68944748
    '''
    p1 = np.array(point)
    thetaAtrans = (position['azimuth'])*np.pi/180.  #刚好按照航向角顺时针到全局
    Atrans = np.mat([[np.cos(thetaAtrans),-np.sin(thetaAtrans)],[np.sin(thetaAtrans),np.cos(thetaAtrans)]])
    p2 = Atrans*np.mat(p1) #p2为mat
    x_global = np.array(p2.A[0])+position['x']
    y_global = np.array(p2.A[1])+position['y']
    point_global = np.array([x_global,y_global])

    return point_global

def local_2_global_point(position,point):
    '''
    局部坐标点转换为全局坐标,适用于单点输入
    input:
        position: 当前位置
        point: 需要转换的局部坐标点
    '''
    p1 = np.array(point)
    thetaAtrans = (position['azimuth'])*np.pi/180.  #逆时针弧度，而航向角是顺时针
    Atrans = np.array([[np.cos(thetaAtrans),-np.sin(thetaAtrans)],[np.sin(thetaAtrans),np.cos(thetaAtrans)]])
    p2 = np.dot(Atrans,p1) #
    point_global = p2 + np.array([position['x'],position['y']])

    return point_global

def global_2_local_point(position,point):
    '''
    全局坐标点转换为局部坐标,适用于单点输入
    input:
        position: 当前位置
        point: 需要转换的局部坐标点
    为何这里先平移后旋转的原因：
        首先先旋转后平移的步骤没有任何问题，但是有个前提：平移操作中的平移值应该是在新坐标系统
        中定义的。又因为两个算法中的平移值是固定的，即当前位置坐标（全局坐标定义），所以在由局
        部转全局时是正确的，但反过来就是错的，除非平移值改变，不采用当前位置坐标，而采用全局原
        点在车辆坐标系的坐标值即可，但这就麻烦多了。所以先平移还是旋转的关键是这个平移值是在哪
        个坐标系统中定义的。
    '''
    xCenterGlobal, yCenterGlobal = point
    p1 = np.array([xCenterGlobal-position['x'],yCenterGlobal-position['y']])
    thetaAtrans = (-position['azimuth'])*np.pi/180.  #逆时针弧度，而航向角是顺时针
    Atrans = np.array([[np.cos(thetaAtrans),-np.sin(thetaAtrans)],[np.sin(thetaAtrans),np.cos(thetaAtrans)]])
    p2 = np.dot(Atrans,p1) #
    point_local = p2

    return point_local

def global_2_local_vect(position,point):
    '''
    全局坐标点转换为局部坐标,适用于一系列点向量输入,参考local_planning.py
    input:
        position: 当前位置
        point: 需要转换的局部坐标点
    '''
    xCenterGlobal, yCenterGlobal = point
    p1 = np.array([xCenterGlobal-position['x'],yCenterGlobal-position['y']])
    thetaAtrans = (-position['azimuth'])*np.pi/180.  #逆时针弧度，而航向角是顺时针
    Atrans = np.mat([[np.cos(thetaAtrans),-np.sin(thetaAtrans)],[np.sin(thetaAtrans),np.cos(thetaAtrans)]])
    p2 = Atrans*np.mat(p1) #p2为mat
    xCenterLocal = np.array(p2.A[0])
    yCenterLocal = np.array(p2.A[1])
    point_local = np.array([xCenterLocal,yCenterLocal])

    return point_local

def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def saturation(minX,maxX,x):
    '''
    saturation(minX,maxX,x)
    '''
    return np.amin([maxX,np.amax([minX,x])])

def findClosestPointInit(position, wayPoint):
    '''
    find closest point in initialization,
    first search point must be start point
    'DIST_SEARCH_RADIUS + 5.' 是考虑到相邻几个点距离的突变
    '''
    DIST_SEARCH_RADIUS = 15.
    dist = []
    flagSearched = False
    for m in xrange(0, wayPoint.shape[0], 1):
        point_map = np.asarray([wayPoint[m,0],wayPoint[m,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        dist.append(distTemp)
        if distTemp < DIST_SEARCH_RADIUS:
            flagSearched = True
        if flagSearched == True:
            if distTemp > (DIST_SEARCH_RADIUS + 5.) or m == wayPoint.shape[0]-1:
                break
    try:
        indClosest = np.argmin(dist)
        flagClosestPointSearched = True
    except:
        print 'no point is available'
        indClosest = 0
        flagClosestPointSearched = False
    print 'indClosest',indClosest,wayPoint.shape[0]
    return indClosest, flagClosestPointSearched

# @timeit
def findClosestPoint(position, wayPoint, indLastClosest, distPreview, flagSearched):
    '''
    find closest point
    main idea: take the indLastClosest point as start point,
        then search toward left and right. It is the most important to
        break loop when abide by threshold condition, so as to reduce time cost
    input:
        position: current position
        wayPoint: way point in global coordinate, generated by np.column_stack
        indLastClosest: the index of last closest point
    '''
    flagClosestPointSearched = flagSearched
    dist = []
    DIST_SEARCH_ClOSE = 0.5 # [m]
    DIST_SEARCH_ENOUGH = 10. # [m] 此值与速度、规划周期相关,两次搜索期间距离 200km/h/3.6*0.1=5.6m
    NO_SEARCHED = 20.  # [m]
    for i in xrange(0,indLastClosest+1,1): # 先往初始点方向搜索
        k = indLastClosest - i
        point_map = np.asarray([wayPoint[k,0],wayPoint[k,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        dist.append(distTemp)
        if distTemp < DIST_SEARCH_ClOSE:
            break
        if flagClosestPointSearched == True:  # 算法需确保已经先找到一个最近点
            if distTemp > DIST_SEARCH_ENOUGH:
                break
    for j in xrange(indLastClosest,wayPoint.shape[0],1): # 再往终点搜索
        point_map = np.asarray([wayPoint[j,0],wayPoint[j,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        dist.append(distTemp)
        if distTemp < DIST_SEARCH_ClOSE:
            break
        if flagClosestPointSearched == True:  # 算法需确保已经先找到一个最近点
            if distTemp > DIST_SEARCH_ENOUGH:
                break
    distClosed = np.amin(dist)
    # print 'i,j', i,j-indLastClosest
    if distClosed > NO_SEARCHED:
        flagClosestPointSearched = False
        print("can not find the closed point. it's too far from map")
        indClosest = indLastClosest
    else:
        flagClosestPointSearched = True
        indClosestTemp = np.argmin(dist)
        if indClosestTemp <= i:  # 说明最近点在第一部分,indLastClosest是前半部分可能取的最大值
            indClosest = indLastClosest - indClosestTemp
        else:
            # note: 前半部分长度为i+1，最大序号是i
            indClosest = indLastClosest + (indClosestTemp - i) - 1

    # find preview point
    for j in xrange(indClosest,wayPoint.shape[0],1): # 由最近点往终点搜索
        point_map = np.asarray([wayPoint[j,0],wayPoint[j,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        maxSearchableInd = (wayPoint.shape[0]-indClosest)-1
        if distTemp > distPreview:
            if j < indClosest+10:      # 考虑最近点就是预瞄点的情况,强制增加至10个点,出山店两点距离0.075m
                indPre = indClosest + np.amin([10,maxSearchableInd])  # finded
            else:           # normal
                indPre = j  # finded
            break
        elif j >= wayPoint.shape[0]-1:     #搜索到最后一个点
            indPre = wayPoint.shape[0]-1
            break
    return indClosest, indPre, flagClosestPointSearched

def findClosestPointSteer(position, wayPoint, indLastClosest, distPreview):
    '''
    find closest point
    main idea: take the indLastClosest point as start point,
        then search toward left and right. It is the most important to
        break loop when abide by threshold condition, so as to reduce time cost
    input:
        position: current position
        wayPoint: way point in global coordinate, generated by np.column_stack
        indLastClosest: the index of last closest point
    '''
    dist = []
    DIST_SEARCH_ClOSE = 0.5 # [m]
    NO_SEARCHED = 20.  # [m]
    for i in xrange(0,indLastClosest+1,1): # 先往初始点方向搜索
        k = indLastClosest - i
        point_map = np.asarray([wayPoint[k,0],wayPoint[k,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        dist.append(distTemp)
        if distTemp < DIST_SEARCH_ClOSE:
            break
    for j in xrange(indLastClosest,wayPoint.shape[0],1): # 再往终点搜索
        point_map = np.asarray([wayPoint[j,0],wayPoint[j,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        dist.append(distTemp)
        if distTemp < DIST_SEARCH_ClOSE:
            break
    distClosed = np.amin(dist)
    if distClosed > NO_SEARCHED:
        print("can not find the closed point in steer. it's too far from map")
        indClosest = indLastClosest
    else:
        indClosestTemp = np.argmin(dist)
        if indClosestTemp <= i:  # 说明最近点在第一部分,indLastClosest是前半部分可能取的最大值
            indClosest = indLastClosest - indClosestTemp
        else:
            # note: 前半部分长度为i+1，最大序号是i
            indClosest = indLastClosest + (indClosestTemp - i) - 1

    # find preview point
    for j in xrange(indClosest,wayPoint.shape[0],1): # 由最近点往终点搜索
        point_map = np.asarray([wayPoint[j,0],wayPoint[j,1]])
        point_self = np.asarray([position['x'],position['y']])
        distTemp = np.linalg.norm(point_self-point_map)
        maxSearchableInd = (wayPoint.shape[0]-indClosest)-1
        if distTemp > distPreview:
            if j < indClosest+10:      # 考虑最近点就是预瞄点的情况,强制增加至10个点,出山店两点距离0.075m
                indPre = indClosest + np.amin([10,maxSearchableInd])  # finded
            else:           # normal
                indPre = j  # finded
            break
        elif j >= wayPoint.shape[0]-1:     #搜索到最后一个点
            indPre = wayPoint.shape[0]-1
            break
    return indClosest, indPre

def calcCurvature(polyCoeff, x):
    '''
    Parameter:
        polyCoeff:  polynomial coefficient
        x:
    '''
    # 一阶导 ref:http://blog.csdn.net/blog_empire/article/details/39298557
    polyder1Coeff = np.polyder(polyCoeff) #导函数多项式系数
    polyder1 = np.polyval(polyder1Coeff,x)
    # 二阶导
    polyder2Coeff = np.polyder(polyder1Coeff)
    polyder2 = np.polyval(polyder2Coeff,x)
    #曲率或曲率数组
    curvature = np.abs(polyder2) / ((1+polyder1**2)**1.5)
    return curvature

def calcQuadCurvature(polyCoeff, x):
    '''
    only apply to quadratic polynomial
    Parameter:
        polyCoeff:  polynomial coefficient
        x:
    '''
    curvature = np.abs(2 * polyCoeff[0]) / \
            ((1 + (2 * polyCoeff[0] * x  + polyCoeff[1])**2)**1.5)
    return curvature

def calcCoeffLatDeviation(d0,df,sf):
    '''
    calculate polynomial coefficient of function, i.e., lateral deviation = f(S)
    Parameter:
        d0: lateral Deviation to ref path at current position
        df: lateral Deviation to ref path at end position
        sf: length of ref path used in planning period
    ref: 《Real-Time Trajectory Planning for Autonomous urban driving-framework,
        algorithms, and verifications》 paper's formula.7 .11 is wrong
    '''
    s0 = 0.
    theta0 = 0.
    A = np.array([[sf**3,sf**2,sf,1.],[s0**3,s0**2,s0,1.],[3*sf**2,2*sf,1.,0.],[3*s0**2,2*s0,1.,0.]])
    b = np.array([df, d0, 0., np.tan(theta0)])
    Coeff = np.linalg.solve(A,b) # [c3 c2 c1 c0]

    return Coeff

def genCandidate(d0,df,wayPointLocal):
    '''
    # 已淘汰
    generate local path candidate
    Coordinate:
        vehicle local coordinate
    Parameter:
        wayPointLocal: wayPoint in local coordinate, two row

    '''
    xWay = wayPointLocal[0,:]
    yWay = wayPointLocal[1,:]
    # calculate distance between two adjacent points
    dx = np.diff(xWay)
    dy = np.diff(yWay)
    dS = np.array([np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)])
    # calc frenet azimuth, p(i) --> p(i+1)
    angle = np.arccos(dx/dS)  # 0-pi
    angle[dy < 0.] -= 2*np.pi
    angle[angle < 0.] *= -1
    # calculate all discrete distance at sample point
    SAll = np.cumsum(dS)
    # calc the whole length of way during plan period
    sf = SAll[-1]
    CoeffLatDeviation = calcCoeffLatDeviation(d0,df,sf)
    latDeviation = np.polyval(CoeffLatDeviation,SAll)
    # 生成单条candidate
    # 模仿local_2_global_point, p2为两列多行，第一列x
    p2 = np.array([np.dot(np.array([[np.cos(thetaAtrans),-np.sin(thetaAtrans)], \
        [np.sin(thetaAtrans),np.cos(thetaAtrans)]]),np.array([0.,iLat])) for \
        (thetaAtrans,iLat) in zip(angle, latDeviation)])
    xCandidate = p2[:,0] + xWay[0:-1]
    yCandidate = p2[:,1] + yWay[0:-1]
    pointCandidate = np.array([xCandidate,yCandidate])

    return pointCandidate

def calcDistPoint2Line(polyCoeff,point):
    x,y = point
    A_Coeff,C_Coeff = polyCoeff
    B_Coeff = -1.
    dist = np.abs((A_Coeff*x+B_Coeff*y+C_Coeff)) / np.sqrt(A_Coeff**2.+B_Coeff**2.)

    return dist

def checkSidePoint2Line(PA, PB, PC):
    '''
    原理：　取最近点Ａ,下一个点B，当前点C。利用叉乘运算，S = CA X CB：
    S > 0 ，CA逆时针到CB,左侧; S < 0 ，CA顺时针到CB，右侧
    令矢量的起点为A(x1,y1)，终点为B(x2,y2)，判断的点为C(x3,y3):
    如果S（A，B，C）为正数，则C在矢量AB的左侧；
    如果S（A，B，C）为负数，则C在矢量AB的右侧；
    如果S（A，B，C）为0，则C在直线AB上。
    注：两向量之间夹角以小于180度计算,常规坐标系
    ref: http://blog.csdn.net/modiz/article/details/9928955
         http://blog.csdn.net/u013378306/article/details/51868384
         http://blog.csdn.net/u010429424/article/details/44829325
    '''
    x1, y1 = PA
    x2, y2 = PB
    x3, y3 = PC
    S = (x1-x3)*(y2-y3)-(y1-y3)*(x2-x3)
    # S = -S  # 设定右边为正, 大地水平坐标系与普通坐标系不同，所以刚好不用反向,(y,x)
    return S

class meanFilter(object):
    def __init__(self,num=5):
        '''
        num: 取多少个测量值用于平均
        ref: http://blog.csdn.net/qins_superlover/article/details/44338415
        '''
        self.raw = deque(maxlen=num)

    def update(self,newData):
        self.raw.append(newData)
        return np.mean(self.raw)

class PID(object):
    '''
    PID参数运行时需要自适应变化的话，就放在update, 否则init
    '''
    def __init__(self, Ts, filterN = 100):
        self.integral = 0.
        self.filter = 0.
        self.filterN = filterN
        self.Ts = Ts

    def update(self, Err, Kp, Ki, Kd):
        filterCoeff = (Kd * Err - self.filter) * self.filterN
        outPID = Kp * Err + self.integral + filterCoeff
        self.integral += Ki * Err * self.Ts
        self.filter += filterCoeff * self.Ts

        return outPID

def genBezier(point):
    '''
    3 order bezier
    point = np.array([[x0,x1,x2,x3], [y0,y1,y2,y3]])
    '''
#    pointRoadBessel = np.array([point0,point1,point2,point3]) #4行2列
    pointRoadBessel = np.array([point[:,0],point[:,1],point[:,2],point[:,3]])
    ARoadBessel = np.array([[-1,3,-3,1],[3,-6,3,0],[-3,3,0,0],[1,0,0,0]])
    polyCoeffiCandidate = np.dot(ARoadBessel,pointRoadBessel) #t**3 t**2 t 1,note: the variable is neither x nor y, but t
    #        pYRoadBessel = np.dot(ARoadBessel,pointRoadBessel[:,1]) #Look
    t = np.linspace(0,1,np.max([100,50]))
    x_S = np.polyval(polyCoeffiCandidate[:,0],t)
    y_S = np.polyval(polyCoeffiCandidate[:,1],t)
    pointBezier = np.array([x_S, y_S])
    return pointBezier

def bezierSmooth(wayPoint):
    numWayPoint = wayPoint.shape[1]
    point0 = wayPoint[:,0]
    point1 = wayPoint[:,np.floor(1./3.*numWayPoint).astype(np.uint32)]
    point2 = wayPoint[:,np.floor(2./3.*numWayPoint).astype(np.uint32)]
    point3 = wayPoint[:,np.floor(3./3.*numWayPoint).astype(np.uint32)-1]
    #######################################################################
    # 变换3个点
    #######################################################################
    pointRoadBessel = np.array([point0,point1,point2,point3]) #4行2列
    ARoadBessel = np.array([[-1,3,-3,1],[3,-6,3,0],[-3,3,0,0],[1,0,0,0]])
    polyCoeffiCandidate = np.dot(ARoadBessel,pointRoadBessel) #t**3 t**2 t 1,note: the variable is neither x nor y, but t
#        pYRoadBessel = np.dot(ARoadBessel,pointRoadBessel[:,1]) #Look
    t = np.linspace(0,1,np.max([100,numWayPoint]))
    x_S = np.polyval(polyCoeffiCandidate[:,0],t)
    y_S = np.polyval(polyCoeffiCandidate[:,1],t)
    return x_S, y_S

# @timeit
def bezierBased(df, wayPointLocal):
    numWayPoint = wayPointLocal.shape[1]
    point0 = np.array([0,0])  # 从当前点出发
    point1Temp = wayPointLocal[:,np.floor(1./3.*numWayPoint).astype(np.uint32)]
    point2Temp = wayPointLocal[:,np.floor(2./3.*numWayPoint).astype(np.uint32)]
    point3Temp = wayPointLocal[:,np.floor(3./3.*numWayPoint).astype(np.uint32)-1]
    #######################################################################
    # 变换3个点
    #######################################################################
    pointTemp = np.array([point1Temp,point2Temp,point3Temp])
    #  在车辆坐标系下通过求导求解终点处的旋转角度
    #一阶导
    polyCoeffiCenter = np.polyfit(wayPointLocal[0,:],wayPointLocal[1,:],2)
    polyder1Coeffi = np.polyder(polyCoeffiCenter) #导函数多项式系数
    polyder1 = np.polyval(polyder1Coeffi,pointTemp[:,0])
    thetaAtrans = np.arctan(polyder1) #正负恰好符合frenet到车辆坐标的顺时针角度
    p2 = np.array([-np.sin(thetaAtrans), np.cos(thetaAtrans)]) * df
    p2 = p2.T
    point1 = point1Temp + p2[0]
    point2 = point2Temp + p2[1]
    point3 = point3Temp + p2[2]
    # bezier
    pointRoadBessel = np.array([point0,point1,point2,point3]) #4行2列
    ARoadBessel = np.array([[-1,3,-3,1],[3,-6,3,0],[-3,3,0,0],[1,0,0,0]])
    polyCoeffiCandidate = np.dot(ARoadBessel,pointRoadBessel) #t**3 t**2 t 1,note: the variable is neither x nor y, but t
#        pYRoadBessel = np.dot(ARoadBessel,pointRoadBessel[:,1]) #Look
    t = np.linspace(0,1,np.max([100,numWayPoint]))
    x_S = np.polyval(polyCoeffiCandidate[:,0],t)
    y_S = np.polyval(polyCoeffiCandidate[:,1],t)
    pathCandidate = np.array([x_S,y_S])
    return pathCandidate

def laneBased(d0,df,wayPointLocal):
    '''
    generate local path candidate
    Coordinate:
        vehicle local coordinate
    Parameter:
        wayPointLocal: wayPoint in local coordinate, two row

    '''
    xWay = wayPointLocal[0,:]
    yWay = wayPointLocal[1,:]
    # calculate distance between two adjacent points
    dx = np.diff(xWay)
    dy = np.diff(yWay)
    # dS = np.array([np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)])
    dS = np.sqrt(dx ** 2 + dy ** 2)
    # calc frenet azimuth, p(i) --> p(i+1)
    angle = np.arccos(dx/dS)  # 0-pi
    angle[dy < 0.] -= 2*np.pi
    angle[angle < 0.] *= -1
    # calculate all discrete distance at sample point
    SAll = np.cumsum(dS)
    # calc the whole length of way during plan period
    sf = SAll[-1]
    # CoeffLatDeviation = commonFcn.calcCoeffLatDeviation(d0,df,sf)  -------
    # latDeviation = np.polyval(CoeffLatDeviation,SAll)              -------
    latDeviation = df
    # 生成单条candidate
    # 模仿local_2_global_point, p2为两列多行，第一列x
    # 与通常坐标转换转换到同一个新坐标系不同，这里的坐标转换，每一个点对应不同的新坐标系
    # p2 = np.array([np.dot(np.array([[np.cos(thetaAtrans),-np.sin(thetaAtrans)], \
    #     [np.sin(thetaAtrans),np.cos(thetaAtrans)]]),np.array([0.,iLat])) for \
    #     (thetaAtrans,iLat) in zip(angle, latDeviation)])
    thetaAtrans = angle
    p2 = np.array([-np.sin(thetaAtrans), np.cos(thetaAtrans)]) * latDeviation
    p2 = p2.T
    # p2 = get_point(thetaAtrans,latDeviation)
    xCandidate = p2[:,0] + xWay[0:-1]
    yCandidate = p2[:,1] + yWay[0:-1]
    pathCandidate = np.array([xCandidate,yCandidate])

    return pathCandidate

def changeLanePlan(polyCoeff, position, lineMapStartPoint, lineMapEndPoint, lenBezier=3):
    '''
    polyCoeff: 下一步要碾压的直线方程系数
        [k, b]
    position: 当前位置，字典
    idea:
        求取四个点作为beizier的四个顶点，再bezier拟合
    Return:
        bezier规划的换道全局路点，从当前位置到换道切点
    '''
    k, b = polyCoeff
#    lenBezier = 3. # bezier两个定点之间的距离
    x0 = 0.
    y0 = 0.
    theata = np.arctan(k)
    # 求x1，平移直线至当前点(0,0)，y=kx
    x1 = abs(lenBezier*np.cos(theata))
    y1 = k*x1
    # 求x2， x3, y = kx + b
    x2 = abs(2.*np.cos(theata))
    y2 = k*x2 + b
    x3 = abs((lenBezier+2.)*np.cos(theata))
    y3 = k*x3 + b
    xAll = np.array([x0, x1, x2, x3])
    yAll = np.array([y0, y1, y2, y3])
    pointAll = np.array([xAll, yAll])
    # 转换到全局坐标
    # 以下一条直线航向为航向
    angle = twoVectAngle(lineMapStartPoint, lineMapEndPoint)
    curPosition = {'x':position['x'], 'y':position['y'], 'azimuth':angle}
    pointAllGlobal = local_2_global_vect(curPosition,pointAll)
    wayPointChangeLane = genBezier(pointAllGlobal)

    return wayPointChangeLane

def twoVectAngle(p1, p2):
    '''
    两点向量顺时针转动到y轴的角度 0-360
    利用cos
    '''
    p1 = np.array(p1)
    p2 = np.array(p2)
    x1, y1 = p1
    x2, y2 = p2
    dist = np.linalg.norm(p2-p1)
    xDist = x2-x1
    yDist = y2-y1
    theata = np.arccos(abs(xDist)/dist)*180./np.pi
    if xDist >= 0.:
        # 1, 4象限
        if yDist >= 0.:
            # 1
            angle = theata + 270.
        else:
            angle = 270. - theata
    else:
        if yDist <= 0.:
            angle = theata + 90.
        else:
            angle = 90. - theata
    return angle

def xAxis2VectCounter(p1, p2):
    '''
    x轴逆时针到向量的角度 0-360
    利用cos
    '''
    p1 = np.array(p1)
    p2 = np.array(p2)
    x1, y1 = p1
    x2, y2 = p2
    dist = np.linalg.norm(p2-p1)
    xDist = x2-x1
    yDist = y2-y1
    theata = np.arccos(abs(xDist)/dist)*180./np.pi
    if xDist >= 0.:
        # 1, 4象限
        if yDist >= 0.:
            # 1
            angle = theata
        else:
            angle = 360. - theata
    else:
        if yDist < 0.:
            angle = theata + 180.
        else:
            angle = 180. - theata
    return angle

if __name__ == '__main__':
    print twoVectAngle([0,0],[-1,0])
