#! /usr/bin/python
# -*- coding: utf-8 -*-
"""

author: qidong
"""
import rospy
import sys
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
from campus_driving.msg import VehState,PlanOP
from mapEndPointDraw import findMapEndPoint
import commonFcn

cur_path = os.path.dirname(__file__)
SAFEDIST = 3.  # [m]
class State(object):

    def __init__(self):
        # 求取矩形区域四个顶点，四条边
        # if taskTypeGLB == 'csdLine':
        mapEndPoint, numGPSallPoint = findMapEndPoint(numAllLaneGLB,distBetweenLaneGLB) # (numAllLaneGLB,distOffsetStep=1.5)
        print 'mapEndPoint',mapEndPoint
        # 向distBetweenLaneGLB反方向一侧平移安全距离，获取1 2点
        # 向左偏移，得到最左边
        startPoint = mapEndPoint[0][0]
        endPoint = mapEndPoint[0][1]
        offsetDist = -np.sign(distBetweenLaneGLB)*SAFEDIST
        self.point0, self.point1  = commonFcn.findOffsetLinePoint(startPoint,endPoint,offsetDist)
        # 按原偏移方向继续平移一次安全距离，获取3 4点
        # 向右偏移得到最右边
        startPoint = mapEndPoint[-1][0]
        endPoint = mapEndPoint[-1][1]
        offsetDist = np.sign(distBetweenLaneGLB)*SAFEDIST
        self.point3, self.point2  = commonFcn.findOffsetLinePoint(startPoint,endPoint,offsetDist)
        # 再将上述两条边的四个顶点组成的前后两条边，进行前后偏移一定距离
        # 以前边为参考，则应对其采取右偏
        startPoint = self.point0
        endPoint = self.point3
        offsetDist = SAFEDIST
        if taskTypeGLB == 'csdCircle':
            offsetDist = SAFEDIST + distBetweenLaneGLB/2.
        self.point0, self.point3  = commonFcn.findOffsetLinePoint(startPoint,endPoint,offsetDist)
        # 以后边为参考，则应对其采取左偏
        startPoint = self.point1
        endPoint = self.point2
        offsetDist = -SAFEDIST
        self.point1, self.point2  = commonFcn.findOffsetLinePoint(startPoint,endPoint,offsetDist)
        plt.plot(self.point0,'ro')
        plt.plot(self.point1,'ro')
        plt.plot(self.point2,'ro')
        plt.plot(self.point3,'ro')

        # elif taskTypeGLB == 'csdCircle':
        #     pass
        self.stopSafeArea = False

    def update(self):
        # 接收当前位置坐标，判断是否在矩形区域内
        # 判断方法： 向量叉积, 左正右负
        # line1
        line0 = np.array([self.point0, self.point1])
        line1 = np.array([self.point1, self.point2])
        line2 = np.array([self.point2, self.point3])
        line3 = np.array([self.point3, self.point0])
        lineAll = [line0, line1, line2, line3]
        numLine = len(lineAll)
        curPosition = np.array([recv['x'], recv['y']])
        curPositionAll = [curPosition]*numLine
        pointSideAll = map(self.checkInPolygon, lineAll, curPositionAll) # 为一个多维list
        # print 'pointSideAll', pointSideAll
        # print 'cur',curPosition
        # print 'point', line0,line1,line2,line3
        if np.sign(distBetweenLaneGLB) > 0.:
            # 往右偏移，多边形定点顺时针，点在直线左侧为外部
            if np.max(pointSideAll) > 0.:
                self.stopSafeArea = True
        else:
            # 往左偏移，多边形定点逆时针，点在直线右侧为外部
            if np.min(pointSideAll) < 0.:
                self.stopSafeArea = True
        print 'stopSafeArea',self.stopSafeArea

    def checkInPolygon(self, line, curPosition):
        # 判断方法： 向量叉积, 左正右负
        PA = line[0]
        PB = line[1]
        pointSide = commonFcn.checkSidePoint2Line(PA, PB, curPosition)
        pointSide = np.sign(pointSide)
        return pointSide

    def publish(self):
        msg = PlanOP()
        # self.stopSafeArea = False
        msg.stopSafeArea = self.stopSafeArea
        msg.header.stamp = rospy.Time.now()
        safeAreaPub.publish(msg)

def getVehState(data):
    recv['x'] = data.x
    recv['y'] = data.y
    recv['z'] = data.z

if __name__ == '__main__':
    rospy.init_node('vehicle', anonymous = False)
    recv = {'x':0., 'y':0., 'z':0.}
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

    rospy.Subscriber("vehState", VehState, getVehState)
    safeAreaPub = rospy.Publisher("safeArea", PlanOP, queue_size =1)

    state = State()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        state.update()
        state.publish()
        rate.sleep()
    rospy.spin()    #rospy.spin()作用是当节点停止时让python程序退出，和C++ spin的作用不同
#    sys.exit()
