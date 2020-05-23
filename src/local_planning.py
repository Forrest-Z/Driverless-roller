#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
input:
    vehicle state, 	occupancy grid map
output:
    way point in global coordinate
    the reason why not to choose the eta or the coefficient of local path:
        eta: lead to reduce control frequency. Deviation feedback should be
            estimate in steer_control.py directly.
        coefficient: in this case, it has the same frequency problem as eta,
            otherwise, if increase the control frequence forcedly, will bring in
            principle mistake, i.e., the coefficient is not the coefficient over time.
self.flagChangesPre检测正在起步,修改sPre的算法暂时屏蔽
author: qidong
'''
import rospy
import numpy as np
import sys
import time
import os
import yaml
import bisect
from scipy.interpolate import UnivariateSpline
#from scipy import interpolate
from std_msgs.msg import MultiArrayDimension
from nav_msgs.msg import OccupancyGrid
from campus_driving.msg import INSPVAX, mcuFlag, stm32TX
from campus_driving.msg import PathDisplay,PlanOP,VehState
from mapEndPointDraw import findMapEndPoint, genMapPoint
from mapCurveDraw import genCurveMap
import commonFcn
from commonFcn import timeit
import vehicle_model

# from lib_coord import get_point

INIT = vehicle_model.INIT
NORMAL = vehicle_model.NORMAL
STOP = vehicle_model.STOP
STOP_SAFE = vehicle_model.STOP_SAFE
DECELERATE = vehicle_model.DECELERATE
FORWARD = vehicle_model.FORWARD
BACK = vehicle_model.BACK
shift_P = vehicle_model.shift_P
shift_R = vehicle_model.shift_R
shift_N = vehicle_model.shift_N
shift_D = vehicle_model.shift_D
START_TRACE_INIT = vehicle_model.start_trace_init
DIST_OBST_OFFSET_FORWARD = vehicle_model.DIST_OBST_OFFSET_FORWARD
DIST_OBST_OFFSET_BACK = vehicle_model.DIST_OBST_OFFSET_BACK

GLBlogDistErr = []
cur_path = os.path.dirname(__file__)
countCALIBRTGLB = 0
flagPositionReceivedGLB = False
CenterGlobalEndGLB = np.array([[0,1,2],[3,4,5]])  # 保存路径，在终点无路时，可以调用之前的路点
class Planning(object):
    def __init__(self):
        self.logPosition = []
        self.logPositionAll = []
        self.flagLogPosition = 0
        self.flagIsLogging = False
        self.indOptimalLast = 10
        self.desireDirDrive = FORWARD # desireDirDrive是为了触发底层换挡
        self.curDirDrive = self.desireDirDrive # curDirDrive是为了协调内部程序，如航向变换
        self.flagIsShifting = 0
        self.distLog = 0
        self.countSingleLane = 0
        self.countLaneFinish = 0
        self.flagLaneFinish = 0
        self.flagMCUIsShifting = 0
        self.flagChangesPre = True
        self.flagLogPointStart = True
        self.pointStart = np.array([0,0])
        self.newStraightPathFlag = False # 准备错距到新的直线碾压路径，用来合并换道规划路径
        self.cloudTaskID = 0 # 云端接收到新任务，ID增1，通过此变量判断是否有新任务，有新任务后，重置相关标志位和地图
        self.mapEndPoint, self.numGPSallPoint = findMapEndPoint(numAllLaneGLB,distBetweenLaneGLB) # (numAllLaneGLB,distOffsetStep=1.5)
        np.save(os.path.join(cur_path,'../log/mapEndPoint.npy'),self.mapEndPoint)
        self.indexCircleMap = 0
        if mapTypeGLB == 'raw':
            self.GPSmap = np.load(os.path.join(cur_path,'../map/rawMap.npy'))
        else:
            if taskTypeGLB == 'csdCircle':
                genCurveMap()
                self.GPSmap = np.load(os.path.join(cur_path,''.join(['../map/curveMap', str(self.indexCircleMap), '.npy'])))
            elif taskTypeGLB == 'csdLine':
                self.GPSmap = genMapPoint(self.mapEndPoint[self.countLaneFinish][0],self.mapEndPoint[self.countLaneFinish][1],self.numGPSallPoint)
        while not flagPositionReceivedGLB:
            print 'is waiting vehState topic'
            if rospy.is_shutdown():
                 sys.exit()
            time.sleep(0.02)
        self.indLastClosest, self.flagClosestPointSearched = commonFcn.findClosestPointInit(recv, self.GPSmap)
        self.indLastPre = self.indLastClosest # 检测是否换地图
        self.dataPlanning = {'xCtrPath':[], 'yCtrPath':[], \
                    'xGlbPath':[], 'yGlbPath':[], \
                    'pathCandidate':np.array([[0,0],]), 'indOptimal':0, \
                    'modeDrive':NORMAL, 'distObst':1000.}
        self.distYClose = 0.
        self.modeDrive = NORMAL
        self.prePoint = []
        self.toCloudStartedFlag = False # 0: 未开始碾压任务； 1: 已按需开始云端任务
        self.toCloudEndTaskFlag = False # 0: 任务未结束； 1: 当前碾压任务已完成

    def calibrate(self):
        # Calibrated Parameters
        global CALIBRT
        flagOpenCalib = rospy.get_param('~GLBflagOpenCalib') # 控制标定量接收程序
        if flagOpenCalib == 1:
            CALIBRT = rospy.get_param('~CALIBRT')
            rospy.set_param('~GLBflagOpenCalib', 0)
    # @timeit
    def publish(self):
        '''
        result = {'xCtrPath':xCtrPath, 'yCtrPath':yCtrPath, \
                    'xGlbPath':xCenterGlobal, 'yGlbPath':yCenterGlobal, \
                    'pathCandidate':pathCandidate, 'indOptimal':indOptimal \
                    'modeDrive':modeDrive, 'distObst':closedObstaclDist}
        '''
        PathMsg = PathDisplay()
        PathMsg.xGlbPath = self.dataPlanning['xGlbPath']
        PathMsg.yGlbPath = self.dataPlanning['yGlbPath']
        # different from the usage in std_msgs
        dim = MultiArrayDimension()
        dim.stride = int(self.dataPlanning['indOptimal'])
        dim.size = self.dataPlanning['pathCandidate'].shape[0]

        PathMsg.xLocPath.data = np.array(self.dataPlanning['pathCandidate'][:,0]).flatten()
        PathMsg.xLocPath.layout.dim = [dim]
        PathMsg.yLocPath.data = np.array(self.dataPlanning['pathCandidate'][:,1]).flatten()
        PathMsg.yLocPath.layout.dim = [dim]
        PathMsg.header.stamp = rospy.Time.now()
        basePathPub.publish(PathMsg)

        PlanMsg = PlanOP()
        PlanMsg.prePoint.x = self.prePoint[0]
        PlanMsg.prePoint.y = self.prePoint[1]
        PlanMsg.prePoint.z = 0.
        PlanMsg.xCtrPath = self.dataPlanning['xCtrPath']
        PlanMsg.yCtrPath = self.dataPlanning['yCtrPath']
        PlanMsg.distObst = self.dataPlanning['distObst']
        PlanMsg.desireDirDrive = self.desireDirDrive
        PlanMsg.curDirDrive = self.curDirDrive
        PlanMsg.flagIsShifting = self.flagIsShifting
        PlanMsg.distYClose = self.distYClose
        PlanMsg.modeDrive = self.modeDrive
        PlanMsg.startedFlag = self.toCloudStartedFlag
        PlanMsg.endTaskFlag = self.toCloudEndTaskFlag
        PlanMsg.header.stamp = rospy.Time.now()
        planPub.publish(PlanMsg)

    @timeit
    def update(self):
        CAN_flagStart_trace = recv['start_trace']
        modeDriveLane = self.changeDirection(recv)
        indClosest, indPre, self.flagClosestPointSearched = commonFcn.findClosestPoint( \
            recv, self.GPSmap, self.indLastClosest, CALIBRT['distPreview'], \
            self.flagClosestPointSearched)
        self.indLastClosest = indClosest
        self.indLastPre = indPre
        self.prePoint = self.GPSmap[indPre,:]
        # 考虑找不到点的情况，此时停止规划
        # if indClosest >= self.GPSmap.shape[0]-1:
        if self.flagClosestPointSearched == False:
            modeDriveSafe = STOP
            rospy.logwarn('no point is available')
        elif modeDriveLane == STOP:
            # 防止到达终点时，无路点可用，导致polyfit等一系列函数报错
            rospy.loginfo('is stoping and pausing planning')
        else:
            # try:
            self.dataPlanning = self.localPlanning(indClosest,indPre,recv)
            modeDriveSafe = self.dataPlanning['modeDrive']
            # except Exception, e:
            #     modeDriveSafe = STOP_SAFE
            #     rospy.logerr('localPlanning run error')
                # print 'repr(e):\t', repr(e)
            # merge the modeDrive output
        # CAN_flagStart_trace = 1
        if modeDriveLane == STOP or CAN_flagStart_trace == False or \
            recv['stopSafeArea'] == True or recv['cloudStartTask'] == False:
            self.modeDrive = STOP
        else:
            if self.curDirDrive == BACK:
                self.modeDrive = modeDriveSafe # 后退无避障功能
            else:
                self.modeDrive = modeDriveSafe #
        # self.estimateDistErr(recv)
#        print 'indClosest, indPre',indClosest, indPre

    def replyCloudServer(self):
        if recv['cloudTaskID'] > self.cloudTaskID:
            # 说明有新任务
            # 响应云端新任务，重置所有相关标志位和地图

        # 更新任务结束标志位
        if self.flagLaneFinish == 1:
            self.toCloudEndTaskFlag = True
        # 更新发送到云端的开始任务标志位
        if self.toCloudEndTaskFlag == True:
            self.toCloudStartedFlag = False # 复位任务开始标志位
        else:
            if self.modeDrive == NORMAL:
                self.toCloudStartedFlag = True

        self.cloudTaskID = recv['cloudTaskID']

    # 检测正在起步
    def checkStart(self,recv):
        position = recv
        point_self = np.asarray([position['x'],position['y']])
        if np.abs(position['x']) < 0.1 and np.abs(position['y']) < 0.1:
            pass
        else:
            if self.flagLogPointStart == True:
                self.pointStart = np.array([position['x'],position['y']])
                self.flagLogPointStart = False
                self.flagChangesPre = True

            distStart  = np.linalg.norm(point_self-self.pointStart)
            # 起步过渡过程结束
            if distStart > 7.0:
                self.flagChangesPre = False

    # 计算实时偏差，采用点到直线距离
    # 先排除初始化过程，未接收到定位数据的情况
    def estimateDistErr(self,recv):
        position = recv
        if np.abs(position['x']) < 0.1 and np.abs(position['y']) < 0.1:
            # is initializing
            distErr = 0.
        else:
            startPoint,endPoint = self.mapEndPoint[self.countLaneFinish][0],self.mapEndPoint[self.countLaneFinish][1]
            polyCoeff = np.polyfit([startPoint[0],endPoint[0]],[startPoint[1],endPoint[1]],1)
            A_Coeff,C_Coeff = polyCoeff
            B_Coeff = -1.
            distErr = (A_Coeff*position['x']+B_Coeff*position['y']+C_Coeff) / np.sqrt(A_Coeff**2.+B_Coeff**2.)
            print('distErr',distErr)
            GLBlogDistErr.append(distErr)
        self.distYClose = distErr

    def changeDirection(self,recv):
        CAN_flagIsShifting = recv['is_shifting']
        shift_stat = recv['shift']
        position = recv
        modeDriveLane = NORMAL
        point_self = np.asarray([position['x'],position['y']])
        point_mapEnd = np.asarray([self.GPSmap[-1,0],self.GPSmap[-1,1]])
        distToEnd = np.linalg.norm(point_self-point_mapEnd)
        # 圆弧路径规划，不断延长地图
        if taskTypeGLB == 'csdCircle':
            if self.indLastPre >= self.GPSmap.shape[0]-2:
                # 延长地图
                self.indexCircleMap += 1
                if self.indexCircleMap > 3:
                    self.indexCircleMap = 3
                else:
                    GPS_all = np.load(os.path.join(cur_path,''.join(['../map/curveMap', str(self.indexCircleMap), '.npy'])))
                    # self.GPSmap = np.concatenate((self.GPSmap[self.indLastClosest:-1], GPS_all), axis=0)
                    self.GPSmap = np.concatenate((self.GPSmap, GPS_all), axis=0)
        # 检测正在起步
        # self.checkStart(recv)
#        if distToEnd < CALIBRT['distIsEnd']:
#            rospy.logwarn("have reach the path end, please stop")
        distIsEnd = CALIBRT['distIsEnd']
        if self.curDirDrive == BACK:
            distIsEnd = max(1., (CALIBRT['distIsEnd'] - 3.4))
        if distToEnd < distIsEnd and self.flagIsShifting == 0:
            if taskTypeGLB == 'csdCircle':
                self.flagLaneFinish = 1
            else:
                # self.flagIsShifting == 0是为了避免换挡过程地图未切换时一直满足CALIBRT['distIsEnd']
                self.countSingleLane += 1
                if self.countSingleLane >= timesSingleLaneGLB: #指示几趟
                    self.countSingleLane = 0
                    self.countLaneFinish += 1
                    self.newStraightPathFlag = True

                if self.countLaneFinish >= numAllLaneGLB:
                    self.flagLaneFinish = 1
                    self.countLaneFinish = numAllLaneGLB-1 # -1的目的：避免后面程序索引溢出

                if self.flagLaneFinish == 0: # 走完全程停车，则不必变换方向和地图
                    self.flagIsShifting = 1 # start shift
                    # 变换期望的换挡方向
                    if self.desireDirDrive == FORWARD:
                        self.desireDirDrive = BACK
                    else:
                        self.desireDirDrive = FORWARD
        #判断是否正在换挡
        if self.flagIsShifting == 1:
            # # 加self.flagIsShifting == 1是为了避免完全由底层控制，但终点时，无法进入这段程序
            flagShiftDone = False  # init
            if vehicle_model.name == 'A60':
                if (self.desireDirDrive == BACK and shift_stat == shift_R) \
                   or (self.desireDirDrive == FORWARD and shift_stat == shift_D) : # 表明换挡完成
                   flagShiftDone = True
            else:
                if CAN_flagIsShifting >= 1: # 0 1; '>=' is for uncertain float
                #     #第一次判断，认为底层响应了换挡信号
                    self.flagMCUIsShifting = 1
                if self.flagMCUIsShifting==1 and CAN_flagIsShifting==False:  # should: True-->0
                    flagShiftDone = True
            if flagShiftDone == True:
                self.flagIsShifting = 0
                self.flagMCUIsShifting = 0
                # 表明换挡完成
                self.flagLogPointStart = True

                # self.mapEndPoint同时控制车道和方向。
                # 车道由第一个索引控制
                # 第二个索引 0 1互换，则表示起点终点互换，方向切换
                if self.flagLaneFinish == 0: # 走完全程停车，则不必变换方向和地图
                    self.indLastClosest = 0  # 换地图前重新初始化
                    self.curDirDrive = self.desireDirDrive
                    if not mapTypeGLB == 'raw':
                        if self.curDirDrive == FORWARD:
                            self.GPSmap = genMapPoint(self.mapEndPoint[self.countLaneFinish][0],self.mapEndPoint[self.countLaneFinish][1],self.numGPSallPoint)
                        elif self.curDirDrive == BACK:
                            self.GPSmap = genMapPoint(self.mapEndPoint[self.countLaneFinish][1],self.mapEndPoint[self.countLaneFinish][0],self.numGPSallPoint)
                    # if taskTypeGLB == 'csdLine':
                    #     if self.newStraightPathFlag == True:
                    #         self.newStraightPathFlag = False
                    #         # 合并换道路径
                    #         lineMapStartPoint = self.GPSmap[0,0:2]
                    #         lineMapEndPoint = self.GPSmap[-1,0:2]
                    #         lineMap_k = (lineMapEndPoint[1]-lineMapStartPoint[1]) / (lineMapEndPoint[0]-lineMapStartPoint[0])
                    #         lineMap_b = lineMapStartPoint[1]-lineMap_k*lineMapStartPoint[0]
                    #         polyCoeff = np.array([lineMap_k, lineMap_b])
                    #         wayPointChangeLane = commonFcn.changeLanePlan(polyCoeff, position,lineMapStartPoint,lineMapEndPoint) # [x..., y...]， 全局路点 lenBezier=3
                    #         # 找到换道路径的末点,通过其x搜索此点在地图中的索引值,前提条件：地图x坐标随索引单调性变化
                    #         indexChangeLaneEnd = bisect.bisect_right(self.GPSmap[:,0], wayPointChangeLane[0,-1])
                    #         wayPointChangeLane = np.column_stack((wayPointChangeLane[0],wayPointChangeLane[1]))
                    #         self.GPSmap = np.concatenate((wayPointChangeLane,self.GPSmap[indexChangeLaneEnd:-1]), axis=0)
        # 记录跟踪实际坐标点
        if self.flagLogPosition == 0:   # 到终点记录一次并保存后，便不再记录
            if self.curDirDrive == BACK:
                self.flagIsLogging = True
                self.logPosition.append(point_self)
            # 保存记录的跟踪坐标点，并清空为再次记录做准备
            if (self.curDirDrive == FORWARD or self.flagLaneFinish == 1) and self.flagIsLogging == True:  # 逻辑是刚走完BACK，马上切换到FORWARD或是终点
                # 此逻辑不足之处在于，必须走完当前路线，才能保存数据
                self.flagIsLogging = False
                self.logPositionAll.append(self.logPosition)
                self.logPosition = []

        # 停车模式判断
        if (self.flagLaneFinish==1) | (self.flagIsShifting==1): # self.flagLaneFinish只有再次初始化才能清除，意味着车到这只有停车模式
            modeDriveLane = STOP
            self.countChangesPre = 0
            print 'tttttttt',self.flagLaneFinish
        # 用于驾校测试，消除停车积分
        if vehicle_model.name == 'A60':
            if shift_stat == shift_P or shift_stat == shift_N:
               modeDriveLane = STOP
        # 保存实际走过的位置点
        # 全程走完时，计算所有线路回程的跟踪误差。去程因存在规划偏移，故无意义
        if self.flagLaneFinish == 1 and self.flagLogPosition == 0:
            self.flagLogPosition = 1
            np.save(os.path.join(cur_path,'../log/logPositionAll.npy'),self.logPositionAll)

        return modeDriveLane

    def checkObstacle(self,pathCandidate):
        #################################################################
        #通过寻找第一个弧线上第一个栅格为0（障碍物）来确定无障碍弧长
        #弧长越长越好
        #################################################################
        if self.curDirDrive == FORWARD:
            OccupancyValueMat = recv['OccupancyValueForward']
        else:
            OccupancyValueMat = recv['OccupancyValueBackward']
        #障碍物检测横向区域，左右各widthMargin*0.1m 。 校园大圈：11；驾校：22;工程车23,左右总宽4.6m,振动碾对角宽6.6m
        widthMargin = int(CALIBRT['widthObstDetect']/0.1)
        limitNumOccupied = 1 #认定为不可通行时占据栅格数目限值

        x_S = pathCandidate[0]
        y_S = pathCandidate[1]
        indRow = np.minimum(299,np.round(np.array(x_S)/0.1).astype(np.int32))  # 校园大圈：284； 驾校:290
        indRow = np.maximum(0,indRow)
        indRowMat = np.tile(indRow,(2*widthMargin+1,1)).transpose()
        indCol = np.minimum(200,100+np.round(np.array(y_S)/0.1).astype(np.int32)) # 负的在右边
        indColTemp = np.arange(-widthMargin,widthMargin+1,1)
        indColTemp1 =  np.tile(indColTemp,(indCol.shape[0],1))
        indColMatTemp = indColTemp1+indCol.reshape(-1,1)
        indColMat = np.minimum(200,indColMatTemp)
        indColMat = np.maximum(0,indColMat)
        awardOccupancyTemp = OccupancyValueMat[indRowMat,indColMat] # x_S行，2*widthMargin+1列
        checkFree = np.where(awardOccupancyTemp>80) # > 60表示有障碍物
        occupiedRowTemp = np.array(checkFree)[0]
        occupiedRow = np.unique(occupiedRowTemp) # 删除数组中由不同列导致的相同行号
        numOccupiedRow = occupiedRow.size
        closedObstaclDist = 1000.  # 初始值无穷大，表示无障碍物
        if numOccupiedRow <= limitNumOccupied:   #无障碍

            awardOccupancy = 1
        else:
#            awardOccupancy = checkFree[0][0]  # 循环行数，可以抽象等价为路径长度
            awardOccupancy = 0
            '''
            障碍物距离计算
            note: awardOccupancyTemp是OccupancyValueMat的子集, 所以不能直接通过它索引x轴距离。
            需按如下方法：
                首先通过occupiedRow找到障碍物在awardOccupancyTemp中的行数，然后根据这个
                行数，映射到indRowMat矩阵的真实物理意义行数。
            '''
            if self.curDirDrive == BACK:
                distOffset = DIST_OBST_OFFSET_BACK
            else:
                distOffset = DIST_OBST_OFFSET_FORWARD
            closedObstaclDist = (indRowMat[np.min(occupiedRow),0])*0.1 - distOffset
        return awardOccupancy,closedObstaclDist

    # @timeit
    def genCandidate(self,d0,df,wayPointLocal):
        '''
        generate local path candidate
        Coordinate:
            vehicle local coordinate
        Parameter:
            wayPointLocal: wayPoint in local coordinate, two row

        '''
        if not taskTypeGLB == 'campus':
            pathCandidate = commonFcn.bezierBased(df, wayPointLocal)
        else:
            pathCandidate = commonFcn.laneBased(d0,df,wayPointLocal)
        # 障碍物检测，输入车辆坐标系序列点
        awardOccupancy,closedObstaclDist = self.checkObstacle(pathCandidate)
        result = [pathCandidate,awardOccupancy,closedObstaclDist]

        return result

    # @timeit
    def localPlanning(self,indClosest,indPre,position):
        global GLBlogDistErr
        global CALIBRT, countCALIBRTGLB
        global CenterGlobalEndGLB
        flagCandidateAllDie = 0
        safeDist2RightEdge = 0.5
        modeDrive = NORMAL
        xCenterGlobal = np.array(self.GPSmap[indClosest:indPre+1,0])
        yCenterGlobal = np.array(self.GPSmap[indClosest:indPre+1,1])
        # 路径终点，采用之前的路点，避免各种拟合函数错误
        distCenterPathVect = np.array([self.GPSmap[indPre,0:2] - \
                                        self.GPSmap[indClosest,0:2]])
        distCenterPath = np.linalg.norm(np.array([distCenterPathVect]))
        # if distCenterPath < max(5., CALIBRT['distIsEnd']):
        #     xCenterGlobal, yCenterGlobal = CenterGlobalEndGLB
        # else:
        #     CenterGlobalEndGLB = np.array([xCenterGlobal, yCenterGlobal])
        # 抽样中心线
        # numSample = xCenterGlobal.size
        # indexSample = np.arange(0,numSample,int(numSample/100)+1)
        # xCenterGlobal = xCenterGlobal[indexSample]
        # yCenterGlobal = yCenterGlobal[indexSample]
        # bezier平滑中心线，结果发现，转弯处偏离过多
        # wayPoint = np.array([xCenterGlobal,yCenterGlobal])
        # xCenterGlobal, yCenterGlobal = commonFcn.bezierSmooth(wayPoint)

        # spline平滑中心线(1)或单纯增加道路点(0)，从而可以使全局地图可以更稀疏，加快最近点搜索速度
        # 经验证此种增加数据点的方法失效，另寻差值方法
        # numGenPathPoint = max(100, int(CALIBRT['distPreview'] / 0.2))
        # spl = UnivariateSpline(xCenterGlobal,yCenterGlobal)
        # spl.set_smoothing_factor(0.)
        # xCenterGlobal = np.linspace(xCenterGlobal[0],xCenterGlobal[-1],numGenPathPoint)
        # yCenterGlobal = spl(xCenterGlobal)
        # 估计当前位置横向偏差
        # 原理：在最近点处，利用点斜式构造切线方程，然后用点到直线方程求距离
        xClosest = self.GPSmap[indClosest,0]
        yClosest = self.GPSmap[indClosest,1]
        polyCoeffCenter = np.polyfit(xCenterGlobal,yCenterGlobal,2)
        polyder1CoeffCenter = np.polyder(polyCoeffCenter) #导函数多项式系数
        polyder1Center = np.polyval(polyder1CoeffCenter,xClosest) # 斜率
        polyCoeffLine = [polyder1Center, yClosest-polyder1Center*xClosest] # A C
        point = [position['x'],position['y']]
        self.distYClose = commonFcn.calcDistPoint2Line(polyCoeffLine,point)
        # 判断当前点在中心线左右，决定初始偏差正负
        if indClosest >= self.GPSmap.shape[0]-2: # 避免索引溢出
            S  = 0
        else:
            PA = np.array([xClosest, yClosest])
            PB = np.array(self.GPSmap[indClosest+1,0:2])
            S = commonFcn.checkSidePoint2Line(PA, PB, point)
        self.distYClose *= np.sign(S)
        # 平滑中心线
        # yCenterGlobal = np.polyval(polyCoeffCenter, xCenterGlobal)
        # 先将最近的全局坐标转换为局部坐标
        pointGlobal = np.array([xCenterGlobal, yCenterGlobal])
        wayPointLocal = commonFcn.global_2_local_vect(position,pointGlobal)
        ##########################################################
        #采用map函数代替for循环，构造不同df的 path candidate
        dfMax = 2.   # 依据道路宽度3.5m设置
        dfStep = 0.1  # candidate采样距离偏差
        d0 = self.distYClose # 规划时刻当前位置偏差
        if not taskTypeGLB == 'campus':
            '''
            出山店遇到障碍物直接停车，不规划绕行
            '''
            dfMapTemp = np.array([0])  #不进行路径规划，只有一条candidate
        else:
            dfMapTemp = np.arange(-dfMax-1.,dfMax+dfStep+1.,dfStep)
        dfMap = dfMapTemp.tolist()
        numCandidate = len(dfMap)
        d0Map = [d0]*numCandidate
        wayPointLocalMap = [wayPointLocal]*numCandidate
        candidate = map(self.genCandidate, d0Map,dfMap,wayPointLocalMap) # 为一个多维list
        pathCandidate = [m for m,n,p in candidate] # 仍然是多维list
        awardOccupancy = [n for m,n,p in candidate]
        ObstaclDist = [p for m,n,p in candidate]
        closedObstaclDist = np.min(ObstaclDist)
        # 检查是否可通行
        indHalfOptimal = np.where(np.array(awardOccupancy)>0)[0] # 索引数组
        dimIndHalfOptimal = np.array(indHalfOptimal).size
        defaultCandi = int(len(dfMap)/2) # 默认直接跟踪中线
        xCtrPath = np.array([])
        yCtrPath = np.array([])
        indOptimal = defaultCandi
        print 'safe number', dimIndHalfOptimal
        if dimIndHalfOptimal == 0:
            flagCandidateAllDie = 1     # 有障碍物，不能通行
            xCtrPath = xCenterGlobal
            yCtrPath = yCenterGlobal
        else:
            # 先找出最右的path，作为右边界。也就是最后一位索引path-->dfSafe[-1]
            dfSafe = dfMapTemp[indHalfOptimal] # df_map是list不能用此方法索引
            # 然后找到距离右边界安全距离的path
            #左手坐标系
            # dfCandi = dfSafe[-1]-1.
            # indOptimalTemp = bisect.bisect_left(dfSafe,dfCandi) # numpy.searchsorted()
            # 右手坐标系
            # idea1
            dfCandi = dfSafe[0]+1.
            indOptimalTemp = bisect.bisect_right(dfSafe,dfCandi) # numpy.searchsorted()
#            indOptimalTemp = np.argmin(costAll) #
            # idea2
            indOptimalTemp = int(len(indHalfOptimal)/2)
            indOptimal = indHalfOptimal[indOptimalTemp] #通过上一步索引，反推剩下一半，也即是全局索引
#            indOptimal = defaultCandi  # 直接跟踪中线，只作用于显示
            pointOptimal = pathCandidate[indOptimal]
            xOptimalGlb, yOptimalGlb = commonFcn.local_2_global_vect(position,pointOptimal)
            # select path to control module, according to option that choose local planning or not.
            if vehicle_model.locPlan == True:
                # use local planning path
                xCtrPath = xOptimalGlb
                yCtrPath = yOptimalGlb
            else:
                # use map directly
                xCtrPath = xCenterGlobal
                yCtrPath = yCenterGlobal
        # 模式判断
        # 首先处理大曲率转弯时，缩短预瞄距离
        distPreviewCurv = CALIBRT['distPreview']
        distPreviewReplan = CALIBRT['distPreview']
#        if recv['flagBigCurv'] == True:
#            distPreviewCurv = 9.
        if flagCandidateAllDie == 1:
            if closedObstaclDist < CALIBRT['distObstDetect']:
                print("closedObstaclDist is ", closedObstaclDist, ' m')
    #            rospy.logwarn("due to obstacle, there is no path, please stop")
                modeDrive = STOP_SAFE
            else:
                # 减速，并缩短规划路径，重规划
                # modeDrive = DECELERATE
                modeDrive = NORMAL # CSD
                countCALIBRTGLB += 5
                distPlanLag = -1. # 规划周期内行驶距离
                distPreviewReplan = CALIBRT['distObstDetect'] + distPlanLag
        else:
            # 恢复预瞄距离
            if countCALIBRTGLB > 0:
                countCALIBRTGLB -= 1
                if countCALIBRTGLB == 0:
                    CALIBRT = rospy.get_param('~CALIBRT')
            modeDrive = NORMAL
        CALIBRT['distPreview'] = min(distPreviewCurv, distPreviewReplan, CALIBRT['distPreview'])
        pathCandidate = np.array(pathCandidate)
        result = {'xCtrPath':xCtrPath, 'yCtrPath':yCtrPath, \
                    'xGlbPath':xCenterGlobal, 'yGlbPath':yCenterGlobal, \
                    'pathCandidate':pathCandidate, 'indOptimal':indOptimal, \
                    'modeDrive':modeDrive, 'distObst':closedObstaclDist}
        return result

def getOccupancyGridForward(data):
    recv['OccupancyValueForward'] = np.array(data.data).reshape(data.info.height,data.info.width).T

def getOccupancyGridBackward(data):
    recv['OccupancyValueBackward'] = np.array(data.data).reshape(data.info.height,data.info.width).T

def getCAN(data):
    recv['is_shifting'] = data.is_shifting
    recv['start_trace'] = data.start_trace

def getVehState(data):
    global flagPositionReceivedGLB
    flagPositionReceivedGLB = True
    recv['x'] = data.x
    recv['y'] = data.y
    recv['z'] = data.z
    recv['roll'] = data.roll
    recv['pitch'] = data.pitch
    recv['azimuth'] = data.azimuth

    recv['speed'] = data.speed
    recv['brake_stat'] = data.brake_stat
    recv['shift'] = data.shift

def getSteerAngle(data):
    recv['curvature'] = data.curvature
    recv['flagBigCurv'] = data.flagBigCurv

def getSafeAreaWarn(data):
    recv['stopSafeArea'] = data.stopSafeArea

def getCloudTask(data):
    recv['cloudTaskID'] = data.taskID
    recv['cloudStartTask'] = data.startTask
    recv['cloudPathType'] = data.VPathType
    # recv['cloudFlagPoint'] = data.taskID


if __name__ == '__main__':
    rospy.init_node('local_planning', anonymous = False)
    recv = {'x':0., 'y':0., 'z':0., 'roll':0., 'pitch':0., 'azimuth':0., \
            'speed':0., 'brake_stat':0, 'shift':shift_P, \
            'is_shifting':False, 'start_trace':START_TRACE_INIT, \
            'OccupancyValueForward':np.zeros([300,201]), \
            'OccupancyValueBackward':np.zeros([300,201]), \
            'curvature':0.000001, 'flagBigCurv':False, 'stopSafeArea':False, \
            'cloudTaskID':0, 'cloudStartTask':False, 'cloudPathType':0}
    # Loading Parameters
    # Constant Parameters
    param_file = cur_path + '/../params/param.yaml'
    with open(param_file,'r') as f:
        param = yaml.load(f)
    mapTypeGLB = param["map"]["mapType"]
    taskTypeGLB = param["map"]["taskType"]
    if mapTypeGLB == 'raw' or taskTypeGLB == 'csdCircle':
        # 校园正常测试，而不是模拟出山店
        numAllLaneGLB = 1
        distBetweenLaneGLB = 1.  # 可随意设置，不再用到
        timesSingleLaneGLB = 1
    else:
        numAllLaneGLB = param["map"]["numAllLane"]
        distBetweenLaneGLB = param["map"]["distBetweenLane"]
        timesSingleLaneGLB = param["map"]["timesSingleLane"]
    # initialize Calibrated Parameters
    rospy.set_param('~GLBflagOpenCalib', 0) # 控制标定量接收程序,初始化将yaml中的强制覆盖
#    CALIBRT = rospy.get_param('~CALIBRT')
    param_file = cur_path + '/../params/calibParam.yaml'
    with open(param_file,'r') as f:
        param = yaml.load(f)
    CALIBRT = param["local_planning"]["CALIBRT"]

    rospy.Subscriber("vehState", VehState, getVehState)
    rospy.Subscriber("/forward/final_grid", OccupancyGrid, getOccupancyGridForward)
    rospy.Subscriber("/backward/final_grid", OccupancyGrid, getOccupancyGridBackward)
    rospy.Subscriber("/mcu_flag", mcuFlag, getCAN)
    rospy.Subscriber('steerAngleCmd', stm32TX, getSteerAngle)
    rospy.Subscriber('safeArea', PlanOP, getSafeAreaWarn)
    rospy.Subscriber('cloudChatter',cloudTask, getCloudTask)

    basePathPub = rospy.Publisher('basePath', PathDisplay, queue_size =1)
    planPub = rospy.Publisher('planOutput', PlanOP, queue_size =5)

    state = Planning()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        state.calibrate()
        state.update()
        state.publish()
        rate.sleep()
    np.save(os.path.join(cur_path,'../log/logDistErr.npy'),GLBlogDistErr)
    rospy.spin()    #rospy.spin()作用是当节点停止时让python程序退出，和C++ spin的作用不同
#    sys.exit()
