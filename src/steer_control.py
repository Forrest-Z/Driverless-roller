#!/usr/bin/python
# -*- coding: utf-8 -*-
'''


author: qidong
'''
import rospy
import sys
import os
import yaml
import time
import numpy as np
from campus_driving.msg import PlanOP,VehState,stm32TX
import vehicle_model
import fuzzyPID
import commonFcn
from commonFcn import timeit

FORWARD = vehicle_model.FORWARD
BACK = vehicle_model.BACK

INIT = vehicle_model.INIT
NORMAL = vehicle_model.NORMAL
STOP = vehicle_model.STOP
STOP_SAFE = vehicle_model.STOP_SAFE
shift_P = vehicle_model.shift_P
shift_R = vehicle_model.shift_R
shift_N = vehicle_model.shift_N
shift_D = vehicle_model.shift_D
MAX_STEER_ANGLE = vehicle_model.MAX_STEER_ANGLE
MIN_STEER_ANGLE = vehicle_model.MIN_STEER_ANGLE
MAX_STEER_RATE = vehicle_model.MAX_STEER_RATE
MIN_STEER_RATE = vehicle_model.MIN_STEER_RATE
Ts_STEER = 0.1
etaLast = 0.
if vehicle_model.name == 'A60':
    gainA60toCSD = 1.  #
else:
    gainA60toCSD = 1.  # 2.52 = 3.5*360 / 500

GLBlogEta = []
flagNewDateGLB = False # receive new wayPoint from planning module
curvatureLastGLB = 0.
prePointLastGLB = np.array([0.,0.])
cur_path = os.path.dirname(__file__)

class SteerControl(object):
    def __init__(self):
        self.steerAngleLast = 0.
        self.steerAngleLastOld = 0.
        self.integral = 0.
        self.flagPositon = 0
        self.initPoint = np.asarray([0,0])
        self.flagClearInitIntegral = 0
        self.signCount = 0
        self.steerAnglePlus = 0.
        self.etaCount = 0
        self.steerAngle = 0. # output initialization
        self.indLastClosest = 0
        self.wayPoint = np.array([])
        self.curvature = 0.
        self.flagBigCurv = False    # 大曲率模式
        self.steerPID = commonFcn.PID(Ts_STEER,1.) # class
        self.eta = 0.

    def publish(self):
        msgSteerAngle = stm32TX()
        msgSteerAngle.steer_angle = self.steerAngle
        msgSteerAngle.curvature = self.curvature
        msgSteerAngle.flagBigCurv = self.flagBigCurv
        msgSteerAngle.eta = self.eta
        msgSteerAngle.header.stamp = rospy.Time.now()
        steerPub.publish(msgSteerAngle)

    def calibrate(self):
        # Calibrated Parameters
        global gainsForward, gainsBack
        flagOpenCalib = rospy.get_param('~GLBflagOpenCalib') # 控制标定量接收程序
        if flagOpenCalib == 1:
            gainsForward = rospy.get_param('~gainsForward') # 前进增益
            gainsBack = rospy.get_param('~gainsBack') # 后退增益
            rospy.set_param('~GLBflagOpenCalib', 0) # 控制标定量接收程序
#    @timeit
    def update(self):
        directionDrive = recv['curDirDrive']
        flagIsShifting = recv['flagIsShifting']
        modeDrive = recv['modeDrive']
        shift = recv['shift']
        #相关代码用于检测是否起步完成，从而清除起步阶段的积分
        if np.abs(recv['x']) > 0 and np.abs(recv['y']) >0 and self.flagPositon == 0:
            self.flagPositon = 1
            self.initPoint = np.asarray([recv['x'],recv['y']])
        if np.abs(self.initPoint[0]) > 0:
            curPoint = np.asarray([recv['x'],recv['y']])
            distToInit = np.linalg.norm(curPoint-self.initPoint)
            if distToInit > 1:
                self.flagClearInitIntegral = 1
            if distToInit < 1. and self.flagClearInitIntegral == 0:
                self.integral = 0.
                self.steerPID.integral = 0.
                self.steerPID.filter = 0.
        if directionDrive == FORWARD:
            steerAngle = self.steer_angle_gen(gainsForward)*gainA60toCSD
        else:
            steerAngle = -self.steer_angle_gen(gainsBack)*gainA60toCSD
        print 'pidangle',steerAngle
        # modeDrive = NORMAL
        # flagIsShifting = 0
        if modeDrive == STOP or flagIsShifting == 1 or shift == shift_N:
            # if not vehicle_model.name == 'A60':
            steerAngle = 0.  # 回正
            self.integral = 0.
            self.steerPID.integral = 0.
            self.steerPID.filter = 0.
        # 考虑回差
        if not vehicle_model.name == 'A60':
            sign1 = np.sign(steerAngle - self.steerAngleLast)
            sign2 = np.sign(self.steerAngleLast - self.steerAngleLastOld)
            if sign1==sign2:
                self.signCount += 1
            else:
                self.signCount = 0
                if self.signCount > 8:
                    if sign1 > 0:
                        self.steerAnglePlus = 30
                    else:
                        self.steerAnglePlus = -30
            steerAngle += self.steerAnglePlus
            self.steerAngleLastOld = self.steerAngleLast
            self.steerAngleLast = steerAngle
        steerAngle = commonFcn.saturation(MIN_STEER_ANGLE, MAX_STEER_ANGLE, steerAngle) #
        steerAngleDisp = steerAngle/360.  #7560=3.5*360,
        print('steerAngle',steerAngle)
#            print('steerAngleDisp',steerAngleDisp)
        self.steerAngle = steerAngle

    def steer_angle_gen(self,gains):
        algorithm = 'PID' # 'PID', 'purePursuit'
        global etaLast
        global GLBlogEta
        '''
        可调整参数pureGain， distPreview
        但distPreview有些特殊，在setCalibParam.py中的直接影响CSD，对A60无效。
        A60目前预瞄距离是采用延迟时间*速度的方式计算的，欲调整，需调整内部计算参数
        '''
        # init local variable
        etaOriginal, Lfw = self.estimateErr(gains)
        gainPlus = gains['pure'] # 尝试值，需进一步确认
        directionDrive = recv['curDirDrive']
        # # 修正偏差信息
        # if not vehicle_model.name == 'A60':
        #     if directionDrive == FORWARD:
        #         plusAngle = (gainPlus * self.steerAngleLast /25.)*np.pi/180.
        #         eta = etaOriginal +plusAngle
        #     else:
        #         eta = etaOriginal
        # else:
        #     eta = etaOriginal
        eta = etaOriginal
        self.eta = eta
        if algorithm == 'PID':
            steeringAngleGen = self.PID(gains,eta)
        # 纯跟踪控制
        elif algorithm == 'purePursuit':
            # 计算适用于纯跟踪的航向偏差
            # fuzzy PID
            delta_eta = eta - etaLast
            etaSat = commonFcn.saturation(-0.4,0.4,eta)
            delta_etaSat = commonFcn.saturation(-0.035,0.035,delta_eta)
            startTime = time.time()
            gainFuzzy = fuzzyPID.fuzzyPID(etaSat,delta_etaSat)
            endTime = time.time()
            timecost = endTime - startTime
            # steeringAngleGen = self.purePursuit(eta,Lfw,gains)*gainFuzzy
            steeringAngleGen = self.purePursuit(eta,Lfw,gains)
            # 根据纯滞后时间，进行更新, A60: 0.2s truck: 0.3s
            self.etaCount += 1
            if self.etaCount >= 4:
                self.etaCount = 0
                etaLast = eta
            logVar = np.array([eta,delta_eta])
            GLBlogEta.append(logVar)
        return steeringAngleGen

    def purePursuit(self,eta,Lfw,gains):
        '''
        input:
            eta: 纯跟踪中的航向偏差，包含了PID中位置偏差和航向偏差信息
            Lfw: 预瞄距离，斜边
        output:
            steerAngle: 方向盘转角
        delta: 前轮转角
        '''
        pureGain = gains['pure']
        Ki = gains['Ki']
        gainDelta2Steer = vehicle_model.Delta2Steer # 前轮转角到方向盘转角的增益
        delta = np.arctan(2.*vehicle_model.L*np.sin(eta)/Lfw)
        steerAngle = delta*gainDelta2Steer*180./np.pi* pureGain
        # I输出
        steerAngle += self.integral
        self.integral += Ki * eta * Ts_STEER
        # print 'self.integral', self.integral
        return steerAngle

    def PID(self,gains,eta):
        '''
        # 航向角偏差横摆角速度双闭环PID
        '''
        Kp = gains['Kp']
        Ki = gains['Ki']
        Kd = gains['Kd']
        # 求解偏差
        K_eta2r = 0.5
        yawRateActual = recv['w_z']
        yawRateDesire = eta * K_eta2r # 依据神龙园右上弯　< MAX_STEER_RATE设置
        yawRateDesire = commonFcn.saturation(MIN_STEER_RATE,MAX_STEER_RATE,yawRateDesire)
        yawRateErr = yawRateDesire - yawRateActual
        # print 'yawRateErr', yawRateErr
        # PID控制
        print 'eta', eta
        if abs(eta) > 0.2:
        #     Ki = 0.
            self.steerPID.integral = 0.
        print 'integral', self.steerPID.integral * MAX_STEER_ANGLE
        steerAngle = self.steerPID.update(eta, Kp, Ki, Kd)
        steerAngle = steerAngle * MAX_STEER_ANGLE  # 0-1
        print 'PID', steerAngle
        # print 'self.steerPID.integral', self.steerPID.integral
        return steerAngle

    def estimateErr(self, gains):
        '''
        estimate tracking error for control
        Coordinate:
            global
        Return:
            eta: tracking error [rad] (-np.pi/2., np.pi/2.)
        '''
        global flagNewDateGLB
        global prePointLastGLB
        T_lag = vehicle_model.T_lag
        speed = recv['speed']
        directionDrive = recv['curDirDrive']
        position = recv
        path = recv
        if recv['xCtrPath'].size <= 10:  # note: np.zeros([]).size = 1
            # system is initializing or no available xCtrPath
            print 'system is initializing'
            eta = 0.    # default value
            Lfw = 5.5
        else:
            # generate path when receive new wayPoint in global coordinate
            if flagNewDateGLB == True:
                flagNewDateGLB = False
                self.indLastClosest = 0
                self.wayPoint = np.column_stack((recv['xCtrPath'],recv['yCtrPath']))
            # set preview distance
            if not vehicle_model.name == 'A60':
                distPreview = self.changeDistPreview(position,prePointLastGLB,path,gains)
                distYClose = recv['distYClose']
                distPreVect = np.array([distYClose, distPreview])
                distPreview = np.linalg.norm(distPreVect)
            else:
                distPreview = T_lag * speed/3.6
#                distPreview = 10.
                # 5.56 --> 8; 13.89 --> 20
                distYClose = recv['distYClose']
                distPreview = 5.
                distPreVect = np.array([distYClose, distPreview])
                distPreview = np.linalg.norm(distPreVect)
                distPreview = commonFcn.saturation(5.56*1.,13.89,distPreview)
                # print 'distPreview', distPreview
                if directionDrive == BACK:
                    distPreview = 1. * distPreview
            # find closest and preview point
            indClosest, indPre = commonFcn.findClosestPointSteer( \
                position, self.wayPoint, self.indLastClosest, distPreview)
            self.indLastClosest = indClosest
            preX = self.wayPoint[indPre,0]
            preY = self.wayPoint[indPre,1]
            prePoint = np.array([preX,preY]) # why not to assign with wayPoint, for robust
            prePointLastGLB = prePoint
            eta = self.estimateEta(prePoint,position,gains)
            # calculate Lfw
            # Lfw = 5.5 is csd value at 12.29 test
            curPoint = np.array([position['x'], position['y']])
            Lfw = np.linalg.norm(prePoint-curPoint)
            # calculate curvature at preview point
            # self.curvature = self.calcCurvature(prePoint,path) # 0312 temp disable

        return eta, Lfw

    def estimateEta(self,prePoint,position,gains):
        '''
        estimate azimuth error by method of pure pursuit
        '''
        preXLoc, preYLoc = commonFcn.global_2_local_point(position,prePoint)
        # print 'position,prepoint', prePoint, preXLoc, preYLoc
        # 将控制点提前，eta可以尽快响应实际变化情况，避免超调
        # 分普通情形和终点
        distAnchor = gains['pure']  # [m]
        curPosition = np.array([recv['x'], recv['y']])
        distPre2Cur = np.linalg.norm(prePoint-curPosition)
        # if recv['distYClose'] >1.:
        #     distAnchor = 0.
        # print 'distAnchor', distAnchor
        if distPre2Cur > distAnchor:
            # eta = -np.arctan(preYLoc/(preXLoc-distAnchor)) # '-'确保右转，eta为正
            pointAnchor = np.array([distAnchor, 0.])
            pointAnchorGlobal = commonFcn.local_2_global_point(position,pointAnchor)
            curPosition = pointAnchorGlobal
            # curPosition = [42.,-7.]
            # prePoint = [36.9,-4.4]
            # curPosition = [2.,0.]
            # prePoint = [-1,0]
            angle = commonFcn.xAxis2VectCounter(curPosition, prePoint) #正北顺时针
            print 'angle,azimuth',angle,recv['azimuth'],curPosition
            eta = (recv['azimuth'] - angle)
            if recv['azimuth'] < 90. and angle > 350.:
                eta = recv['azimuth'] + 360. - angle
            if angle < 90. and recv['azimuth'] > 350.:
                eta = angle + 360. - recv['azimuth']
            eta = eta*np.pi/180.
            # print 'angle111',angle
        else:
            # 应该取消锚点，同时也间接意味着到终点，所以直接转向回位
            eta = 0.
        # # 考虑大于90°情况，也就是x<0
        # if preXLoc < 0 :
        #     if preYLoc < 0:
        #         # 位于YX坐标系第四象限
        #         eta = np.pi - np.abs(eta)
        #     else:
        #         eta = -(np.pi - np.abs(eta))
            # print 'xxx is negative'
#        print 'eta', eta
        etaSat = commonFcn.saturation(-np.pi/2., np.pi/2., eta)
        print 'etaSat',etaSat

        return etaSat

    def calcCurvature(self,prePoint,path):
        '''
        Parameter:
            prePoint: preview point represented in global coordinate
        '''
        global curvatureLastGLB
        xPath = path['xCtrPath']
        yPath = path['yCtrPath']
        polyCoeff = np.polyfit(xPath, yPath, 2)
        curvature = commonFcn.calcCurvature(polyCoeff,prePoint[0])
        # if curvature > 0.05:
        #     # 判断进入圆弧区，数值需要标定，越靠近
        # 对曲率大小进行评估，判断是否正在大曲率转弯
        # if self.flagBigCurv == False:
        #     if curvature > 0.02:  # 0.02由campus20180207.bag曲率图获知
        #         self.flagBigCurv = True
        #     else:
        #         pass
        # else:
        #     if curvature < curvatureLastGLB and curvature < 0.04:
        #         self.flagBigCurv = False
        #     else:
        #         pass
        # curvatureLastGLB = curvature

        return curvature

    def changeDistPreview(self,position,prePoint,path,gains):
        '''
        注：先试验csdCircle模式下，直接用统一的最短预瞄距离会否能全程满足
        默认采用正常直线预瞄距离，开始进入圆弧区时，缩短预瞄距离；退出圆弧区时，恢复预瞄距离
        如何判断进入圆弧区：
            判断与每条直线终点的距离，若<=预瞄距离，则为真
        如何判断退出圆弧区:
            圆弧标志为真的前提下，若与下一条直线起点距离小于某定值或预瞄点处曲率小于某阈值为真
        还是采用曲率比较好，因为若采用起点终点的方法，首先还得不断判断正处于那条直线的地图部分上，
        算法适用性差。曲率算法的前提是路点都是由采样生成而不是原始路点，保证曲率有较好的光滑性和单调性
        最近点搜索算法不受此影响
        '''
        distPreview = gains['distPreview']
        curvaturePre = np.abs(calcCurvature(prePoint,path))
        print 'curvaturePre', curvaturePre
        if curvaturePre > 0.05:
            # 判断进入圆弧区，数值需要标定，越靠近1/R
            distPreview = gains['distPreviewCurve']

        return distPreview


def getModeDrive(data):
    global flagNewDateGLB
    flagNewDateGLB = True
    recv['flagIsShifting'] = data.flagIsShifting
    recv['modeDrive'] = data.modeDrive
    recv['xCtrPath'] = np.array(data.xCtrPath)
    recv['yCtrPath'] = np.array(data.yCtrPath)
    recv['distYClose'] = data.distYClose

def getVehState(data):
    recv['x'] = data.x
    recv['y'] = data.y
    recv['z'] = data.z
    recv['roll'] = data.roll
    recv['pitch'] = data.pitch
    recv['azimuth'] = data.azimuth
    recv['w_x'] = data.w_x
    recv['w_y'] = data.w_y
    recv['w_z'] = data.w_z
    recv['a_x'] = data.a_x
    recv['a_y'] = data.a_y
    recv['a_z'] = data.a_z
    recv['curDirDrive'] = data.curDirDrive
    recv['speed'] = data.speed
    recv['shift'] = data.shift

if __name__ == '__main__':
    rospy.init_node('steer_control', anonymous = False)
    recv = {'x':0., 'y':0., 'z':0., 'roll':0., 'pitch':0., 'azimuth':0., \
            'xCtrPath':np.zeros([]), 'yCtrPath':np.zeros([]), 'speed':0., \
            'curDirDrive':FORWARD, 'flagIsShifting':0, 'modeDrive':NORMAL, \
            'w_x':0., 'w_y':0., 'w_z':0., 'a_x':0., 'a_y':0., 'a_z':0., \
            'shift':shift_P, 'distYClose':0.}
    # initialize Calibrated Parameters
    param_file = cur_path + '/../params/calibParam.yaml'
    rospy.set_param('~GLBflagOpenCalib', 0) # 控制标定量接收程序,初始化将yaml中的强制覆盖
#    gainsForward = rospy.get_param('~gainsForward') # 前进增益
#    gainsBack = rospy.get_param('~gainsBack') # 后退增益
    with open(param_file,'r') as f:
        param = yaml.load(f)
    gainsForward = param["steer_control"]["gainsForward"]
    gainsBack = param["steer_control"]["gainsBack"]

    rospy.Subscriber("planOutput", PlanOP, getModeDrive)
    rospy.Subscriber("vehState", VehState, getVehState)

    steerPub = rospy.Publisher('steerAngleCmd', stm32TX, queue_size =5)

    state = SteerControl()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        state.calibrate()
        state.update()
        state.publish()
        rate.sleep()
    np.save(os.path.join(cur_path,'../log/logEta_Delta.npy'),GLBlogEta)
    rospy.spin()    #rospy.spin()作用是当节点停止时让python程序退出，和C++ spin的作用不同
#    sys.exit()
