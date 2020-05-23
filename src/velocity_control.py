#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
vehicle volocity control node
遗留问题：
    1 过减速带等坎子，可能是积分过大，导致前轮过去后，会突然加速一下，上坡时更为明显，会导致较大超调
    2 后退时，加速参数过大，导致超调极大。需采用一套不同的参数
    3 下坡时，若期望车速为0, 制动过猛
author: qidong
'''
import rospy
import sys
import os
import time
import numpy as np
import yaml
from campus_driving.msg import stm32TX
from campus_driving.msg import INSPVAX
from campus_driving.msg import PlanOP,VehState
from aeb_pub.msg import paramVehicle
import vehicle_model
import commonFcn

MODE_BRAKE = 0
MODE_ACC = 1
MODE_ENG_BRAKE = 2

FORWARD = vehicle_model.FORWARD
BACK = vehicle_model.BACK
INIT = vehicle_model.INIT
NORMAL = vehicle_model.NORMAL
STOP = vehicle_model.STOP
STOP_SAFE = vehicle_model.STOP_SAFE
DECELERATE = vehicle_model.DECELERATE
shift_P = vehicle_model.shift_P
shift_R = vehicle_model.shift_R
shift_N = vehicle_model.shift_N
shift_D = vehicle_model.shift_D
MAX_DA = vehicle_model.MAX_DA   # 踏板电压高
MIN_DA = vehicle_model.MIN_DA      # 踏板电压低 0.333
MAX_IAI_S = vehicle_model.MAX_IAI_S  # IAI行程终点 79
MIN_IAI_S = vehicle_model.MIN_IAI_S   # IAI行程起点 16-1
MAX_IAI_V = vehicle_model.MAX_IAI_V
MIN_IAI_V = vehicle_model.MIN_IAI_V
GLBlog = []
cur_path = os.path.dirname(__file__)

AxErrPlotGLB = 0.  # temp, for display
daDeqGLB = commonFcn.meanFilter(5)
iaiSDeqGLB = commonFcn.meanFilter(5)
countEstimateVel = 0
lastPosition = np.array([0.,0.])
class SpeedControl(object):
    def __init__(self):
        self.integral_vel = 0.
        self.integral_ACC = 0.
        self.integral_BK = 0.
        self.filter_ACC = 0.
        self.filter_BK = 0.

        self.TrqCmd = 0.
        self.TrqCmdStop = 0.
        self.iai_s_pre = MIN_IAI_S
        self.modeLast = MODE_ACC
        self.flagStopPlus = False

        self.pedalCmd = [] # output initialization
        self.curMeanVel = 0. # 采用高精度GPS估计车速
        self.errEstimateVel = 0.
        self.velDesire = 0.

    def calibrate(self):
        # Calibrated Parameters
        global CALIBRT
        flagOpenCalib = rospy.get_param('~GLBflagOpenCalib') # 控制标定量接收程序
        if flagOpenCalib == 1:
            CALIBRT = rospy.get_param('~CALIBRTVEL')
            rospy.set_param('~GLBflagOpenCalib', 0)

    def publish(self):
        msg = stm32TX()
        msg.da = self.pedalCmd[0]
        if vehicle_model.VelCtrEnable == True:
            msg.iai_s = float(self.pedalCmd[1])
        else:
            msg.iai_s = MIN_IAI_S
        msg.iai_v = float(self.pedalCmd[2])
        msg.steer_angle = AxErrPlotGLB  # 临时发布的伪消息，用于调试时观察趋势
        msg.velDesire = self.velDesire
        spdCtrPub.publish(msg)

    def update(self):
        global GLBlog
        global AxErrPlotGLB

        MAX_AX = 1
        MIN_AX = -2.5
        OFFSET_IAI_S = 6.  # IAI行程起点偏移量，因为MIN_IAI_S过于保守，另外制动踏板本身也存在死区
        K_v2a = 0.36  # 按照起步平稳加速到10加速度 == 舒适加速度设置
        Ts = 0.1 #0.1
        Kp_ACC = 0.3  # 纯 P ，得到的da为0.58左右设置
        Kp_BK = 0.15
        Ki_ACC = 0.15
        Ki_BK = 0.4
        Kd_ACC = 0.0
        Kd_BK = 0.0
        filterN = 100
        K_pitch = 0.3  # 0.37 = MIN_AX/minPitch

        velActual = recv['speed']
        SW_brake_stat = recv['brake_stat']
        PID_shift_stat = recv['shift']
        pitch = recv['pitch']
        directionDrive = recv['curDirDrive']
        AxActual = recv['a_x']
        aExpectAEB = recv['aExpectAEB']
        modeDrive = recv['modeDrive']
        velDesire = self.selectVelDesire()
        self.velDesire = velDesire
        velErr = (velDesire - velActual)/3.6
        # 避免平衡点震荡
        if np.abs(velErr) < 1/3.6:
            velErr = 0.
        self.integral_vel += velErr
        if vehicle_model.AEBenable == True:
            AxDesire = aExpectAEB
        else:
            if modeDrive == DECELERATE:
#                AxDesire = MIN_AX * 0.5
                AxDesire = velErr * K_v2a
            else:
                AxDesire = velErr * K_v2a
        # print 'velDesire, velActual, AxDesire', velDesire, velActual, AxDesire
        # 坡道补偿前馈
        # 暂时只针对下坡制动工况，避免模式切换混乱
        if AxDesire < 0. and pitch < -1.2:  # -1.5
            AxDesire = AxDesire + pitch * K_pitch
        # print 'pitch',pitch
        # print 'AxDesire, Axpitch', AxDesire, - pitch * K_pitch
        if vehicle_model.AEBenable == False:
            AxDesire = commonFcn.saturation(MIN_AX, MAX_AX, AxDesire)
        AxErr = AxDesire - AxActual
        # 避免停车时意外积分,需观测停车数值并标定
        if np.abs(AxErr) < 0.001:
            AxErr = 0.
        # 暂不确定是否需要限制，或适当增大限值
        # 可先rqt_plot观测其变化
        # 因为偏差若超过限值，则必然是在不同模式切换中。所以若限制，则存在一个假设前提：
        # 从反向加速度为0开始，进行加速度跟踪
        AxErrPlotGLB = AxErr
        if vehicle_model.AEBenable == False:
            AxErr = commonFcn.saturation(MIN_AX, MAX_AX, AxErr)

        #######################################################################
        # 模式切换
        if self.modeLast == MODE_BRAKE:
            if AxDesire > 0.1:
                mode = MODE_ACC
            else:
                mode = MODE_BRAKE
        else:
            if AxDesire < -0.3:  # 还是需要下坡前kui
                mode = MODE_BRAKE
            elif AxDesire < -0.1:
                # 发动机制动模式，设置此模式原因：
                # 只有制动模式的话，因减速度设限过大，导致难以退出ACC，进而使得积分难以消除，
                # 导致速度不断增大，超调严重，偶尔起步时尤为明显
                # 阈值不设置到0的原因：轻微减速可以通过减少油门实现，此时不能清除积分，否则震荡
                mode = MODE_ENG_BRAKE
            else:
                mode = MODE_ACC
        if SW_brake_stat > 0. or (modeDrive == STOP or modeDrive == STOP_SAFE):
            # 解决响应停车指令时，总是无法停稳的问题
            mode = MODE_BRAKE
        self.modeLast = mode
        # print 'mode', mode

        if mode==MODE_ACC:
            iai_s = MIN_IAI_S
            iai_v = MAX_IAI_V
            self.integral_BK = 0.
            self.filter_BK = 0.

            filterCoeff = (Kd_ACC * AxErr - self.filter_ACC) * filterN
            da = Kp_ACC * AxErr + self.integral_ACC + filterCoeff
            self.integral_ACC += Ki_ACC * AxErr * Ts
            self.filter_ACC += filterCoeff * Ts

            da = da * MAX_DA  # 0-1
            da = commonFcn.saturation(MIN_DA, MAX_DA, da)
        elif mode == MODE_ENG_BRAKE:
            da = MIN_DA
            iai_v = MAX_IAI_V
            self.integral_ACC = 0.
            self.filter_ACC = 0.
            iai_s = self.iai_s_pre
        else:
            da = MIN_DA
            iai_v = MAX_IAI_V
            self.integral_ACC = 0.
            self.filter_ACC = 0.

            # PID controller
            filterCoeff = (Kd_BK * AxErr - self.filter_BK) * filterN
            iai_s = Kp_BK * AxErr + self.integral_BK + filterCoeff
            iai_s = -iai_s * MAX_IAI_S
            # 停车时会出现快到0时，P部分为0，导致iai_s回程，然后积分后继续刹车的现象
            # 需要避免回程动作出现
            if velDesire < 0.5 and velActual < 4.:
                # 适当增加积分作用，抵消P的下降
                Ki_BK = Ki_BK * 1.35
                # # 同时尝试算法：在速度接近0时，直接不断累计,直到加速度为0
                # if velActual < 1. and AxActual > 0.05: #观察停车时a_x零飘设置
                #     iai_s = self.iai_s_pre + 5.
            self.integral_BK += Ki_BK * AxErr * Ts
            self.filter_BK += filterCoeff * Ts

            iai_s = commonFcn.saturation(MIN_IAI_S, MAX_IAI_S, iai_s)
            # 停车时限制回程
            if velDesire < 0.5 and iai_s < self.iai_s_pre:
                iai_s = self.iai_s_pre
            #阶梯式指令，避免IAI不断颤振
            if abs(iai_s-self.iai_s_pre) < 2.5:
                iai_s = self.iai_s_pre
            else:
                self.iai_s_pre = iai_s
        # 避免停车时
        # 清零，刹车窜出问题解决，但是爬坡被迫为0的时候怎么解决
        if not (PID_shift_stat == shift_D or PID_shift_stat == shift_R):
            self.integral_vel = 0.
            self.integral_ACC = 0.
            self.filter_ACC = 0.
        if SW_brake_stat > 0.:
            self.integral_ACC = 0.
            self.filter_ACC = 0.
        # 输出多次平均，避免颤振.配合控制频率增加4倍, Ts
        da = daDeqGLB.update(da)
        iai_s = iaiSDeqGLB.update(iai_s)
        # print 'da,iai_s', da, iai_s
        # 保存数据
        logVar = np.array([velDesire,velActual,AxDesire,AxActual,AxErr,da,iai_s,self.errEstimateVel])
        GLBlog.append(logVar)
        # 更新输出
        self.pedalCmd = [da,iai_s,iai_v]

    def selectVelDesire(self):
        velDesirePlan = self.velPlanning()
        velDesireAEB = recv['velDesireAEB']
        if vehicle_model.AEBenable == True:
            velDesire = velDesireAEB
        else:
            velDesire = velDesirePlan

        return velDesire

    def velPlanning(self):
        # value set ref: 孙振平P97 --> 0.08g. 0.67=0.08*1/0.12
        # 最终设置应该参考神龙园转弯限制速度到10km/h
        MAX_AY = 0.67
        MIN_AY = -MAX_AY
        modeDrive = recv['modeDrive']
        curvature = recv['curvature']
        flagBigCurv = recv['flagBigCurv']
        # desire velocity calculation
        if modeDrive == STOP or modeDrive == STOP_SAFE:
            velocityDesire = 0.
        else:
            velocityDesire = CALIBRT['manu_vel']
        # 道路曲率导致的侧向加速度限制
        # a = v**2 / R
        if flagBigCurv == False:
            velRoadLimit = 30.
        else:
            # velRoadLimit = np.sqrt(np.abs(MAX_AY/curvature))*3.6
            velRoadLimit = 30.
        velDesire = min(velocityDesire, velRoadLimit)
        return velDesire

    def estimateVel(self):
        global countEstimateVel, lastPosition
        curPosition = np.array([recv['x'],recv['y']])
        countEstimateVel += 1
        calcTime = 5
        if countEstimateVel > calcTime:
            countEstimateVel = 0
            # 0.5s计算一次.程序0.1s循环一次
            dist = np.linalg.norm(curPosition-lastPosition)
            self.curMeanVel = 3.6*dist/(calcTime*0.1)
            lastPosition = curPosition
            print 'curMeanVel', self.curMeanVel
            self.errEstimateVel = recv['speed'] - self.curMeanVel
            print 'errEstimateVel', self.errEstimateVel


def getVelDesire(data):
    recv['velDesireAEB'] = data.speedExpect
    recv['aExpectAEB'] = data.aExpect

def getModeDrive(data):
    recv['modeDrive'] = data.modeDrive

def getVehState(data):
    recv['x'] = data.x
    recv['y'] = data.y
    recv['z'] = data.z
    recv['roll'] = data.roll
    recv['pitch'] = data.pitch
    recv['azimuth'] = data.azimuth
    recv['curDirDrive'] = data.curDirDrive

    recv['speed'] = data.speed
    recv['brake_stat'] = data.brake_stat
    recv['shift'] = data.shift

    recv['w_x'] = data.w_x
    recv['w_y'] = data.w_y
    recv['w_z'] = data.w_z
    recv['a_x'] = data.a_x
    recv['a_y'] = data.a_y
    recv['a_z'] = data.a_z

def getSteerAngle(data):
    recv['curvature'] = data.curvature
    recv['flagBigCurv'] = data.flagBigCurv

if __name__ == '__main__':

    rospy.init_node('velocity_control', anonymous = False)
    recv = {'x':0., 'y':0., 'z':0., 'roll':0., 'pitch':0., 'azimuth':0., \
            'curDirDrive':FORWARD, 'speed':0., 'brake_stat':0, 'shift':shift_P, \
            'velDesireAEB':0., 'aExpectAEB':0., 'modeDrive':NORMAL, \
            'w_x':0., 'w_y':0., 'w_z':0., 'a_x':0., 'a_y':0., 'a_z':0., \
            'curvature':0.000001, 'flagBigCurv':False}
    # initialize Calibrated Parameters
    rospy.set_param('~GLBflagOpenCalib', 0) # 控制标定量接收程序,初始化将yaml中的强制覆盖
#    CALIBRT = rospy.get_param('~CALIBRTVEL')
    param_file = cur_path + '/../params/calibParam.yaml'
    with open(param_file,'r') as f:
        param = yaml.load(f)
    CALIBRT = param["velocity_control"]["CALIBRTVEL"]
    rospy.Subscriber("vehState", VehState, getVehState)
    rospy.Subscriber("aeb_cmd", paramVehicle, getVelDesire)
    rospy.Subscriber("planOutput", PlanOP, getModeDrive)
    rospy.Subscriber('steerAngleCmd', stm32TX, getSteerAngle)

    spdCtrPub = rospy.Publisher("daIAI_cmd", stm32TX, queue_size =1)

    state = SpeedControl()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        state.calibrate()
        state.update()
        state.publish()
        state.estimateVel()
        rate.sleep()
    np.save(os.path.join(cur_path,'../log/logVelCtr.npy'),GLBlog)
    rospy.spin()    #rospy.spin()作用是当节点停止时让python程序退出，和C++ spin的作用不同
#    sys.exit()
