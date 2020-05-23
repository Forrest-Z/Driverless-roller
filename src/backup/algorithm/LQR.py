# -*- coding: utf-8 -*-
"""

Path tracking simulation with LQR steering control and PID speed control.

author Atsushi Sakai (@Atsushi_twi)

need vehicle_model add "vel" variable (i.e., the current speed [m/s])
by qidong
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import vehicle_model
# from pycubicspline import pycubicspline
# from matplotrecorder import matplotrecorder
import scipy.linalg as la

Kp = 1.0  # speed proportional gain

# LQR parameter
Q = np.eye(4)
R = np.eye(1)

# matplotrecorder.donothing = True


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def local_2_global_vect(position,point):
    '''
    局部坐标点转换为全局坐标,适用于一系列点向量输入,参考display.py
    input:
        position: 当前位置
        point: 需要转换的局部坐标点
    '''
    p1 = np.array(point)
    thetaAtrans = (position['azimuth'])*np.pi/180.  #逆时针弧度，而航向角是顺时针
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
    xCenterGlobal, yCenterGlobal = point
    p1 = np.array([xCenterGlobal-position['x'],yCenterGlobal-position['y']])
    thetaAtrans = (360.-position['azimuth'])*np.pi/180.  #逆时针弧度，而航向角是顺时针
    Atrans = np.array([[np.cos(thetaAtrans),-np.sin(thetaAtrans)],[np.sin(thetaAtrans),np.cos(thetaAtrans)]])
    p2 = np.dot(Atrans,p1) #
    point_local = p2

    return point_local

def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    P = Q   # X是一般文献中的P矩阵
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Pn = A.T * P * A - A.T * P* B * \
            la.inv(R + B.T * P * B) * B.T * P * A + Q
        if (abs(Pn - P)).max() < eps:
            P = Pn
            break
        P = Pn

    return P


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    P = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = np.matrix(la.inv(B.T * P * B + R) * (B.T * P * A))

    # compute other gains according to MPC book
    Ku = np.matrix(la.inv(B.T * P * B + R) * R)[0,0]
    # Kv = np.matrix(la.inv(B.T * P * B + R) * B.T
    # vk = (A - B * K).T * vk_1 - K.T * R * ff

    eigVals, eigVecs = la.eig(A - B * K)

    return K, Ku


def lqr_error_control(e, pe, th_e, pth_e, curvature, feedF):
    '''
    input:
        e: position error
        pre: previous position error
        th_e: azimuth error
        pth_e: previous azimuth error
    output:
        delta: front wheel steer angle
    '''
    # LQR parameter
    Q = np.eye(4)
    Q[0,0] = 0.1
    Q[1,1] = 0
    Q[2,2] = 0
    Q[3,3] = 0
    R = np.eye(1)*50.

    v = vehicle_model.vel
    gainDelta2Steer = vehicle_model.Delta2Steer # 前轮转角到方向盘转角的增益
    th_e = pi_2_pi(th_e)

    A = np.matrix(np.zeros((4, 4)))
    A[0, 0] = 1.0
    A[0, 1] = vehicle_model.dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = vehicle_model.dt
    # print(A)

    B = np.matrix(np.zeros((4, 1)))
    B[3, 0] = v / vehicle_model.L

    K, Ku = dlqr(A, B, Q, R)

    x = np.matrix(np.zeros((4, 1)))

    x[0, 0] = e
    x[1, 0] = (e - pe) / vehicle_model.dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / vehicle_model.dt

#    ff = math.atan2(vehicle_model.L * curvature, 1) # 前馈控制量
    ff = feedF
#    fb = pi_2_pi((-K * x)[0, 0])        # 反馈控制量
    fb = -(-K * x)[0, 0]       # 反馈控制量

    delta = ff + fb

    steerAngle = delta*gainDelta2Steer*180./np.pi
    return steerAngle

def lqr_state_control(positon,PrePointReal,PrePoint,curvature,errTheta,feedF):
    '''
    ref: MPC book p.71
    主要坐标系：
        全局坐标系,在全局地图中进行跟踪
    input:
        errTheta: 航向角偏差 [rad]
    '''
    # LQR parameter
    Q = np.array([[1,0,0],[0,1,0],[0,0,10]])
    R = np.array([100])

    Ts = vehicle_model.dt
    v = vehicle_model.vel
    L = vehicle_model.L
    gainDelta2Steer = vehicle_model.Delta2Steer # 前轮转角到方向盘转角的增益
#    delta = np.arctan(curvature*L) # 前馈控制量
    delta = feedF
    # 求解全局点和绝对航向
    preRealGlobal = local_2_global_point(positon,PrePointReal)
    preGlobal = local_2_global_point(positon,PrePoint)
    preAzimuth = positon['azimuth']*np.pi/180. + errTheta #顺时针
    # 构造状态变量
    errPos = preRealGlobal - preGlobal
    state = np.matrix([errPos[0],errPos[1],errTheta]).T
    # 系统矩阵 A B
    A = np.matrix([[1.,0,-v*np.sin(preAzimuth)*Ts],[0,1,v*np.cos(preAzimuth)*Ts],[0,0,1]])
    B = np.matrix([0,0,v*Ts/(L*np.cos(delta)**2)]).T

    K, Ku = dlqr(A, B, Q, R)

    ff = delta # 前馈控制量
    fb = -(K * state)[0,0] - Ku * ff     # 反馈控制量

    delta = ff + fb

    steerAngle = delta*gainDelta2Steer*180./np.pi

    return steerAngle

if __name__ == '__main__':
    pass
