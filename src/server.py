#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
ref: https://github.com/wy36101299/socket
key feature:
    python communicate with matlab in another PC by TCP/IP
    this program is server, and it is not only communicate with matlab,
    but all client, in the same PC or not.
edit by qidong
2018.1.8
'''
import rospy
import socket
import sys
import os
import thread
import numpy as np
import re
import time
from campus_driving.msg import VehState,PlanOP,stm32TX
import vehicle_model
from commonFcn import timeit

shift_P = vehicle_model.shift_P
shift_R = vehicle_model.shift_R
shift_N = vehicle_model.shift_N
shift_D = vehicle_model.shift_D
FORWARD = vehicle_model.FORWARD
BACK = vehicle_model.BACK
HOST = ''  #不同电脑通信，不能用localhost
# PORT = int(sys.argv[1])
PORT = 21569
BUFSIZE = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'

# 錯誤處理
try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

print 'Socket bind complete'

# 最多連接10個
s.listen(10)
print 'Socket now listening'
timeStart = 0.
GLBlog = []
cur_path = os.path.dirname(__file__)
# 處理連接程序,並可創造多執行緒
@timeit
def clientthread(conn,clientaddress):
    global timeStart
    global GLBlog
    # 一直保持接收data
    while True:

        # data為client所傳入的資訊
        data = conn.recv(BUFSIZE)
        # print(clientaddress+'  '+'say:', data)
        if not data:
            break
        data = data.strip()
        # 利用正则表达式处理数值提取问题
        # ref: http://blog.csdn.net/u010412858/article/details/71747488
        ## \b 匹配一个单词的边界。
        ## \d 匹配任意数字。
        ## \D 匹配任意非数字字符。
        ## x? 匹配一个可选的 x 字符 (换言之，它匹配 1 次或者 0 次 x 字符)。
        ## x* 匹配0次或者多次 x 字符。
        ## x+ 匹配1次或者多次 x 字符。
        ## x{n,m} 匹配 x 字符，至少 n 次，至多 m 次。
        ## (a|b|c) 要么匹配 a，要么匹配 b，要么匹配 c。
        sensor = re.findall(r"\-?\d+\.?\d*",data)  # 不兼容科学计数法e
    	# 在該連結的client 回覆
        msgState = VehState()
        msgState.x = float(sensor[0])    # [m] Pointing to due north
        msgState.y = float(sensor[1])    # [m] Pointing to due east
        msgState.z = 0.    # [m] up
        msgState.roll = float(sensor[5])  # [deg]
        msgState.pitch = float(sensor[6])   # [deg] on level road is -2.717. on campus, min is -9.28
        yaw = float(sensor[7])
        if recv['curDirDrive'] == BACK:
            if yaw > 180.:
                yaw = yaw - 180.
            else:
                yaw = yaw + 180.
        msgState.azimuth = yaw    # [deg] counterclockwise from East 0-360

        msgState.w_x = float(sensor[8])   # [rad/s]  # 坐标互换
        msgState.w_y = float(sensor[9])   # [rad/s]
        msgState.w_z = float(sensor[10])   # [rad/s]
        msgState.a_x = float(sensor[3])    # [m/ss]  # 坐标互换
        msgState.a_y = float(sensor[4])   # [m/ss] # Pointing to left
        msgState.a_z = 0.   # [m/ss]

        msgState.steer_angle = 0.   # [deg] current steering angle, clockwise -540 - 540
        msgState.speed = float(sensor[2])     # [km/h] >= 0
        msgState.brake_stat = 0 # 0: default; 1:brake
        gear = int(float(sensor[11]))
        if gear == -1:
            shift = shift_R
        elif gear == 0:
            shift = shift_P
        else:
            shift = shift_D
        msgState.shift = shift # 1: P; 2: R; 3: N; 4: D
        msgState.curDirDrive = recv['curDirDrive']
        msgState.header.stamp = rospy.Time.now()
        statePub.publish(msgState)

        if recv['desireDirDrive'] == FORWARD:
            gearDesire = 0
        elif recv['desireDirDrive'] == BACK:
            gearDesire = -8
        else:
            gearDesire = -7
        steerAngle = recv['steerAngle']
        velDesire = recv['velDesire']
        eta = recv['eta']
        reply = np.array([steerAngle,velDesire,gearDesire,eta])
        conn.sendall(str(reply))  # 会传输科学计数法e
        logVar = np.array([steerAngle])
        GLBlog.append(logVar)

    # 連線中斷
    conn.close()

def getModeDrive(data):
    recv['curDirDrive'] = data.curDirDrive
    recv['desireDirDrive'] = data.desireDirDrive

def getSteerAngle(data):
    recv['steerAngle'] = data.steer_angle
    recv['eta'] = data.eta

def getDaIAI(data):
    recv['velDesire'] = data.velDesire

if __name__ == '__main__':
    rospy.init_node('socket', anonymous = False)
    recv = {'curDirDrive':FORWARD, 'desireDirDrive':FORWARD, 'steerAngle':0., \
            'velDesire':0., 'eta':0.}

    rospy.Subscriber("planOutput", PlanOP, getModeDrive)
    rospy.Subscriber('steerAngleCmd', stm32TX, getSteerAngle)
    rospy.Subscriber('daIAI_cmd', stm32TX, getDaIAI)

    statePub = rospy.Publisher("vehState", VehState, queue_size =1)

    # state = State()
    # rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # 一直等待client端連線
        while 1:
            try:
                # 連線成功時出現連線資訊
                conn, addr = s.accept()
                clientaddress = addr[0]+':'+str(addr[1])
                print 'Connected with ' + clientaddress

                # 開啟一個thread(每一個新連線就會開一個thread,限制為listen參數)
                thread.start_new_thread(clientthread ,(conn,clientaddress))
            except:
                print 'connection interrupt'
                np.save(os.path.join(cur_path,'../log/logSocket.npy'),GLBlog)
                conn.close()
                s.close()
                break
        # rate.sleep()
        break
    rospy.spin()    #rospy.spin()作用是当节点停止时让python程序退出，和C++ spin的作用不同
