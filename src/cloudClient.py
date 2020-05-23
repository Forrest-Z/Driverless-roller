#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 29 15:23:35 2018

@author: yq
"""
import rospy
import requests
import time
import json
from  campus_driving.msg import cloudTask, PlanOP
from  geometry_msgs.msg import Point32
import os
import numpy as np

LISTEN_IP='60.205.106.133'
LISTEN_PORT=8080

Error=0 #0表示没有任何问题,1表示发生错误需要工作人员处理，2表示因为临时原因需要取消任务
i=0
NTNumber=0
TId=1
VId=0
VCompactionType=0
VPathType=0
VSpeed=1
NYCount=0
JointWidth=0
MiniR=0
MiniBack=0
TreadWidth=0
P1X=0
P1Y=0
P2X=0
P2Y=0
Path0X1=0
Path0Y1=0
Path0X2=0
Path0Y2=0
Path1X=0
Path1Y=0
Path1R=0
Path1StartAngle=0
Path1EndAngle=0
Path1X1=0
Path1Y1=0
Path1X2=0
Path1Y2=0
HQX1=0
HQY1=0
HQZ1=0
HQX2=0
HQY2=0
HQZ2=0
HQlen=0
obs_Marker=[]

startTask=0
taskCount=0
endTask=0
hasstart=0

def getModeDrive(msg):
    global endTask, hasstart
    endTask=msg.endTaskFlag
    hasstart=msg.startedFlag

def talker():#发布话题，传递相关参数
    msg=cloudTask()
    msg.Error= Error
    msg.i= i
    msg.NTNumber= NTNumber
    msg.TId= TId
    msg.VId= VId
    msg.VCompactionType= VCompactionType
    msg.VPathType= VPathType
    msg.VSpeed= VSpeed
    msg.NYCount= NYCount
    msg.JointWidth= JointWidth
    msg.MiniR= MiniR
    msg.MiniBack= MiniBack
    msg.TreadWidth= TreadWidth
    msg.obs_marker=obs_Marker
#    msg=cloudTask()
    msg.startTask=startTask
    msg.taskID=taskCount
    pub.publish(msg)

def dget(dictionary,cmd,default=None):#获取3层嵌套字典对应键的值
    cmd_list=cmd.split('.')
    tmp=dict(dictionary)
    for c in cmd_list:
        try:
            val=tmp.get(c,None)
        except AttributeError:
            return default
        if val!=None:
            tmp=val
        else:
            return default
    return tmp


if __name__=='__main__':
    rospy.init_node('cloudClient',anonymous=True)
    rospy.Subscriber('planOutput', PlanOP, getModeDrive)
    pub=rospy.Publisher('cloudChatter',cloudTask,queue_size=50)
    rate=rospy.Rate(20)
    #vhvi={'Error':0.,'i':0.,'NTNumber':0.,'TId':0.,'VId':0.,'VCompactionType':0.,\
    #      'VPathType':0.,'VSpeed':0.,'P1X':0.,'P1Y':0.,'P2X':0.,'P2Y':0.,'Path0X1':0.,\
    #     'Path0Y1':0.,'Path0X2':0.,'Path0Y2':0.,'Path1X':0.,'Path1Y':0.,'Path1R':0.,\
    #     'Path1StartAngle':0.,'Path1EndAngle':0.,'Path1X1':.0,'Path1Y1':0.,'Path1X2':0.,\
    #     'Path1Y2':0.,'HQX1':0.,'HQY1':0.,'HQZ1':0.,'HQX2':0.,'HQY2':0.,'HQZ2':0.}
    while not rospy.is_shutdown():
        if Error==0:
            if i==0:#接收当前车辆的新任务的数量判断
                r=requests.get('http://60.205.106.133:8080/AutoDrive/NewAutoDriveTaskCount?vehicleId=20')
                #print r.content
                json_response=r.content.decode()
                #print r.text
                dict_json=json.loads(json_response)
                if dict_json.get('HasError')==0:
                    NTNumber=dict_json.get('Data')
                    print 'the NTNumber is %d' % NTNumber
                    if NTNumber>=1:
                        i=i+1
                    else:
                        i=0
            if i==1:#获取到新任务后，获取当前车辆最前任务的相关任务信息
               r=requests.get('http://60.205.106.133:8080/AutoDrive/NewAutoDriveTask?vehicleId=20')
               #print r.text

               dict_json=json.loads(r.content)
               #print dict_json
               #print type(dict_json)
               #print 1


               if dict_json.get('HasError')==0:
                   TId=dict_json['Data'].get('Id')
                   VId=dict_json['Data'].get('VehicleId')
                   VCompactionType=dict_json['Data'].get('CompactionType')
                   VPathType=dict_json['Data'].get('PathType')
                   VSpeed=dict_json['Data'].get('Speed')
                   NYCount=dict_json['Data'].get('CompactionCount')
                   JointWidth=dict_json['Data'].get('JointWidth')
                   MiniR=dict_json['Data'].get('MinCornerLoopRadius')
                   MiniBack=dict_json['Data'].get('MinBackingDistance')
                   TreadWidth=dict_json['Data'].get('TreadWidth')
                   P1X=dget(dict_json,'Data.Point1.X')
                   #print 2
                   P1Y=dget(dict_json,'Data.Point1.Y')
                   P2X=dget(dict_json,'Data.Point2.X')
                   P2Y=dget(dict_json,'Data.Point1.Y')
                   a=dict_json['Data']
                   b=a['PathSegments']
                   #print b
                   print len(b)
                   #print b[0]
                   cur_path = os.path.dirname(__file__)
                   ZUOBIAO_all=()
                   np.save(os.path.join(cur_path,'../map/yunduanshuju.npy'),b)
                   ZUOBIAO_all=np.load(os.path.join(cur_path,'../map/yunduanshuju.npy'))
                   print ZUOBIAO_all[1]
                   print ZUOBIAO_all[3]
                   print ZUOBIAO_all[5]
                   c=a['Markers']
                   if len(c)==0:
                       print 'has no HQ data'
                   else:
                       HQlen=len(c)
                       obs_Marker=[]
                       for i in range (c):
                           po=Point32()
                           po.x=c[i].get(x)
                           po.y=c[i].get(y)
                           po.z=c[i].get(z)
                           obs_Marker.append(po)
                      
                   i=i+1

            if i==2:#接收到任务相关信息后，通知后台已经接收到新任务
                url='http://60.205.106.133:8080/AutoDrive/ReceiveAutoDriveTask'
                d={'taskId':9}
                r=requests.post(url,data=d)
                #print r.text
                #print type(r)
                dict_json=json.loads(r.content)
                #print dict_json
                time.sleep(0.01)#延时0.01s，防止有信息传递延时到达的情形致使错误判断为接收不到信息
                #print 1
                if dict_json.get('HasError')==0:
                    #print 2
                    i=i+1
                    #print dict_json.get('HasError')
                    #print 2
                    #print i
                else:
                    i=2
            if i==3:#通知后台已经接收任务后，询问当前任务是否开始
                r=requests.get('http://60.205.106.133:8080/AutoDrive/IsAutoDriveTaskStarted?taskId=20')
                dict_json=json.loads(r.content)
                if dict_json.get('HasError')==0:
                    #print 1
                    if dict_json.get('Data')==0:#此处测试程序时暂时写为0，正式测试时应该写成1
                        #print 1
                        i=i+1
                    else:
                        i=3
                else:
                    i=3
            if i==4:#请求后台可以开始任务后，执行任务，并通知后台已经开始执行任务
                startTask=1
                taskCount=taskCount+1
                if hasstart==1:
                #此处与车辆控制程序结合，需要添加一个开始任务的判断
                    url='http://60.205.106.133:8080/AutoDrive/ExecuteAutoDriveTask'
                    d={'taskId':9}
                    r=requests.post(url,data=d)
                    dict_json=json.loads(r.content)
                    time.sleep(0.01)
                    #print 1
                    if dict_json.get('HasError')==0:
                        #print 1
                        i=i+1
            if i==5:#完成任务后，通知后台任务已经完成
                #与车辆控制程序相结合，需要添加一个车辆任务是否完成的判断
                if endTask==1:
                    url='http://60.205.106.133:8080/AutoDrive/FinishAutoDriveTask'
                    d={'taskId':9}
                    r=requests.post(url,data=d)
                    dict_json=json.loads(r.content)
                    time.sleep(0.01)
                    #print 1
                    if dict_json.get('HasError')==0:
                        startTask=0
                    #print 1
                        i=0#车辆完成任务后，开始新一轮循环，将i置0

        elif Error==1:#有错误发生时，打印有问题需要被处理提醒
            url='http://60.205.106.133:8080/AutoDrive/TerminateAutoDriveTask'
            d={'taskId':9}
            r=requests.post(url,data=d)
            dict_json=json.loads(r.content)
            print 'has error need to be solved'
        elif Error==2:#因情况任务取消，说明情况
            url='http://60.205.106.133:8080/AutoDrive/CancelAutoDriveTask'
            d={'taskId':9}
            r=requests.post(url,data=d)
            dict_json=json.loads(r.content)
            print 'has something wrong The task is canceled'
            Error=0
        talker()
        print 'testtt'
        rate.sleep()
