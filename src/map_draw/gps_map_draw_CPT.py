#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import pynmea2
import numpy as np
#import sys
import os
import matplotlib.pyplot as plt
import yaml
import time

def map_draw():
    a_rad=6378137      #地球半径
    #c_1=0.08181919     #第一偏心率
    #c_2=0.082094438     #第二偏心率
    f=298.2572235635
    c_1=np.sqrt(1-((f-1)/f)**2)
    c_2=np.sqrt((f/(f-1))**2-1)
    g=0.0033528     #椭球扁率
    GPS_all = ()
    cur_path = os.path.dirname(__file__)
    GPSfile = open(os.path.join(cur_path,'../../map/M30_dynamic_qianjin_20171103.txt'), "r")
    #GPSfile = open("/home/qidong/catkin_ws/src/campus_driving/map/xueyuan.txt", "r")
    while True:
        line = GPSfile.readline()
        if line:
            line = line.strip()
            if line.startswith('$GPGGA'):
                GPGGA = line.split(',')
                latitude = float(GPGGA[2])
                longitude = float(GPGGA[4])
                altitude = float(GPGGA[9])
    
    #            B_deg=latitude
    #            L_deg=longitude
                B_deg=int(latitude/100)+(latitude - int(latitude/100)*100)/60.
                L_deg=int(longitude/100)+(longitude - int(longitude/100)*100)/60.
                B=B_deg*np.pi/180.0
                L=L_deg
                l_deg=L-(6*20-3)    #Ÿ­¶ÈLÓëžÃµãŽøÖá×ÓÎçÏßŸ­¶È²î
                l=l_deg*np.pi/180.0
                t=np.tan(B)
    
                u=c_2*np.cos(B)
                A0=1+(3.0/4.0)*c_1**2+(45.0/64.0)*c_1**4+(350.0/512.0)*c_1**6+(11025.0/16384.0)*c_1**8
                A2=-1.0/2.0*((3.0/4.0)*c_1**2+(60.0/64.0)*c_1**4+(525.0/512.0)*c_1**6+(17640.0/16384.0)*c_1**8)
                A4=1.0/4.0*((15.0/64.0)*c_1**4+(210.0/512.0)*c_1**6+(8820.0/16384.0)*c_1**8)
                A6=-1.0/6.0*((35.0/64.0)*c_1**6+(2520.0/16384.0)*c_1**8)
                A8=1.0/8.0*((315.0/16384.0)*c_1**8)
                X=a_rad*(1-c_1**2)*(A0*B+A2*np.sin(2*B)+A4*np.sin(4*B)+A6*np.sin(6*B)+A8*np.sin(8*B))
                N=a_rad/np.sqrt(1-c_1**2*np.sin(B)*np.sin(B))
                x=X+(N*t*(l**2)*np.cos(B)*np.cos(B))*(1.0/2.0+1.0/24.0*(l**2)*(5-t**2+9*u**2+4*u**4)*np.cos(B)*np.cos(B)+(1.0/720.0)*l**4*(61-58*t**2+t**4+270*u**2-330*u**2*t**2)*np.cos(B)**4)
                y=500000+N*l*np.cos(B)*(1+(1.0/6.0)*l**2*np.cos(B)*np.cos(B)*(1-t**2+u**2)+(1.0/120.0)*l**4*np.cos(B)*np.cos(B)*np.cos(B)*np.cos(B)*(5-18*t**2+t**4+14*u**2-58*t**2*u**2))
                x_offset = x-X_ORI          #指向正北
                y_offset = y-Y_ORI    #指向正东
                GPS_all += ((x_offset,y_offset,altitude),)
                print(latitude,longitude)
                #plt.plot(y_offset,x_offset,'*')
            else:
                continue
        else:
            break
    GPSfile.close()
    GPS_all_arr = np.array(GPS_all)
    plt.figure(2)
    plt.plot(GPS_all_arr[:,1],GPS_all_arr[:,0],'*')     #绘图时，将x y互换位置以匹配地图视角
    np.save(os.path.join(cur_path,'../../map/xueyuan.npy'),GPS_all_arr)
    
    start_x=GPS_all[0][0]  #GPS起点x,y换算值
    start_y=GPS_all[0][1]    
    end_x=GPS_all[-1][0]   #GPS终点x,y换算值
    end_y=GPS_all[-1][1]   
    c_line = np.polyfit([start_x,end_x],[start_y,end_y],1)
    x_line = np.linspace(start_x,end_x,len(GPS_all))
    y_line = np.polyval(c_line,x_line)
    GPS_all_line = np.column_stack((x_line,y_line,GPS_all_arr[:,2]))
    
    plt.figure(2)
    plt.plot(GPS_all_line[:,1],GPS_all_line[:,0],'-')     #绘图时，将x y互换位置以匹配地图视角
    np.save(os.path.join(cur_path,'../../map/xueyuanLine.npy'),GPS_all_line)
    
    end_x=GPS_all[0][0]  #GPS起点x,y换算值
    end_y=GPS_all[0][1]    
    start_x=GPS_all[-1][0]   #GPS终点x,y换算值
    start_y=GPS_all[-1][1]   
    c_line = np.polyfit([start_x,end_x],[start_y,end_y],1)
    x_line = np.linspace(start_x,end_x,len(GPS_all))
    y_line = np.polyval(c_line,x_line)
    GPS_all_lineBack = np.column_stack((x_line,y_line,GPS_all_arr[:,2]))
    
    plt.figure(2)
    plt.plot(GPS_all_lineBack[:,1],GPS_all_lineBack[:,0],'-')     #绘图时，将x y互换位置以匹配地图视角
    np.save(os.path.join(cur_path,'../../map/xueyuanLineBack.npy'),GPS_all_lineBack)
    
    #np.save("/home/qidong/catkin_ws/src/campus_driving/map/xueyuan.npy",GPS_all_arr)
    
if __name__ == "__main__":
        # Loading Parameters
    base_path = os.path.dirname(__file__)
    param_file = base_path + '/../../params/param.yaml' 
    with open(param_file,'r') as f:
        param = yaml.load(f)
    X_ORI = param["map"]["X_ORI"]
    Y_ORI = param["map"]["Y_ORI"]
    map_draw()
