#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import yaml
import os

# Loading Parameters
cur_path = os.path.dirname(__file__)
with open(os.path.join(cur_path,'../params/param.yaml'),'r') as f:
    param = yaml.load(f)
X_ORI = param["map"]["X_ORI"]
Y_ORI = param["map"]["Y_ORI"]

def gps_2_xy(longitude,latitude):
    """
    """
    global X_ORI,Y_ORI

    a_rad=6378137      #地球半径
    #c_1=0.08181919     #第一偏心率
    #c_2=0.082094438     #第二偏心率
    f=298.2572235635
    c_1=np.sqrt(1-((f-1)/f)**2)
    c_2=np.sqrt((f/(f-1))**2-1)
#    g=0.0033528     #椭球扁率

    B_deg=latitude
    L_deg=longitude
    B=B_deg*np.pi/180.0
    L=L_deg
    l_deg=L-(6*20-3)
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
    y=X+(N*t*(l**2)*np.cos(B)*np.cos(B))*(1.0/2.0+1.0/24.0*(l**2)*(5-t**2+9*u**2+4*u**4)*np.cos(B)*np.cos(B)+(1.0/720.0)*l**4*(61-58*t**2+t**4+270*u**2-330*u**2*t**2)*np.cos(B)**4)
    x=500000+N*l*np.cos(B)*(1+(1.0/6.0)*l**2*np.cos(B)*np.cos(B)*(1-t**2+u**2)+(1.0/120.0)*l**4*np.cos(B)*np.cos(B)*np.cos(B)*np.cos(B)*(5-18*t**2+t**4+14*u**2-58*t**2*u**2))
#    x_ori = 3380400.6653839252   #车库前
#    y_ori = 245529.56042564771  #车库前
    x_offset = x  + 282706.391602          # 统一到M30坐标系，指向正东
    y_offset = y  + (-4005.85898309)  # 统一到M30坐标系，指向正北
    x_offset = x_offset-X_ORI          #指向正东
    y_offset = y_offset-Y_ORI    #指向正北
    return x_offset, y_offset

if __name__ == "__main__":
    a= gps_2_xy(115.55,32.33)
    print a
