#!/usr/bin/python
# -*- coding: utf-8 -*-


import numpy as np
import time
import matplotlib.pyplot as plt
import commonFcn

'''
python的类变量和C++的静态变量不同，并不是由类的所有对象共享。
类本身拥有自己的类变量（保存在内存），当一个TestClass类的对象被构造时，
会将当前类变量拷贝一份给这个对象，当前类变量的值是多少，
这个对象拷贝得到的类变量的值就是多少；而且，通过对象来修改类变量，
并不会影响其他对象的类变量的值，因为大家都有各自的副本，
更不会影响类本身所拥有的那个类变量的值；只有类自己才能改变类本身拥有的类变量的值。

在一个文件内通过直接给类变量赋值，可以相当于全局变量使用，
但是其他文件再通过import方式引用，则又创建了另外一个副本。
'''
class Employee(object):
   '所有员工的基类'
   empCount = 0
 
   def __init__(self, name, salary):
      self.name = name
      self.salary = salary
      Employee.empCount += 1
   
   def displayCount(self):
       Employee.empCount += 1
       print "Total Employee %d" % Employee.empCount
 
   def displayEmployee(self):
      print "Name : ", self.name,  ", Salary: ", self.salary
    
def func(f):
    sensor['d'] = 3
    a = 2
    
    
class Person(object):
    def __init__(self,name):
        self.name = name

        
    def ddd(self):
        a = self.name
        self.aw = 2
        print 'test'

class Child(Person):                # Child 继承 Person
    def __init__(self,name,sex,mother,father):
#        Person.__init__(self,name,sex,mother,father)
        self.name = name
        self.sex = sex


if __name__ == '__main__':
    a = Employee('d',2)
    print 'dd',a.__doc__, a.__sizeof__
    a.displayCount()
    Employee.empCount += 1
    a.displayCount()
    print 'dd',a.empCount
    sensor = {'d':10,'ddddddddd':45,'eeeeeeeeeeeeeeeeee':25, \
              'df':78,'ww':89}
    print 'sensor',sensor
    a = 1
    func(4)
    print sensor
    input = 1
    b = Person('d')
    b.ddd()
    print 'bbbb',b.aw
    recv = {'x':0., 'y':0., 'z':0., 'roll':0., 'pitch':0., 'azimuth':0., \
                'polyCoeffiCenter':np.zeros([3]), }
    print 'recv',recv
    d = commonFcn.saturation(10,20,35)
    print 'd',d
    
    x = 1
    polyCoeffiCenter = np.array([1,2,3])
    # 一阶导 ref:http://blog.csdn.net/blog_empire/article/details/39298557
    polyder1CoeffiCenter = np.polyder(polyCoeffiCenter) #导函数多项式系数
    polyder1Center = np.polyval(polyder1CoeffiCenter,x)
    # 二阶导
    polyder2CoeffiCenter = np.polyder(polyder1CoeffiCenter)
    polyder2Center = np.polyval(polyder2CoeffiCenter,x)
    #曲率数组,如何指定方向?
    curvatureCenter = np.abs(polyder2Center)/(1+polyder1Center**2)**(3./2.)
    curvature = np.abs(2 * polyCoeffiCenter[0]) / ((1 + (2 * polyCoeffiCenter[0] * x  + polyCoeffiCenter[1])**2)**1.5)
    curvature1 = commonFcn.calcCurvature(polyCoeffiCenter,x)
    curvature2 = commonFcn.calcQuadCurvature(polyCoeffiCenter,x)

    print 'polyder1CoeffiCenter, polyder2CoeffiCenter',polyder1CoeffiCenter, polyder2CoeffiCenter
    print 'curvatureCenter, urvature_radius_in_meters,curvature1', curvatureCenter, curvature,curvature1,curvature2
    
    A = np.array([[1.,1,1],[2.,-1.,2],[2,2,1]])
    b = np.array([9,5,3])
    x = np.linalg.solve(A,b)
    print 'x',x
    
    # bezier
    A =  np.array([[-1,3,-3,1],[3,-6,3,0],[-3,3,0,0],[1,0,0,0]])
    t = np.linspace(0,1,100)
    Point = np.array([[0,0], [2, 1], [3, 7], [6, 5]])
    Param = np.dot(A, Point) 
    x = np.polyval(Param[:,0],t)*1
    y = np.polyval(Param[:,1],t)*1
    wayPointLocal = np.array([x,y])
    plt.plot(x,y)
    start = time.time()
    dfMax = 1.8
    dfStep = 0.1
    d0 = 0.5
    dfMap = np.arange(-dfMax,dfMax+dfStep,dfStep).tolist()
    numCandidate = len(dfMap)
    d0Map = [d0]*numCandidate
    wayPointLocalMap = [wayPointLocal]*numCandidate
    candi = map(commonFcn.genCandidate, d0Map,dfMap,wayPointLocalMap)
#    candi = commonFcn.genCandidate(1,wayPointLocal)
    end = time.time()
    print 'cost',end-start
    
    for (xc,yc) in candi:
        plt.plot(xc,yc,'r') 
        
    a = commonFcn.meanFilter(2)
    a.update(1)
    a.update(2)
    a.update(3)
    d = a.update(1)
    b = commonFcn.meanFilter(2)
    b.update(5)
    b.update(6)
    b.update(7)
    c = b.update(8)
    d = a.update(4)
    
    print 'd',d,a
    print 'c',c,b
    position = {'x':3,'y':3,'azimuth':180}
    pl = np.array([-1,-1])
    pg = commonFcn.local_2_global_point(position,pl)
    print 'pg',pg
    position = {'x':3,'y':3,'azimuth':90}
    pg = np.array([[4,1],[2,1]])
    pl = commonFcn.global_2_local_vect(position,pg)
    print 'pl',pl
    