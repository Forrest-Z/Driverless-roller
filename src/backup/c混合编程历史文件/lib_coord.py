import os
import time

import ctypes as ct
import types

import copy

import numpy as np
import numpy.ctypeslib as npct

libtest = npct.load_library("libcorrdinateEach",os.path.dirname(__file__))

def get_point(thetaAtrans, latDeviation):
    # 变量赋值
    arr_num = thetaAtrans.size
    arr_out = np.zeros((arr_num,2),dtype=np.float32)
    # 类型定义声明
    array_2d_float  = npct.ndpointer(dtype=np.float32, ndim=2, flags='CONTIGUOUS')
    array_1d_float = npct.ndpointer(dtype=np.float32, ndim=1, flags='CONTIGUOUS')
    array_1d_int = npct.ndpointer(dtype=np.int32, ndim=1, flags='CONTIGUOUS')
    # 被调用函数声明
    libtest.getNewPoint.argtypes = [array_1d_float,
                                    array_1d_float,
                                    ct.c_int,
                                    array_2d_float]
    # 函数返回值类型
    libtest.restype = None
    # numpy变量强制转换为连续存储，以便指针索引
    thetaAtrans = np.ascontiguousarray(thetaAtrans, dtype=np.float32)
    latDeviation = np.ascontiguousarray(latDeviation, dtype=np.float32)
    arr_out = np.ascontiguousarray(arr_out, dtype=np.float32)
    # 调用函数，通过指针返回值
    libtest.getNewPoint(thetaAtrans,latDeviation,ct.c_int(arr_num),arr_out)

    return arr_out
