ctrl-shift-M 即可打开markdown preview功能
NumPy基本方法
ref: http://blog.csdn.net/blog_empire/article/details/39298557


编程学习
1 Subscriber回调函数如何传入自己的参数。
    ref中给出了python解决方案及C++链接
    display.py中应用到，可参考。根据测试经验，只可以将参数集中到args的tuple变量中
    ref：https://answers.ros.org/question/231492/passing-arguments-to-callback-in-python/

2 几种从终端传入数据的方法
    1 flappuBird->qlearn.py中的parser
        parser = argparse.ArgumentParser(description='Description of your program')
        parser.add_argument('-m','--mode', help='Train / Run', default = 'Run', required=False)
        args = vars(parser.parse_args())
        note: 此方法可同时传入多个参数，参数修改只能在初始化时
    2 socket->server.py
        PORT = int(sys.argv[1])
        note: 此方法也是在初始化时传入，非常简单.运行后不能继续修改
    3 socket->client.py
        cmd=raw_input("Please input cmd:")       #輸入傳給server的data
        note: 此方法可在运行后，在终端持续修改.需配合while循环使用

3 避免执行器颤振的几种方法
    1 进一步优化控制参数
    2 目标量设置平衡点平衡区间
    3 控制输出多次平均
    4 忽略控制输出较小的变化，不对其进行更新

4 曲率计算
    定义：随弧长变化，曲线切线方向角变化率： K = d_alpha / dS
    两种计算公式：
        1 y = f(x)
            K = abs(ddy)/(1+dy**2)**1.5
        2 x = h(t), y = g(t)
            K = abs(dx*ddy - ddx*dy)/(dx**2+dy**2)**1.5
        ref1: https://wenku.baidu.com/view/e40fed04f12d2af90242e6f9.html
    ref2 、ref3采用的是方法1，即先拟合再连续求导;(ref中只适合二次曲线)
    ref4采用的是方法2。且为离散点计算
    个人认为实际中两种方法皆可。但是推荐采用拟合方法求导，因为实际中的点存在漂移误差问题，影响很大
    不像仿真中都是真值。
    ref2: https://github.com/IvanLim/advanced-lane-line
    ref3: https://github.com/AlexSickert/Udacity-SDC-T1-P4/blob/master/step_6_curvature.py
    ref4: https://github.com/AtsushiSakai/PythonRobotics
    ref5: 3Blue1Brown曲率数学推导可视化 https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/curvature/v/curvature-intuition

5 编译c文件
    gcc -fPIC -shared coordinateEach.c -o libcorrdinateEach.so
    -shared 该选项指定生成动态连接库（让连接器生成T类型的导出符号表，有时候也生成弱连接W类型的导出符号），不用该标志外部程序无法连接。相当于一个可执行文件. 不加：代码必须包含main；加：无需main

6 bezier,贝塞尔曲线拟合
    贝塞尔算法参考文献：
        1 中科大博士论文-辛煜  P96
        2 http://blog.csdn.net/cdnight/article/details/48468915
    ref: rosLaserPathTrack_comment.py in campus_driving-comment-backup.zip

7 局部规划路径生成
    ref:
        1 《Real-Time Trajectory Planning for Autonomous urban driving-framework, algorithms, and verifications》 paper's formula.7 .11 is wrong
        2 孙振平博硕

8 rviz视觉焦点随车辆移动的设置方法
    1 拷贝并编译package: rviz_animated_view_controller（pkg1) 和 view_controller_msgs (pkg2)(修正：先拷pkg2并编译，在操作pkg1)
    2 将campus_driving的.rviz中Visualization Manager下的Views项复制替换自己相应文件和编写rvizSet.py
    或
      将pkg1-launch-demo.rviz中Visualization Manager下的Views项内容复制替换自己的rviz对应部分
      参考pkg2-scripts-SquareTest例程，编写动态设置代码。
    ref:
        1 https://github.com/UTNuclearRoboticsPublic/rviz_animated_view_controller
        2 http://blog.csdn.net/crazyquhezheng/article/details/50493502
        3 https://answers.ros.org/question/222091/change-focal-point-in-rviz-automatically/
        4 http://wiki.ros.org/rviz_animated_view_controller
        5 http://wiki.ros.org/view_controller_msgs

9 将某个包加入到环境变量ROS_PACKAGE_PATH
    $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/dev/workspace/cartest/smartcar_description
    ref: http://blog.csdn.net/ktigerhero3/article/details/64439815

10 中心线平滑
    1 直接采用二次多项式拟合
        原因：
            1 与样条对比： 三次样条插值明显比多项式插值更逼近已知点几乎与已知点重合，过于逼近在这里反倒不是好事
            ref: https://wenku.baidu.com/view/9dc423c3bb0d4a7302768e9951e79b896802680e.html
            2 与更高次对比： 车道线或道路中心线，用二次已经足够，曲率不会多次转折
    2 曲线拟合知识扩展
        ref: 算法系列之二十一：实验数据与曲线拟合
            http://blog.csdn.net/orbit/article/details/12793343
    3 国科大论文中提到的梯度下降拟合，并不是什么新鲜事，matlab,numpy等内置拟合函数，内部有些就是用梯度下降来迭代优化逼近的。
        ref: http://tieba.baidu.com/p/3477970732

11 全局变量
    commonFcn中涉及的全局变量，只存活在调用它的那个文件或节点中，相互之间不影响，类似于在各调用文件中分别创建了相应的同名的独立全局变量。
