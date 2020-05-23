std_msgs/Header header
geometry_msgs/Point prePoint    # preview point

float32[] xCtrPath              # x array in path of control module, represent in the global coordinate.

float32[] yCtrPath              # y array in path of control module, represent in the global coordinate.

float32 distObst                # obstacle distance from vehicle origin, currently means the closest obstacle distance

int16 desireDirDrive            # desired driving direction. 0:forward; 1:back

uint8 curDirDrive               # actual driving direction. 0:forward; 1:back. 这是源头，VehState.msg是转发。区别于发给底层的期望行驶方向。两者区别在于换挡过渡阶段有所不同

int16 flagIsShifting            # planning module verify this state from MCU or human driver

float32 distYClose              # the minimum lataral error

int16 modeDrive                 # dirving mode. 0: INIT; 1: NORMAL; 2: STOP; 4: STOP_SAFE. STOP_SAFE means must stop the car due to obstacle.

# used in safeArea
uint8 stopSafeArea

# for cloudClient
uint8 endTaskFlag #结束任务标志，0：表示任务还未结束； 1：表示当前任务已经结束
uint8 startedFlag #是否开始任务判断  0：表示车辆还未开始执行任务； 1：表示车辆已经开始执行任务
