std_msgs/Header header

float64 x           # [m] current position, Pointing to due East
float64 y           # [m] current position, Pointing to due North
float64 xVibr
float64 yVibr 
float64 z           # [m] current position, Pointing up
float64 roll        # [deg]
float64 pitch       # [deg] down: <0; up: >0.
float64 azimuth     # [deg] counterclockwise from East 0-360
float64 rawAzimuth

float64 a_x         # [m/ss] longitudinal linear acceleration
float64 a_y         # [m/ss] lateral linear acceleration
float64 a_z         # [m/ss] vertical acceleration
float64 w_x         # [rad/s] roll rate
float64 w_y         # [rad/s] pitch rate
float64 w_z         # [rad/s] yaw rate

float32 steer_angle # [deg] current steering angle, clockwise -540 - 540
float64 speed       # [km/h] >= 0
uint8 brake_stat    # 0: default; 1:brake
uint8 shift         # 1: P; 2: R; 3: N; 4: D

uint8 curDirDrive  # 0/FORWARD:前进；1/BACK：后退。当前实际行驶方向，区别于发给底层的期望行驶方向。两者区别在于换挡过渡阶段有所不同
