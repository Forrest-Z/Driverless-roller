# message 1465
# novatel_msgs/CommonHeader header uncomment by tlq

# Table 29 in the SPAN on OEM6 manual:
# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121

# ref: /ubuntu-exchange/novatel_span_driver-master/novatel_span_driver-master
# ref: https://github.com/ros-drivers/novatel_span_driver

std_msgs/Header header

float64 latitude
float64 longitude
float64 altitude

float64 north_velocity
float64 east_velocity
float64 up_velocity

float64 roll
float64 pitch
float64 azimuth

string ins_status
uint32 INS_STATUS_INACTIVE=0
uint32 INS_STATUS_ALIGNING=1
uint32 INS_STATUS_HIGH_VARIANCE=2
uint32 INS_STATUS_SOLUTION_GOOD=3
uint32 INS_STATUS_SOLUTION_FREE=6
uint32 INS_STATUS_ALIGNMENT_COMPLETE=7
uint32 INS_STATUS_DETERMINING_ORIENTATION=8
uint32 INS_STATUS_WAITING_INITIALPOS=9

float64 x
float64 y

float64 a_x
float64 a_y
float64 a_z
float64 w_x
float64 w_y
float64 w_z

#编码器读取参数
uint64 startQuanshu
int64 Quanshu
uint64 startAngle
float64 angle
