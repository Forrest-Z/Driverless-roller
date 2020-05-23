std_msgs/Header header

float32 steer_angle
float32 velocity_CAN
uint8 brake_stat
uint8 shift_stat

# for actuator
float32 da
float32 iai_s
float32 iai_v

# for velocity planning
float32 curvature
uint8 flagBigCurv
float32 velDesire

# for simulate
float32 eta
