time stamp
uint8 Error #错误标志位,0表示无错误，1表示有错误需要人来处理，2表示因为原因当前任务取消
uint8 i
# 以下都是云端发送
uint8 NTNumber #车辆接收新任务的数量
uint8 TId #任务ID编号
uint8 VId #车辆ID编号
uint8 VCompactionType #碾压方法，0表示小错据，1表示大错据
uint8 VPathType #路线类型，0折线，1环型
uint8 VSpeed #车辆速度
uint8 NYCount #任务要求碾压遍数
float32 JointWidth #任务要求搭接宽度
float32 MiniR #车辆最小转弯半径
float32 MiniBack #任务要求最小倒车距离
float32 TreadWidth #车辆轮间距
float64 P1X #矩形区域第一个坐标x(所有坐标都是全局坐标，不是水坝局部坐标)
float64 P1Y #矩形区域第一个坐标y
float64 P2X #矩形区域第二个坐标x
float64 P2Y #矩形区域第二个坐标y
float64 Path0X1 #折线第一坐标x
float64 Path0Y1 #折线第一坐标y
float64 Path0X2 #折线第二坐标x
float64 Path0Y2 #折线第二坐标y
float64 Path1X #圆弧坐标x
float64 Path1Y #圆弧坐标y
float64 Path1R #圆弧坐标r
float32 Path1StartAngle #圆弧开始角度
float32 Path1EndAngle #圆弧结束角度
float64 Path1X1 #圆弧第一坐标x
float64 Path1Y1 #圆弧第一坐标y
float64 Path1X2 #圆弧第二坐标x
float64 Path1Y2 #圆弧第二坐标y
float64 HQX1 #第一个红旗x
float64 HQY1 #第一个红旗y
float64 HQZ1 #第一个红旗z
float64 HQX2 #第二个红旗x
float64 HQY2 #第二个红旗y
float64 HQZ2 ##第二个红旗z
uint8 startTask # 开始碾压，0: 默认，停止碾压； 1:开始碾压
uint32 taskID # 任务ID，用于判断是否新任务，从0开始
geometry_msgs/Point32[] obs_marker

