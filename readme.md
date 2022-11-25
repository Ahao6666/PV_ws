# PX4 offboard mode 

## start gazebo 
- `make px4_sitl gazebo`
- `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

or 
- `roslaunch px4 mavros_posix_sitl.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

## open QGC
- `cd ~/Downloads`
- `./QGroundControl.AppImage`

## run offboard node
- `source ~/mavros_ws/devel/setup.bash`
- `rosrun offboard offboard_node`


## run actuator_ctrl node
- `rosrun offboard actuator_ctrl`
在保留位置、姿态控制的条件下，调节四个控制量有问题（滚转、俯仰、偏航、油门）。参考actuator_ctrl.cpp，mavros_msgs::ActuatorControl不能直接发布，不然会报错，只能强制关闭lockstep(参考:https://docs.px4.io/main/en/simulation/)，如此一来，仿真中各个传感器的时间都不同步了，飞行效果很差，不可使用。

## run att_rate_setpoint node
- `rosrun offboard att_rate_setpoint`
直接设置**姿态角速度参考值**可以实现（参考文件att_rate_setpoint.cpp），但其中对发布位置参考值和角速度参考值的时机需要调整，可以通过if判断实现切换。还存在的问题是角速度参考值有时候没有发布，还不清楚是什么原因


