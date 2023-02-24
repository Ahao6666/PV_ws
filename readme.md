# PX4 offboard mode 

## start gazebo 
- `make px4_sitl gazebo`
- `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

or 
- `roslaunch px4 mavros_posix_sitl.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

or
- `roslaunch px4 mavros_pv_sitl.launch`

## open QGC
- `cd ~/Downloads`
- `./QGroundControl.AppImage`
or
- `QGC` directly

## run pv_mission mode
### run offboard node
- `source ~/mavros_ws/devel/setup.bash`
- `rosrun pv_mission off_mission_node`

### run actuator_ctrl node
- `rosrun pv_mission actuator_ctrl`

在保留位置、姿态控制的条件下，**调节四个控制量有问题**（滚转、俯仰、偏航、油门）。参考actuator_ctrl.cpp，mavros_msgs::ActuatorControl不能直接发布，不然会报错，只能强制关闭lockstep(参考:https://docs.px4.io/main/en/simulation/)，如此一来，仿真中各个传感器的时间都不同步了，飞行效果很差，不可使用。

### run att_rate_setpoint node
- `rosrun pv_mission att_rate_setpoint`

直接设置**姿态角速度参考值**可以实现（参考文件att_rate_setpoint.cpp），但其中对发布位置参考值和角速度参考值的时机需要调整，可以通过if判断实现切换。还存在的问题是角速度参考值有时候没有发布，还不清楚是什么原因

### run att_setpoint node
- `rosrun pv_mission att_setpoint`

直接设置**姿态角参考值**可以实现两边升力的不同（参考：att_setpoint.cpp），效果较为理想，后续可以进一步修改

### run inclined landing node
- `rosrun pv_mission inclined_landing`

仿真斜面降落流程，较为顺利

## run landing leg node
### test landing simualtion
- `roslaunch px4 mavros_pv_sitl.launch`
- `rosrun landing_leg landing_leg_controller`
- `rosrun landing_leg landing_leg_server`

- `rosrun landing_leg landing_leg_client_init`
for test the landing leg operation

- `rosrun landing_leg landing_leg_client`
for test the incline landing operation

## force_torque sensor test
- `roscore`
- `cd ~/PX4-Autopilot/Tools/sitl_gazebo/worlds/`
- `gazebo --verbose force_sensor.world`
- `rosrun landing_leg force_torque_sensor`
- `rosrun landing_leg landing_leg_ft_sensor`

## the whole simualtion precesss is 
1. start PV simulation
- `roslaunch pv4 mavros_pv_sitl.launch`

2. open QGroundControl
- `QGC`

3.  start landing_leg nodes
- `roslaunch launch_folder landing_leg`

4. start pv_mission node
- `roslaunch launch_folder pv_mission`  

5. start tcp test  
- under python folder, after `roscore`;    
- `rosrun landing_leg landing_leg_tcp_client`  
- note: ip Address and port need to check