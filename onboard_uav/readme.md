# 仿真
## 启动程序
```bash
# 启动PX4、Gazebo、mavros
roslaunch uav_launch multi_uav_mavros_sitl.launch

# 决策规划和PX4的中间层，进入OFFBOARD状态，一键起飞、降落，也可用于将位置控制指令转换为姿态控制指令
roslaunch uav_launch run_px4_ctrl.launch

# 相对位姿估计算法，输出目的点在惯性系下的位姿，uav_landing_pose为KF，landing_target_pose为ESKF，真机使用ESKF还需修改相关程序，目前仅用于测试
roslaunch uav_launch uav_landing_pose.launch
#roslaunch vision_pose landing_target_pose.launch

# 状态机，实现远程引导到精准降落的转变；规划器，输出优化后的轨迹
roslaunch uav_launch onboard_uav_fsm.launch

# 用以控制子机的状态
rosparam set /uav0_onboard/flight_command 1 #2任务 5返航 6降落
```
