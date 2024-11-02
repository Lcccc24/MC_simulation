Header header
uint8 uav_id # 本机ID
uint8 target_uav_id # 目标无人机ID
uint8 flight_command # 母机飞行指令：0-无指令 1-起飞 2-任务 3-对接 4-降落 5-允许返航 6-允许精准降落
uint8 flight_status # 子机飞行状态：10-起飞完成 20-任务完成 40-降落完成 40-返航完成 50-精准降落完成 
geometry_msgs/Point position # 位置： 任务点 返航点

# 母机飞行指令
uint8 IDLE = 0 # 待机
uint8 TAKEOFF =1 # 起飞
uint8 MISSION = 2 # 执行任务
uint8 DOCKING = 3 # 对接
uint8 LAND = 4 # 降落
uint8 ALLOW_RETURN = 5 # 允许返航
uint8 ALLOW_PRECISION_LANDING = 6 # 允许精准降落

# 子机飞行状态
uint8 TAKEOFF_COMPLETE = 10 # 起飞完成
uint8 MISSION_COMPLETE = 20 # 任务完成
uint8 LAND_COMPLETE = 30 # 降落完成
uint8 RETURN_COMPLETE = 40 # 返航完成
uint8 PRECISION_LANDING_COMPLETE = 50 # 精准降落完成


