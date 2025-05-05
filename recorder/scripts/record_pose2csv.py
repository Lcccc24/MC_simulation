#!/usr/bin/env python3
import rospy
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# 设置CSV文件路径
csv_file_path = "/home/lc/mc_simu_ws/pose_data.csv"  # 修改为实际路径

# 存储两架飞机的位置信息
plane1_position = [None, None, None]
plane2_position = [None, None, None]

def write_to_csv():
    """
    将两架飞机的数据合并后写入CSV文件
    """
    if None not in plane1_position and None not in plane2_position:
        with open(csv_file_path, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow(plane1_position + plane2_position)  # 合并两架飞机的位置
        rospy.loginfo(f"Recorded positions: Plane1 {plane1_position}, Plane2 {plane2_position}")

def pose_callback1(msg):
    """
    第一架飞机的回调函数，更新位置信息
    """
    global plane1_position
    plane1_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    write_to_csv()

def pose_callback2(msg):
    """
    第二架飞机的回调函数，更新位置信息
    """
    global plane2_position
    #x+1.0 初始位置时子机相对与母机的位置 使字母机的位置基于同一坐标系
    plane2_position = [msg.pose.position.x + 1.0, msg.pose.position.y, msg.pose.position.z]
    write_to_csv()

def main():
    """
    主函数，初始化ROS节点并订阅话题
    """
    rospy.init_node('pose_to_csv_recorder', anonymous=True)

    # 参数配置
    topic_type1 = rospy.get_param("~topic_type1", "PoseStamped")  # 第一架飞机的消息类型
    topic_name1 = rospy.get_param("~topic_name1", "/uav0/mavros/local_position/pose")  # 第一架飞机的消息话题

    topic_type2 = rospy.get_param("~topic_type2", "PoseStamped")  # 第二架飞机的消息类型
    topic_name2 = rospy.get_param("~topic_name2", "/mavros/local_position/pose")  # 第二架飞机的消息话题

    # 创建CSV文件并写入表头
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(["m_x", "m_y", "m_z", "c_x", "c_y", "c_z"])  # 表头

    # 根据话题类型订阅第一架飞机的位置
    if topic_type1 == "PoseStamped":
        rospy.Subscriber(topic_name1, PoseStamped, pose_callback1)
    elif topic_type1 == "Odometry":
        rospy.Subscriber(topic_name1, Odometry, pose_callback1)
    else:
        rospy.logerr("Invalid topic type for plane1. Use 'PoseStamped' or 'Odometry'.")
        return

    # 根据话题类型订阅第二架飞机的位置
    if topic_type2 == "PoseStamped":
        rospy.Subscriber(topic_name2, PoseStamped, pose_callback2)
    elif topic_type2 == "Odometry":
        rospy.Subscriber(topic_name2, Odometry, pose_callback2)
    else:
        rospy.logerr("Invalid topic type for plane2. Use 'PoseStamped' or 'Odometry'.")
        return

    rospy.loginfo(f"Subscribed to {topic_name1} and {topic_name2}, recording to {csv_file_path}")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
