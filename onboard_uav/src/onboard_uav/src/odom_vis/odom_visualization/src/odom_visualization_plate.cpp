#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// #include <Eigen/Geometry>
#include <eigen3/Eigen/Geometry>

std::string uav0_mesh_resource, uav1_mesh_resource;

ros::Publisher uav0_pose_vis_pub_, uav1_pose_vis_pub_;

void uav0_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  visualization_msgs::Marker uav0_pose_vis;
  uav0_pose_vis.header = msg->header;
  uav0_pose_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
  uav0_pose_vis.action = visualization_msgs::Marker::ADD;
  uav0_pose_vis.mesh_use_embedded_materials = true;
  uav0_pose_vis.mesh_resource = uav0_mesh_resource;
  uav0_pose_vis.pose = msg->pose;
  uav0_pose_vis.scale.x = 2.5;
  uav0_pose_vis.scale.y = 2.5;
  uav0_pose_vis.scale.z = 2.5;
  uav0_pose_vis.color.a = 1.0;
  uav0_pose_vis.color.r = 255.0 / 255.0;
  uav0_pose_vis.color.g = 190.0 / 255.0;
  uav0_pose_vis.color.b = 122.0 / 255.0;
  uav0_pose_vis_pub_.publish(uav0_pose_vis);
}

void uav1_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  visualization_msgs::Marker uav1_pose_vis;
  uav1_pose_vis.header = msg->header;
  uav1_pose_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
  uav1_pose_vis.action = visualization_msgs::Marker::ADD;
  uav1_pose_vis.mesh_use_embedded_materials = true;
  uav1_pose_vis.mesh_resource = uav1_mesh_resource;
  uav1_pose_vis.pose = msg->pose;
  uav1_pose_vis.scale.x = 1.0;
  uav1_pose_vis.scale.y = 1.0;
  uav1_pose_vis.scale.z = 1.0;
  uav1_pose_vis.color.a = 0.0;
  uav1_pose_vis.color.r = 0.0;
  uav1_pose_vis.color.g = 0.0;
  uav1_pose_vis.color.b = 0.0;
  uav1_pose_vis_pub_.publish(uav1_pose_vis);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_visualization");
  ros::NodeHandle nh;

  uav0_mesh_resource = "package://odom_visualization/meshes/hummingbird.mesh";
  uav1_mesh_resource = "package://odom_visualization/meshes/f250.mesh";

  ros::Subscriber uav0_pose = nh.subscribe("/uav0/mavros/local_position/local", 100, uav0_pose_callback);
  ros::Subscriber uav1_pose = nh.subscribe("/mavros/local_position/local", 100, uav1_pose_callback);

  uav0_pose_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/uav0/pose/visualization", 100, true);
  uav1_pose_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/uav1/pose/visualization", 100, true);

  ros::spin();
  return 0;
}
