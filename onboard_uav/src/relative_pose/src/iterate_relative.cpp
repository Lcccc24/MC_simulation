#include <nlink_parser/LinktrackNodeframe2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <Eigen/Eigen>
#include <ros/ros.h>


double uwb_distance;
ros::Publisher ite_estimate_pub;
Eigen::Vector3d uav_odom_pos_, ite_c2d_q;
geometry_msgs::PoseStamped uav_local_pose_; 
geometry_msgs::Vector3 ite_c2d;

void iterate_estimate()
{
    //子机相对于对接点的位置
    static Eigen::Vector3d vio_pk, vio_pk0, Delta_pk, ite_vel, sigma;
    static Eigen::Vector2d rho;
    static int ite_k = 0;
    static float uwb_intermediate, error_k;
    static float a = 72;
    static float theta = 0;
    static float b1_val = 1/2;
    static float b2_val = sqrt(3)/2;
    static float gamma_val = 10;
    static float alpha_val = 3;
    static float beta_value = 8;
    static double vel_max = 10.0f;
    static double uwb_dk,uwb_dk0;
    static float iterate_time = 0.1f;
    static Eigen::Matrix2d D;
    D << cos(2*M_PI/a), -sin(2*M_PI/a),
    sin(2*M_PI/a), cos(2*M_PI/a);


    if(ite_k == 0)
    {   
        vio_pk0 = uav_odom_pos_;
        uwb_dk0 = uwb_distance;
        rho << cos(theta), sin(theta);
        sigma << b1_val*rho.x(), b1_val*rho.y(), b2_val*(4*pow(rho.x(),3) - 3*rho.x());
        ite_c2d_q = {1,1,1};
        ite_k = 1;
    }

    else
    {
        vio_pk = uav_odom_pos_;
        uwb_dk = uwb_distance; 
        Delta_pk = vio_pk - vio_pk0;
        rho = D*rho;
        //sigma << b1_val*(4*pow(rho.x(),3) - 3*rho.x()), b1_val*(3*rho.y() - 4*pow(rho.y(),3)), b2_val*rho.x(); 
        sigma << b1_val*rho.x(), b1_val*rho.y(), b2_val*(4*pow(rho.x(),3) - 3*rho.x());
        uwb_intermediate = (pow(uwb_dk,2) - pow(uwb_dk0,2) - pow(Delta_pk.norm(),2)) / 2;
        error_k = uwb_intermediate - Delta_pk.transpose()*ite_c2d_q;
        ite_c2d_q = (ite_c2d_q + Delta_pk + gamma_val*Delta_pk*error_k);
        ite_c2d_q = ite_c2d_q * uwb_dk / std::max(ite_c2d_q.norm(),uwb_dk);
        
        //update parameter
        vio_pk0 = vio_pk;
        uwb_dk0 = uwb_dk;
    }

}

void Iterate_Callback(const ros::TimerEvent&)
{
    iterate_estimate();
    ite_c2d.x = ite_c2d_q.x();
    ite_c2d.y = ite_c2d_q.y();
    ite_c2d.z = ite_c2d_q.z();
    ite_estimate_pub.publish(ite_c2d);
    ROS_INFO("relative_pos: [%f, %f, %f]", ite_c2d_q.x(), ite_c2d_q.y(), ite_c2d_q.z());
}

void Uwb_distance_callback(const std_msgs::Float64 msg) 
{ 
    uwb_distance = msg.data;
    //ROS_INFO("uwbdistance %f",uwb_distance);
}

void UavLocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_local_pose_ = *msg;
    uav_odom_pos_ << uav_local_pose_.pose.position.x, uav_local_pose_.pose.position.y, uav_local_pose_.pose.position.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "iterate_relative");
    ros::NodeHandle nh;

    ros::Subscriber uwb_distance_sub_  = nh.subscribe("/fake_uwb_distance", 1, Uwb_distance_callback);

    ros::Subscriber uav_local_pose_sub_ = nh.subscribe("/Sub_UAV/mavros/local_position/pose", 1, UavLocalPoseCallback);
    
    ite_estimate_pub = nh.advertise<geometry_msgs::Vector3>("/ite_relative/estimate", 1);

    //20hz
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), Iterate_Callback);

    ros::spin();

    return 0;
}
