#include "vision_pose/landing_target_pose_jain.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_target_pose");
    ros::NodeHandle nh;
    LandingTargetPose landing_target_pose(nh);
    ros::spin();
    return 0;
}