#include "landing_pose/landing_pose.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_pose");
    ros::NodeHandle nh;
    LandingPose landing_pose(nh);
    ros::spin();
    return 0;
}