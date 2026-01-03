#include "planner/onboard_uav_fsm.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "onboard_uav_node");
    ros::NodeHandle nh;
    OnboardUavFsm onboard_uav_fsm(nh);
    ros::spin();
    return 0;
}