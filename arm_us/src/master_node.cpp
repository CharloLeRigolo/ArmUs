#include "ArmUs.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "master_node");

    ArmUs arm_us;
    arm_us.Run();

    return 0;
}