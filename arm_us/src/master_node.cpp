/**
 * @file master_node.cpp
 * @author Mikael St-Arnaud et Philippe Michaud (stam1001, micp1402)
 * @brief This is the master_node and runs the main loop
 * @version 0.1
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023 - See ARM_US licence
 */

#include "ArmUs.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "master_node");

    ArmUs arm_us;
    arm_us.Run();

    return 0;
}