#include "ArmUs.hpp"

ControlMode controlMode = ControlMode::Simulation;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "master_node");

    ArmUs arm_us(controlMode);

    arm_us.Initalize();
    arm_us.Run();

    return 0;
}

/*
void calculate_motor_velocities()
{
    motorVelocitiesCmd.m1 = (g_controller.leftJoystick.vertical + g_controller.rightJoystick.horizontal) / 2;
    motorVelocitiesCmd.m2 = (g_controller.leftJoystick.vertical - g_controller.rightJoystick.horizontal) / 2;
    motorVelocitiesCmd.m3 = 0;
    motorVelocitiesCmd.m4 = 0;
    motorVelocitiesCmd.m5 = 0;

    if (verbose)
    {
        ROS_WARN("%f", g_armInfo.positionDifference);
    }
        
    if (motorVelocitiesCmd.m1 > 0 && g_armInfo.positionDifference < MIN_DIFF)
    {
        if (verbose)
        {
            ROS_WARN("At Limit, go the other way");
        }        
        motorVelocitiesCmd.m1 = 0.0;
        motorVelocitiesCmd.m2 = 0.0;
    }

    if (motorVelocitiesCmd.m1 < 0 && g_armInfo.positionDifference > MAX_DIFF)
    {
        if (verbose)
        {
            ROS_WARN("At Limit, go the other way");
        }
        motorVelocitiesCmd.m1 = 0.0;
        motorVelocitiesCmd.m2 = 0.0;
    }
}
*/