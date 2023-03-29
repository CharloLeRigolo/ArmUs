#include "ArmUs_Info.hpp"

void Vector5f::set(float f1, float f2, float f3, float f4, float f5)
{
    m1 = f1;
    m2 = f2;
    m3 = f3;
    m4 = f4;
    m5 = f5;
}

void Vector5f::set(float f, int m)
{
    switch(m)
    {
    case 1:
        m1 = f;
        break;
    case 2:
        m2 = f;
        break;
    case 3:
        m3 = f;
        break;
    case 4:
        m4 = f;
        break;
    case 5:
        m5 = f;
        break;
    }
}

std::vector<double> Vector5f::get()
{
    return { m1, m2, m3, m4, m5 };
}

float Vector5f::get(int m)
{
    switch(m)
    {
    case 1:
        return m1;
        break;
    case 2:
        return m2;
        break;
    case 3:
        return m3;
        break;
    case 4:
        return m4;
        break;
    case 5:
        return m5;
        break;
    default:
        return -1;
    }
}

void Vector5f::add(float f, int m)
{
    switch(m)
    {
    case 1:
        m1 += f;
        m1 = CheckLimits(m1);
        break;
    case 2:
        m2 += f;
        m2 = CheckLimits(m2);
        break;
    case 3:
        m3 += f;
        m3 = CheckLimits(m3);
        break;
    case 4:
        m4 += f;
        m4 = CheckLimits(m4);
        break;
    case 5:
        m5 += f;
        m5 = CheckLimits(m5);
        break;
    }
}

void Vector5f::print()
{
    ROS_INFO("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", m1, m2, m3, m4, m5);
}

float Vector5f::CheckLimits(float m)
{
    if (m > MAX_POS)
    {
        m -= MAX_POS;
    }
    else if (m <= MIN_POS)
    {
        m += MAX_POS;
    }
    return m;
}

void Vector5b::set(bool b1, bool b2, bool b3, bool b4, bool b5)
{
    m1 = b1;
    m2 = b2;
    m3 = b3;
    m4 = b4;
    m5 = b5;   
}

std::vector<uint8_t> Vector5b::get()
{
    return { m1, m2, m3, m4, m5 };
}

void Vector3f::set(float xx, float yy, float zz)
{
    x = xx;
    y = yy;
    z = zz;
}

ArmUsInfo::ArmUsInfo(ros::NodeHandle &nh) : m_nh(nh) 
{
    m_client_inv_kin_calc = m_nh.serviceClient<arm_us::InverseKinematicCalc>("inverse_kinematic_calc_service");
};

void ArmUsInfo::calculate_joint_angles()
{
    JointAngles.m1 = convert_motor_pos_to_deg(MotorPositions.m1);
    JointAngles.m2 = convert_motor_pos_to_deg(MotorPositions.m2);
    JointAngles.m3 = convert_motor_pos_to_deg(MotorPositions.m3);
    JointAngles.m4 = convert_motor_pos_to_deg(MotorPositions.m4);
    JointAngles.m5 = convert_motor_pos_to_deg(MotorPositions.m5);
    //JointAngles.print();
}

float ArmUsInfo::convert_motor_pos_to_deg(float current_pos, float min_input, float max_input, float min_val, float max_val)
{
    return ((current_pos / (max_input - min_input)) * (max_val - min_val)) + min_val;
}

ArmUsInfoSimul::ArmUsInfoSimul(ros::NodeHandle &nh) :
ArmUsInfo(nh)
{

}

void ArmUsInfoSimul::calculate_motor_velocities()
{
    if (MoveMode == MovementMode::Joint)
    {
        MotorVelocities.set(JointCommand, JointControlled);
        MotorPositions.add(JointCommand, JointControlled);
        //MotorVelocities.print();
        //MotorPositions.print();
    }
    else if (MoveMode == MovementMode::Cartesian)
    {
        ROS_INFO("Calling function pointer to service");
        call_inv_kin_calc_service;
        /*
        bool service_success = static_cast<bool>(mf_call_inv_kin_calc_service);
        if (service_success)
        {
            //ROS_INFO("Service called");
        }
        else 
        {
            //ROS_INFO("Error calling service");
        }
        */
    }
}

ArmUsInfoReal::ArmUsInfoReal(ros::NodeHandle &nh) :
ArmUsInfo(nh)
{

}

void ArmUsInfoReal::calculate_motor_velocities()
{
    if (MoveMode == MovementMode::Joint)
    {

    }
    else if (MoveMode == MovementMode::Cartesian)
    {

    }
};

bool ArmUsInfo::call_inv_kin_calc_service()
{
    ROS_INFO("Service function called");

    arm_us::InverseKinematicCalc srv;

    Vector5f angles = JointAngles;
    srv.request.angles =  { angles.m1, angles.m2, angles.m3, angles.m4, angles.m5 };

    srv.request.commands[0] = CartesianCommand.x;
    srv.request.commands[1] = CartesianCommand.y;
    srv.request.commands[2] = CartesianCommand.z;

    if (m_client_inv_kin_calc.call(srv))
    {
        if (srv.response.singularMatrix == false)
        {
            MotorPositions.set(srv.response.velocities[0], srv.response.velocities[1], srv.response.velocities[2], srv.response.velocities[3], srv.response.velocities[4]);
        }
        else 
        {
            ROS_INFO("Singular Matrix");
        }
        return true;
    }
    else 
    {
        ROS_ERROR("Failed to call service ");
        return false;
    }
}