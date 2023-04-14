#include "ArmUs_Info.hpp"

void ArmUsInfo::Vector5f::set(float f1, float f2, float f3, float f4, float f5)
{
    m1 = f1;
    m2 = f2;
    m3 = f3;
    m4 = f4;
    m5 = f5;
}

void ArmUsInfo::Vector5f::set(float f, int m)
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

std::vector<double> ArmUsInfo::Vector5f::get()
{
    return { m1, m2, m3, m4, m5 };
}

float ArmUsInfo::Vector5f::get(int m)
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

void ArmUsInfo::Vector5f::add(float f1, float f2, float f3, float f4, float f5)
{
    m1 += f1;
    m2 += f2;
    m3 += f3;
    m4 += f4;
    m5 += f5;
}

void ArmUsInfo::Vector5f::add(float f, int m)
{
    switch(m)
    {
    case 1:
        m1 += f;
        //m1 = CheckLimits(m1);
        break;
    case 2:
        m2 += f;
        //m2 = CheckLimits(m2);
        break;
    case 3:
        m3 += f;
        //m3 = CheckLimits(m3);
        break;
    case 4:
        m4 += f;
        //m4 = CheckLimits(m4);
        break;
    case 5:
        m5 += f;
        //m5 = CheckLimits(m5);
        break;
    }
}


void ArmUsInfo::Vector5f::print()
{
    ROS_INFO("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", m1, m2, m3, m4, m5);
}

void ArmUsInfo::Vector5b::set(bool b1, bool b2, bool b3, bool b4, bool b5)
{
    m1 = b1;
    m2 = b2;
    m3 = b3;
    m4 = b4;
    m5 = b5;   
}

std::vector<uint8_t> ArmUsInfo::Vector5b::get()
{
    return { m1, m2, m3, m4, m5 };
}

void ArmUsInfo::Vector3f::set(float xx, float yy, float zz)
{
    x = xx;
    y = yy;
    z = zz;
}

ArmUsInfo::ArmUsInfo(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service) : 
mf_call_inv_kin_calc_service(call_inv_kin_calc_service) 
{

};

/**
 * Calls the default Arm Us Info constructor with the parameter 
 */
ArmUsInfoSimul::ArmUsInfoSimul(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service) :
ArmUsInfo(call_inv_kin_calc_service)
{

}

/**
 * Simulation mode implementation
 */
void ArmUsInfoSimul::calculate_motor_velocities()
{
    // Joint mode
    if (MoveMode == MovementMode::Joint)
    {
        MotorVelocities.set(JointCommand, JointControlled);
    }
    
    // Cartesian mode
    else if (MoveMode == MovementMode::Cartesian)
    {
        Vector3f velocities = { 0.f, 0.f, 0.f }; // Joint velocities returned from the service
        int singularMatrix = 0; // Int that expresses if the service encountered a singular matrix

        // Call the function in Arm Us that calls the serive
        // Returns true if the service call was successful, false otherwise
        bool service_success = static_cast<bool>(mf_call_inv_kin_calc_service(velocities, singularMatrix));

        if (service_success  == false)
        {
            // ROS_ERROR("Inverse kinematic calculation service not called");
        }
        else
        {
            MotorVelocities.m1 = velocities.x;
            MotorVelocities.m2 = velocities.y;
            MotorVelocities.m3 = velocities.z;
        }
    }
}

/**
 * Calls the default Arm Us Info constructor with the parameter 
 */
ArmUsInfoReal::ArmUsInfoReal(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service) :
ArmUsInfo(call_inv_kin_calc_service)
{

}

/**
 * Real mode implementation
 */
void ArmUsInfoReal::calculate_motor_velocities()
{
    // Joint mode
    if (MoveMode == MovementMode::Joint)
    {
        // Joints 1 and 2 are controlled by a differential
        // A combination of the first two motors is needed to make the proper rotations

        // Both motors make the two big gears rotate in the same direction, making the arm go up or down
        // Negative sign is needed on motor 2 since motors are facing opposite directions in assembly
        if (JointControlled == 1)
        {
            MotorVelocities.m1 = JointCommand;
            MotorVelocities.m2 = -JointCommand;
        }

        // Both motors make the two big gears rotate in te opposite direction, making the arm go left or right
        // Negative sign is NOT needed on motor 2 since motors are facing opposite directions in assembly
        else if (JointControlled == 2)
        {
            MotorVelocities.m1 = JointCommand;
            MotorVelocities.m2 = JointCommand;
        }

        // Other joints are controlled by one motor each
        else
        {
            MotorVelocities.set(JointCommand, JointControlled);
        }
    }

    // Cartesian mode
    else if (MoveMode == MovementMode::Cartesian)
    {
        Vector3f velocities = { 0.f, 0.f, 0.f }; // Joint velocities returned from the service
        int singularMatrix = 0; // Int that expresses if the service encountered a singular matrix

        // Call the function in Arm Us that calls the serive
        // Returns true if the service call was successful, false otherwise
        bool service_success = static_cast<bool>(mf_call_inv_kin_calc_service(velocities, singularMatrix));

        if (service_success  == false)
        {
            // ROS_ERROR("Inverse kinematic calculation service not called");
        }
        else
        {
            MotorVelocities.m1 = (velocities.x + velocities.y) / 2;
            MotorVelocities.m2 = (-velocities.x + velocities.y) / 2;
            MotorVelocities.m3 = velocities.z;
        }
    }
};
