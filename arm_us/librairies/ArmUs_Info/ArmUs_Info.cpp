#include "ArmUs_Info.hpp"

/**
 * @brief Set all five floats at the same time
 * 
 * @param f1 m1
 * @param f2 m2
 * @param f3 m3
 * @param f4 m4
 * @param f5 m5
 */
void Vector5f::set(float f1, float f2, float f3, float f4, float f5)
{
    m1 = f1;
    m2 = f2;
    m3 = f3;
    m4 = f4;
    m5 = f5;
}

/**
 * @brief Set one float with value "f" at index "m"
 * 
 * @param f Set value of float f
 * @param m Index between 1 and 5
 */
void Vector5f::set(float f, int m)
{
    // Set all other floats to 0
    set(0.f, 0.f, 0.f, 0.f, 0.f);

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

/**
 * @brief Return all 5 floats as vector of doubles
 * 
 * @return std::vector<double> 
 */
std::vector<double> Vector5f::get()
{
    return { m1, m2, m3, m4, m5 };
}

/**
 * @brief Get float at index "m"
 * 
 * @param m Index between 1 and 5
 * @return float 
 */
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

/**
 * @brief Add 5 float values to existing float values
 * 
 * @param f1 + m1 
 * @param f2 + m2
 * @param f3 + m3
 * @param f4 + m4
 * @param f5 + m5
 */
void Vector5f::add(float f1, float f2, float f3, float f4, float f5)
{
    m1 += f1;
    m2 += f2;
    m3 += f3;
    m4 += f4;
    m5 += f5;
}

/**
 * @brief Add float to index
 * 
 * @param f Value to be added to existing float
 * @param m Index of float to be added, between 1 and 5
 */
void Vector5f::add(float f, int m)
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

bool Vector5f::checkIfNull()
{
    if (m1 == 0.0 && m2 == 0.0 && m3 == 0.0 && m4 == 0.0 && m5 == 0.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Print all 5 floats on one line
 * 
 */
void Vector5f::print()
{
    ROS_INFO("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", m1, m2, m3, m4, m5);
}

/**
 * @brief Set all five booleans at the same time
 * 
 * @param b1 m1
 * @param b2 m2
 * @param b3 m3
 * @param b4 m4
 * @param b5 m5
 */
void Vector5b::set(bool b1, bool b2, bool b3, bool b4, bool b5)
{
    m1 = b1;
    m2 = b2;
    m3 = b3;
    m4 = b4;
    m5 = b5;   
}

/**
 * @brief Return all 5 booleans as vector of uint8_t
 * 
 * @return std::vector<uint8_t> 
 */
std::vector<uint8_t> Vector5b::get()
{
    return { m1, m2, m3, m4, m5 };
}

/**
 * @brief Set all 3 floats at the same time
 * 
 * @param xx x
 * @param yy y
 * @param zz z
 */
void Vector3f::set(float xx, float yy, float zz)
{
    x = xx;
    y = yy;
    z = zz;
}

ArmUsInfo::ArmUsInfo(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service) : mf_call_inv_kin_calc_service(call_inv_kin_calc_service) 
{

};

// void ArmUsInfo::calculate_joint_angles()
// {
//     JointAngles.m1 = convert_motor_pos_to_deg(MotorPositions.m1);
//     JointAngles.m2 = convert_motor_pos_to_deg(MotorPositions.m2);
//     JointAngles.m3 = convert_motor_pos_to_deg(MotorPositions.m3);
//     JointAngles.m4 = convert_motor_pos_to_deg(MotorPositions.m4);
//     JointAngles.m5 = convert_motor_pos_to_deg(MotorPositions.m5);
//     //JointAngles.print();
// }

// float ArmUsInfo::convert_motor_pos_to_deg(float current_pos, float min_input, float max_input, float min_val, float max_val)
// {
//     return ((current_pos / (max_input - min_input)) * (max_val - min_val)) + min_val;
// }

ArmUsInfoSimul::ArmUsInfoSimul(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service) :
ArmUsInfo(call_inv_kin_calc_service)
{

}

/**
 * @brief Calculate motor velocities to send to interface when in simulation mode
 * 
 */
void ArmUsInfoSimul::calculate_motor_velocities()
{
    // Joint mode
    if (MoveMode == MovementMode::Joint)
    {
        MotorVelocities.set(JointCommand, JointControlled);
        // if (!MotorVelocities.checkIfNull())
        // {
        //     ROS_INFO("Joint commands :");
        //     MotorVelocities.print();
        // }
    }
    
    // Cartesian mode
    else if (MoveMode == MovementMode::Cartesian)
    {
        Vector3f velocities = { 0.f, 0.f, 0.f };
        int singularMatrix = 0;
        bool service_success = static_cast<bool>(mf_call_inv_kin_calc_service(velocities, singularMatrix));
        if (service_success  == false)
        {
            ROS_ERROR("Inverse kinematic calculation service not called");
        }

        // ROS_INFO("--------------------------------------------------");
        // ROS_INFO("Velocities: %f, %f, %f, %f", velocities.x, velocities.y, velocities.z, velocities.a);
        // ROS_INFO("Singular matrix: %d", singularMatrix);
        // ROS_INFO("--------------------------------------------------");

        MotorVelocities.m1 = velocities.x;
        MotorVelocities.m2 = velocities.y;
        MotorVelocities.m3 = velocities.z;
        // MotorPositions.add(velocities.x, velocities.y, velocities.z, 0, 0);
    }
}

ArmUsInfoReal::ArmUsInfoReal(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service) :
ArmUsInfo(call_inv_kin_calc_service)
{

}

void ArmUsInfoReal::calculate_motor_velocities()
{
    // Joint mode
    if (MoveMode == MovementMode::Joint)
    {
        if (JointControlled == 1)
        {
            MotorVelocities.m1 = JointCommand;
            MotorVelocities.m2 = -JointCommand;
        }
        else if (JointControlled == 2)
        {
            MotorVelocities.m1 = JointCommand;
            MotorVelocities.m2 = JointCommand;
        }
        else
        {
            MotorVelocities.set(JointCommand, JointControlled);
        }

        // if (!MotorVelocities.checkIfNull())
        // {
        //     ROS_INFO("Joint commands :");
        //     MotorVelocities.print();
        // }
    }

    // Cartesian mode
    else if (MoveMode == MovementMode::Cartesian)
    {
        Vector3f velocities = { 0.f, 0.f, 0.f };
        int singularMatrix = 0;
        
        // Call inverse kinematic calculation service, return velocities of first 3 motors and if matrix is singular
        bool service_success = static_cast<bool>(mf_call_inv_kin_calc_service(velocities, singularMatrix));
        if (service_success  == false)
        {
            ROS_ERROR("Inverse kinematic calculation service not called");
        }

        // ROS_INFO("--------------------------------------------------");
        // ROS_INFO("Velocities: %f, %f, %f, %f", velocities.x, velocities.y, velocities.z, velocities.a);
        // ROS_INFO("Singular matrix: %d", singularMatrix);
        // ROS_INFO("--------------------------------------------------");

        MotorVelocities.m1 = (velocities.x + velocities.y) / 2;
        MotorVelocities.m2 = (velocities.x - velocities.y) / 2;
        MotorVelocities.m3 = velocities.z;
    }
};