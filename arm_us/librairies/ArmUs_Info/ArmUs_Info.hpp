#pragma once

#include "ros/ros.h"

#include <vector>
#include <functional>

// #include "arm_us_msg/InverseKinematicCalc.h"

const float MAX_VEL = 1;

enum class MovementMode { Joint = 0, Cartesian = 1 };

/**
 * @brief Data structure that holds 5 floats : m1, m2, m3, m4, m5
 * 
 */
struct Vector5f
{
    void set(float f1, float f2, float f3, float f4, float f5);
    void set(float f, int m);
    std::vector<double> get();
    float get(int m);
    void add(float f1, float f2, float f3, float f4, float f5);
    void add(float f, int m);
    void print();
    // float CheckLimits(float m);

    float m1, m2, m3, m4, m5;
};

/**
 * @brief Data structure that holds 5 booleans : m1, m2, m3, m4, m5
 * 
 */
struct Vector5b
{
    void set(bool b1, bool b2, bool b3, bool b4, bool b5);
    std::vector<uint8_t> get();

    bool m1, m2, m3, m4, m5;
};

/**
 * @brief Data structure that holds 3 floats : x, y, z
 * 
 */
struct Vector3f
{
    void set(float xx, float yy, float zz);

    float x, y, z;
};

/**
 * @brief Class that holds information about the arm : velocities, angles, positions, etc
 * 
 */
class ArmUsInfo
{
public:

    ArmUsInfo(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    virtual void calculate_motor_velocities() = 0; // Different implementation if in real mode or simulation mode

    // Obsolete because of motor commands interface
    // void calculate_joint_angles(); 
    // float convert_motor_pos_to_deg(float current_pos, float min_input = 0, float max_input = 4095, float min_val = 0, float max_val = 360);

    // Command sent to motor when in joint mode (One motor moving at a time)
    float JointCommand; 
    // Command sent to motors when in cartesian mode (All 5 motors moving at a time)
    Vector3f CartesianCommand;

    // Vector5f MotorPositions; // Only useful in simulation mode
    Vector5f MotorVelocities; // Command sent to interface
    Vector5f JointAngles; // Current joint angles returned from interface

    // Vector5b MotorConnections;
    // Vector5b MotorLimits;

    MovementMode MoveMode = MovementMode::Cartesian;
    
    int JointControlled = 1;

    // float PositionDifference = 0.0f;

protected:
    // Function pointer to cal service from base class that contains node handler
    std::function<bool(Vector3f &velocities, int &singularMatrix)> mf_call_inv_kin_calc_service;
};

class ArmUsInfoSimul : public ArmUsInfo
{
public:
    ArmUsInfoSimul(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    void calculate_motor_velocities();
};

class ArmUsInfoReal : public ArmUsInfo
{
public:
    ArmUsInfoReal(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    void calculate_motor_velocities();
};
