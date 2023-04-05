#pragma once

#include "ros/ros.h"

#include <vector>
#include <functional>

// #include "arm_us_msg/InverseKinematicCalc.h"

typedef bool (*func_inv_kin_calc_service_t)();

const float MIN_POS = 0; 
const float MAX_POS = 4095;

const float MAX_VEL = 1;
const float MIN_DIFF = -6.8;
const float MAX_DIFF = 5;

enum class MovementMode { Joint = 0, Cartesian = 1 };

struct Vector5f
{
    void set(float f1, float f2, float f3, float f4, float f5);
    void set(float f, int m);
    std::vector<double> get();
    float get(int m);
    void add(float f1, float f2, float f3, float f4, float f5);
    void add(float f, int m);
    void print();
    float CheckLimits(float m);

    float m1, m2, m3, m4, m5;
};

struct Vector5b
{
    void set(bool b1, bool b2, bool b3, bool b4, bool b5);
    std::vector<uint8_t> get();

    bool m1, m2, m3, m4, m5;
};

struct Vector4f
{
    void set(float xx, float yy, float zz, float aa);

    float x, y, z, a;
};

class ArmUsInfo
{
public:

    ArmUsInfo(std::function<bool(Vector4f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    virtual void calculate_motor_velocities() = 0;

    void calculate_joint_angles();

    float convert_motor_pos_to_deg(float current_pos, float min_input = 0, float max_input = 4095, float min_val = 0, float max_val = 360);

    bool call_inv_kin_calc_service();


    float JointCommand;
    Vector4f CartesianCommand;

    Vector5f MotorPositions;
    Vector5f MotorVelocities;
    Vector5f JointAngles;

    Vector5b MotorConnections;
    Vector5b MotorLimits;

    MovementMode MoveMode = MovementMode::Joint;
    
    int JointControlled = 1;

    float PositionDifference = 0.0f;

protected:
    std::function<bool(Vector4f &velocities, int &singularMatrix)> mf_call_inv_kin_calc_service;

};

class ArmUsInfoSimul : public ArmUsInfo
{
public:
    ArmUsInfoSimul(std::function<bool(Vector4f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    void calculate_motor_velocities();
};

class ArmUsInfoReal : public ArmUsInfo
{
public:
    ArmUsInfoReal(std::function<bool(Vector4f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    void calculate_motor_velocities();
};
