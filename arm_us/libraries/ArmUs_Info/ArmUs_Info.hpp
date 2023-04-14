/**
 * @file ArmUs_Info.hpp
 * @author Mikael St-Arnaud et Philippe Michaud (stam1001, micp1402)
 * @brief This class is used to calculate the commands sent to the motor translator to make the arm move
 * @version 0.1 
 * @date 2023-04-13
 *
 * @copyright Copyright (c) 2023 - See ARM_US licence
 */


#pragma once

#include "ros/ros.h"

#include <vector>
#include <functional>

const float MAX_VEL = 1; // Maximum velocity of the motors #TODO: Instead of using this local constant, get max velocity from rosparams

/**
 * @brief Class that controls the arm.
 * Contains a virtual void function, cannot be instantiated.
 * Instead, instantiate either Arm Us Info Real or Arm Us Info Simul
 * depending on the control mode.
 * The two derived classes exist because the implementation of the controls 
 * are different in each one.
 */
class ArmUsInfo
{
public:

    /**
     * @brief Movement mode of the arm
     */
    enum class MovementMode 
    { 
        Joint = 0, // Move joint by joint
        Cartesian = 1 // Move in cartesian coordinates (X, Y, Z)
    };

     /********** Helper data structures **********/

    /**
     * @brief Data structure that holds 5 floats : m1, m2, m3, m4, m5
     * Used for motor positions, angles, velocities
     */
    struct Vector5f
    {
        /**
         * @brief Set all five float values at the same time
         * 
         * @param f1 Float value of m1
         * @param f2 Float value of m2
         * @param f3 Float value of m3
         * @param f4 Float value of m4
         * @param f5 Float value of m5
         */
        void set(float f1, float f2, float f3, float f4, float f5);

        /**
         * @brief Set one float value at a specified index
         * 
         * @param f Float value to be set
         * @param m Index of the float to be set (Int value between 1 and 5)
         */
        void set(float f, int m);

        /**
         * @brief Get all five float values in order
         * 
         * @return std::vector<double> of the five values
         */
        std::vector<double> get();

        /**
         * @brief Get one float value at a specified index
         * 
         * @param m Index of the float wanted (Int value between 1 and 5)
         * @return float value at index m
         */
        float get(int m);

        /**
         * @brief Add the float values in parameters to the current values
         * 
         * @param f1 Float value f1 to be added to current value m1
         * @param f2 Float value f2 to be added to current value m2
         * @param f3 Float value f3 to be added to current value m3
         * @param f4 Float value f4 to be added to current value m4
         * @param f5 Float value f5 to be added to current value m5
         */
        void add(float f1, float f2, float f3, float f4, float f5);

        /**
         * @brief Add one float value to the current value specified by index
         * 
         * @param f Float value f added to m specified by index
         * @param m Index value (Int between 1 and 5)
         */
        void add(float f, int m);

        /**
         * @brief Print all five values in the terminal
         */
        void print();

        float m1 = 0.0f; // Float value m1 initialized at 0.0
        float m2 = 0.0f; // Float value m2 initialized at 0.0
        float m3 = 0.0f; // Float value m3 initialized at 0.0
        float m4 = 0.0f; // Float value m4 initialized at 0.0
        float m5 = 0.0f; // Float value m5 initialized at 0.0
    };

    /**
     * @brief Data structure that holds 5 booleans : m1, m2, m3, m4, m5
     * Used for joint limits of the arm
     */
    struct Vector5b
    {
        /**
         * @brief Set all five boolean values at the same time
         * 
         * @param b1 Boolean value of m1
         * @param b2 Boolean value of m2
         * @param b3 Boolean value of m3
         * @param b4 Boolean value of m4
         * @param b5 Boolean value of m5
         */
        void set(bool b1, bool b2, bool b3, bool b4, bool b5);

        /**
         * @brief Get all five boolean values in order
         * 
         * @return std::vector<uint8_t> of the five values
         */
        std::vector<uint8_t> get();

        bool m1 = false; // Boolean value m1 initialized at "false" 
        bool m2 = false; // Boolean value m2 initialized at "false" 
        bool m3 = false; // Boolean value m3 initialized at "false" 
        bool m4 = false; // Boolean value m4 initialized at "false" 
        bool m5 = false; // Boolean value m5 initialized at "false" 
    };

    /**
     * @brief Data structure that holds 3 floats : x, y, z
     * Used for the cartesian velocity commands sent and the joint velocities returned
     */
    struct Vector3f
    {
        /**
         * @brief Set all three float values at the same time
         * 
         * @param xx Float value of x
         * @param yy Float value of y
         * @param zz Float value of z
         */
        void set(float xx, float yy, float zz);

        float x = 0.0f; // Float value x initalized at 0.0
        float y = 0.0f; // Float value y initalized at 0.0
        float z = 0.0f; // Float value z initalized at 0.0
    };

    /**
     * @brief Construct a new Arm Us Info object
     * 
     * @param call_inv_kin_calc_service Function pointer that points to the function that calls the service in Arm Us::call_inv_kin_calc_service()
     */
    ArmUsInfo(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    /**
     * @brief Virtual void function that calculates the motor commands that are sent to the motor translator
     * The implementation is different in each derived class, depending on the control mode chosen when launching the node
     */
    virtual void calculate_motor_velocities() = 0;

    float JointCommand; // Command in joint mode (One motor moving at a time)

    Vector3f CartesianCommand; // Command in cartesian mode (First three motors moving at the same time)

    Vector5f MotorVelocities; // Command sent to the motor translator

    Vector5f JointAngles; // Current joint angles (published from the motor translator)

    MovementMode MoveMode = MovementMode::Joint; // Current movement mode (Initialized in joint mode)
    
    int JointControlled = 1; // Current joint controlled (Initialized at 1, values between 1 and 5)

protected:
    // Function pointer to call service from Arm Us that contains a node handle
    std::function<bool(Vector3f &velocities, int &singularMatrix)> mf_call_inv_kin_calc_service;
};

/**
 * @brief Derived class from Arm Us Info
 * Used to simulate the motors to debug
 */
class ArmUsInfoSimul : public ArmUsInfo
{
public:
    /**
     * @brief Construct a new Arm Us Info Simul object
     * 
     * @param call_inv_kin_calc_service Function pointer that points to the function that calls the service in Arm Us::call_inv_kin_calc_service()
     */
    ArmUsInfoSimul(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

    /**
     * @brief Calculate the motor commands to send to the motor translator
     * Implementation to work in simulation mode
     */
    void calculate_motor_velocities();
};

/**
 * @brief Derived class from Arm Us Info
 * Used to control the arm in real time with Dynamixels connected
 */
class ArmUsInfoReal : public ArmUsInfo
{
public:
    /**
     * @brief Construct a new Arm Us Info Real object
     * 
     * @param call_inv_kin_calc_service Function pointer that points to the function that calls the service in Arm Us::call_inv_kin_calc_service()
     */
    ArmUsInfoReal(std::function<bool(Vector3f &velocities, int &singularMatrix)> call_inv_kin_calc_service);

     /**
     * @brief Calculate the motor commands to send to the motor translator
     * Implementation to work when Dynamixels are connected
     */
    void calculate_motor_velocities();
};
