#pragma once

#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us_msg/GuiInfo.h"
#include "arm_us_msg/GraphInfo.h"

#include "arm_us_msg/InverseKinematicCalc.h"

#include "../ArmUs_Info/ArmUs_Info.hpp"
#include "../ArmUs_Controller/ArmUs_Controller.hpp"

const int ROS_RATE = 50;

class ArmUs
{
    // Control mode
    enum class ControlMode 
    { 
      Real = 0, // Real mode
      Simulation = 1 // Simulation mode
    };

public:
    
    /**
     * @brief Construct a new Arm Us object. Contains Controller object and Arm Us Info object.
     */
    ArmUs();

    /**
     * @brief Main loop.
     */
    void Run();
    
private:

    /**
     * @brief Initialize publishers, subscribers and service. Get Rosparams to initialize internal constants.
     */
    void Initalize();

    /**
     * @brief Callback that receives information about the controller.
     * 
     * @param data Receives messages of type sensor_msgs::Joy that contains the axes measurements 
     * and the buttons measurements from a controller.
     */
    void subControllerCallback(const sensor_msgs::Joy::ConstPtr &data);

    /**
     * @brief Callback that receives the joint angles from the motor translator
     * 
     * @param data Receives message of type sensor_msgs::JointState in which data.position is the angle in degrees
     */
    void sub_join_angles_callback(const sensor_msgs::JointState::ConstPtr &data);
    
    /**
     * @brief Get all the Rosparams and set them in the corresponding variables and constants for later use
     */
    void setParams();

    /**
     * @brief Send the motor commands to the motor translator by publishing a message of type
     * sensor_msgs::JointState in which the name and velocity attributes are set.
     */
    void send_cmd_motor();

    /**
     * @brief Send a command of 0 for all motors to the motor translator.
     * Called before the node is shutdown because the dynamixel motors keep executing the last command received
     */
    void send_cmd_motor_stop();

    /**
     * @brief Send information to the dashboard by publishing a message of type ArmUsMsg::GuiInfo that contains
     * the current control mode and the current joint controlled
     */
    void send_gui_info();

    /**
     * @brief Send information to the 3d graph node that visualizes the position of the arm in real time in Rviz
     * Publishes a message of type ArmUsMsg::GraphInfo which contains the angles of each joints
     */
    void send_3d_graph_info();

    /**
     * @brief Call inverse kinematic calculation service to calculate velocities of first 3 motors to move in cartesian mode
     * 
     * @param velocities Motor velocities calculated for each joint by the service, is [0, 0, 0] if service call failed or singular matrix detected
     * @param singularMatrix 1 if singular matrix detected, 0 otherwise
     * @return true if call to service was successful and false if call to service was unsuccessful
     */
    bool call_inv_kin_calc_service(Vector3f &velocities, int &singularMatrix);

    ros::NodeHandle m_nh; // Node handle

    ros::Subscriber m_sub_input; // Subsribes to the controller
    ros::Subscriber m_sub_joint_angles; // Subscribes to the motor translator to get the joint angles
    
    ros::Publisher m_pub_motor_interface; // Publish motor (sensor_msgs::JointState) commands to the motor translator
    ros::Publisher m_pub_gui; // Publish information (ArmUsMsg::GuiInfo) to dashboard 
    ros::Publisher m_pub_3d_graph; // Publish joint angles (ArmUsMsg::GraphInfo) for real time representation of arm in Rviz

    ros::ServiceClient m_client_inv_kin_calc; // Inverse kinematic calculation client

    Controller m_controller; // Object that olds information about the controller (Joysticks, buttons, triggers, bumpers)

    std::unique_ptr<ArmUsInfo> m_arm_us_info; // Object that holds information about the arm (Motor commands, joint angles)

    ControlMode m_controlMode; // Current movement mode : Joint or Cartesian

    // Constants used to map the indexes of the controller to the right elements
    int LEFT_JOY_HORI;
    int LEFT_JOY_VERT;
    int LEFT_TRIG;
    int RIGHT_JOY_HORI;
    int RIGHT_JOY_VERT;
    int RIGHT_TRIG;
    int PAD_HORI;
    int PAD_VERT;

    int BUTTON_1;
    int BUTTON_2;
    int BUTTON_3;
    int BUTTON_4;
    int LEFT_BUMP;
    int RIGHT_BUMP;
};
