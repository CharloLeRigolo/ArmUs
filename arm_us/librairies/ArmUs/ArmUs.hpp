#pragma once

#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us_msg/GuiFeedback.h"
#include "arm_us_msg/GuiInfo.h"
#include "arm_us_msg/GraphInfo.h"

#include "arm_us_msg/InverseKinematicCalc.h"

#include "../ArmUs_Info/ArmUs_Info.hpp"
#include "../ArmUs_Controller/ArmUs_Controller.hpp"

const int ROS_RATE = 50;

bool verbose = true;

enum class ControlMode { Real = 0, Simulation = 1 };

class ArmUs
{
public:
    
    ArmUs();

    void Run();
    
    void Initalize();

private:

    void subControllerCallback(const sensor_msgs::Joy::ConstPtr &data);

    void sub_gui_callback(const arm_us_msg::GuiFeedback::ConstPtr &data);

    void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data);

    void setParams();

    void send_cmd_motor();

    void send_cmd_motor_stop();

    void send_gui_info();

    void send_3d_graph_info();

    bool call_inv_kin_calc_service(Vector3f &velocities, int &singularMatrix);

    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_input; // Controller
    ros::Subscriber m_sub_gui; // GUI
    // ros::Subscriber m_sub_joint_states;

    ros::Publisher m_pub_motor_interface; // Send motor velocities
    ros::Publisher m_pub_gui; // Send info to GUI
    ros::Publisher m_pub_3d_graph; // Send joint angles for real time representation of arm

    ros::ServiceClient m_client_inv_kin_calc; // Inverse kinematic calculation client

    Controller m_controller; // Object that olds information about the controller (Joysticks, buttons, triggers, bumpers)

    std::unique_ptr<ArmUsInfo> m_arm_us_info; // Object that holds information about the arm (Motor velocities)

    ControlMode m_controlMode; // Joint or Cartesian

    /********** Constantes **********/

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
