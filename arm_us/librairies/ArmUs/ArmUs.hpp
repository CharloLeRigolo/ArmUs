#pragma once

#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us/GuiFeedback.h"
#include "arm_us/GuiInfo.h"
#include "arm_us/GraphInfo.h"

#include "arm_us/InverseKinematicCalc.h"

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

    void sub_gui_callback(const arm_us::GuiFeedback::ConstPtr &data);

    void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data);

    void setParams();

    void send_cmd_motor();

    void send_cmd_motor_stop();

    void send_gui_info();

    void send_3d_graph_info();

    bool call_inv_kin_calc_service(Vector4f &velocities, int &singularMatrix);

    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_input;
    ros::Subscriber m_sub_gui;
    ros::Subscriber m_sub_joint_states;

    ros::Publisher m_pub_motor;
    ros::Publisher m_pub_gui;
    ros::Publisher m_pub_3d_graph;

    ros::ServiceClient m_client_inv_kin_calc;

    Controller m_controller;

    std::unique_ptr<ArmUsInfo> m_arm_us_info;

    ControlMode m_controlMode;

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
