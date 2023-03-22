#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us/GuiFeedback.h"
#include "arm_us/GuiInfo.h"
#include "arm_us/GraphInfo.h"

#include <memory>

const float MIN_POS = 0; 
const float MAX_POS = 4095;

const float MAX_VEL = 4.8;
const float MIN_DIFF = -6.8;
const float MAX_DIFF = 5;

const int ROS_RATE = 50;

enum class ControlMode { Real = 0, Simulation = 1 };

bool verbose = true;
ControlMode controlMode = ControlMode::Simulation;

/******************** Controller ********************/

struct Joystick
{
    void set(float v, float h)
    {
        Vertical = v;
        Horizontal = h;
    }

    float Vertical;
    float Horizontal;
};

struct DirectionPad
{
    void set(int v, int h)
    {
        Vertical = v;
        Horizontal = h;
    }

    int Vertical;
    int Horizontal;
};

struct Trigger
{
    void set(int l, int r)
    {
        Left = l;
        Right = r;
    }

    int Left;
    int Right;
};

struct Button
{
    void set(int b1, int b2, int b3, int b4)
    {
        Button1 = b1;
        Button2 = b2;
        Button3 = b3;
        Button4 = b4;
    }

    int Button1; 
    int Button2; 
    int Button3; 
    int Button4; 
};

struct Controller
{
    void DisplayControllerInputs()
    {
        if (verbose)
        {
            ROS_WARN("----------------------------------------");
            ROS_WARN("Joy Left (H, V) = %f, %f", JoyLeft.Horizontal, JoyLeft.Vertical);
            ROS_WARN("Joy Right (H, V) = %f, %f", JoyRight.Horizontal, JoyRight.Vertical);
            ROS_WARN("Direction Pad (H, V) = %d, %d", Pad.Horizontal, Pad.Vertical);
            ROS_WARN("Buttons (1, 2, 3, 4) = %d, %d, %d, %d", Buttons.Button1, Buttons.Button2, Buttons.Button3, Buttons.Button4);
            ROS_WARN("Bumpers (L, R) = %d, %d", Bumpers.Left, Bumpers.Right);
            ROS_WARN("Triggers (L, R) = %d, %d", Triggers.Left, Triggers.Right);
        }
    }

    Joystick JoyLeft;
    Joystick JoyRight;
    DirectionPad Pad;
    Button Buttons;
    Trigger Bumpers;
    Trigger Triggers;
};

/******************** ArmUs Information ********************/

enum class MovementMode { Joint = 0, Cartesian = 1 };

struct Vector5f
{
    void set(float f1, float f2, float f3, float f4, float f5)
    {
        m1 = f1;
        m2 = f2;
        m3 = f3;
        m4 = f4;
        m5 = f5;
    }

    void set(float f, int m)
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

    std::vector<double> get()
    {
        return { m1, m2, m3, m4, m5 };
    }

    float get(int m)
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

    void add(float f, int m)
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

    void print()
    {
        ROS_WARN("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", m1, m2, m3, m4, m5);
    }

    float CheckLimits(float m)
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

    float m1, m2, m3, m4, m5;
};

struct Vector5b
{
    void set(bool b1, bool b2, bool b3, bool b4, bool b5)
    {
        m1 = b1;
        m2 = b2;
        m3 = b3;
        m4 = b4;
        m5 = b5;   
    }

    std::vector<uint8_t> get()
    {
        return { m1, m2, m3, m4, m5 };
    }

    bool m1, m2, m3, m4, m5;
};

struct Vector3f
{
    void set(float xx, float yy, float zz)
    {
        x = xx;
        y = yy;
        z = zz;
    }

    float x, y, z;
};

class ArmUsInfo
{
public:

    virtual void calculate_motor_velocities() = 0;

    void calculate_joint_angles()
    {
        JointAngles.m1 = convert_motor_pos_to_deg(MotorPositions.m1 + MotorPositions.m2);
        JointAngles.m2 = convert_motor_pos_to_deg(MotorPositions.m1 - MotorPositions.m2);
        JointAngles.m3 = convert_motor_pos_to_deg(MotorPositions.m3);
        JointAngles.m4 = convert_motor_pos_to_deg(MotorPositions.m4);
        JointAngles.m5 = convert_motor_pos_to_deg(MotorPositions.m5);

        //JointAngles.print();
    }

    float convert_motor_pos_to_deg(float current_pos, float min_input = 0, float max_input = 4095, float min_val = 0, float max_val = 360)
    {
        return ((current_pos / (max_input - min_input)) * (max_val - min_val)) + min_val;
    }

    float JointCommand;
    Vector3f CartesianCommand;

    Vector5f MotorPositions;
    Vector5f MotorVelocities;
    Vector5f JointAngles;

    Vector5b MotorConnections;
    Vector5b MotorLimits;

    MovementMode MoveMode = MovementMode::Joint;
    
    int JointControlled = 1;

    float PositionDifference = 0.0f;
};

class ArmUsInfoSimul : public ArmUsInfo
{
    void calculate_motor_velocities()
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

        }
    }
};

class ArmUsInfoReal : public ArmUsInfo
{
    void calculate_motor_velocities()
    {
        switch(JointControlled)
        {

        }
    }
};

/******************** ArmUs ********************/

class ArmUs
{
public:
    
    ArmUs(ControlMode controlMode) : m_controlMode(controlMode)
    {
        if (controlMode == ControlMode::Real)
        {
            m_arm_us_info = std::make_unique<ArmUsInfoReal>();
        }
        else if (controlMode == ControlMode::Simulation)
        {
            m_arm_us_info = std::make_unique<ArmUsInfoSimul>();
        }
    }

    void Run()
    {
        ros::Rate loop_rate(ROS_RATE);

        while (ros::ok())
        {
            m_arm_us_info->calculate_motor_velocities();
            m_arm_us_info->calculate_joint_angles();

            if (!ros::ok())
            {
                send_cmd_motor_stop();
            }
            else 
            {
                send_cmd_motor();
            }

            send_gui_info();
            send_3d_graph_info();

            ros::spinOnce();
            loop_rate.sleep();
        }

        send_cmd_motor_stop();
        ros::shutdown();
    }
    
    void Initalize()
    {
        m_sub_input =         m_nh.subscribe("joy", 1, &ArmUs::subControllerCallback, this);
        m_sub_gui =           m_nh.subscribe("gui_arm_us_chatter", 1, &ArmUs::sub_gui_callback, this);
        if (m_controlMode == ControlMode::Real)
        {
            m_sub_joint_states =  m_nh.subscribe("joint_states", 1, &ArmUs::sub_joint_states_callback, this);
        }
       
        m_pub_motor =         m_nh.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
        m_pub_gui =           m_nh.advertise<arm_us::GuiInfo>("gui_info", 10);
        m_pub_3d_graph =      m_nh.advertise<arm_us::GraphInfo>("graph_info", 10);

        setParams();
    }

private:

    void subControllerCallback(const sensor_msgs::Joy::ConstPtr &data)
    {
        
        m_controller.JoyLeft.set(data->axes[LEFT_JOY_VERT], data->axes[LEFT_JOY_HORI]);
        m_controller.JoyRight.set(data->axes[RIGHT_JOY_VERT], data->axes[RIGHT_JOY_HORI]);

        /*
        m_controller.Pad.set(data->axes[PAD_VERT], data->axes[PAD_HORI]);
        m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
        m_controller.Bumpers.set(data->buttons[LEFT_BUMP], data->buttons[RIGHT_BUMP]);
        m_controller.Triggers.set(data->buttons[LEFT_TRIG], data->buttons[RIGHT_TRIG]);
        m_controller.DisplayControllerInputs();
        */

        // Switch modes (cartesian and joint) with controller
        if (data->buttons[BUTTON_3] == 1 && m_controller.Buttons.Button3 == 0)
        {
            if (m_arm_us_info->MoveMode == MovementMode::Cartesian)
            {
                m_arm_us_info->MoveMode = MovementMode::Joint;
                ROS_WARN("Joint");
            }
            else 
            {
                m_arm_us_info->MoveMode = MovementMode::Cartesian;
                ROS_WARN("Cartesian");
            }
        }
        
        /********** Joint **********/
        if (m_arm_us_info->MoveMode == MovementMode::Joint)
        {
            // Change joint controlled with controller
            if (data->buttons[BUTTON_4] == 1 && m_controller.Buttons.Button4 == 0)
            {
                m_arm_us_info->JointControlled++;
                if (m_arm_us_info->JointControlled > 5)
                {
                    m_arm_us_info->JointControlled = 1;
                }
                ROS_WARN("Joint controlled : %d", m_arm_us_info->JointControlled);
            }

            if (data->buttons[BUTTON_2] == 1 && m_controller.Buttons.Button2 == 0)
            {
                m_arm_us_info->JointControlled--;
                if (m_arm_us_info->JointControlled < 1)
                {
                    m_arm_us_info->JointControlled = 5;
                }
                ROS_WARN("Joint controlled : %d", m_arm_us_info->JointControlled);
            }

            // Set speed of joint
            m_arm_us_info->JointCommand = m_controller.JoyLeft.Vertical * MAX_VEL;

        }
        /********** Cartesian **********/
        else if (m_arm_us_info->MoveMode == MovementMode::Cartesian)
        {

        }

        m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
    }

    void sub_gui_callback(const arm_us::GuiFeedback::ConstPtr &data)
    {
        if (data->joint)
        {
            m_arm_us_info->MoveMode = MovementMode::Joint;
        }
        else if (data->cartesian)
        {
            m_arm_us_info->MoveMode = MovementMode::Cartesian;
        }

        m_arm_us_info->JointControlled = data->joint_controlled;
    }

    void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data)
    {
        m_arm_us_info->MotorPositions.set(data->position[0], data->position[1], data->position[2], data->position[3], data->position[4]);
        m_arm_us_info->MotorVelocities.set(data->velocity[0], data->velocity[1], data->velocity[2], data->velocity[3], data->velocity[4]);

        m_arm_us_info->PositionDifference = data->position[0] - data->position[1];
    }

    void setParams()
    {   
        m_nh.getParam("/master_node/left_joy_hori", LEFT_JOY_HORI);
        m_nh.getParam("/master_node/left_joy_vert", LEFT_JOY_VERT);
        
        m_nh.getParam("/master_node/right_joy_hori", RIGHT_JOY_HORI);
        m_nh.getParam("/master_node/right_joy_vert", RIGHT_JOY_VERT);

        m_nh.getParam("/master_node/pad_hori", PAD_HORI);
        m_nh.getParam("/master_node/pad_vert", PAD_VERT);


        m_nh.getParam("/master_node/button_1", BUTTON_1);
        m_nh.getParam("/master_node/button_2", BUTTON_2);
        m_nh.getParam("/master_node/button_3", BUTTON_3);
        m_nh.getParam("/master_node/button_4", BUTTON_4);

        m_nh.getParam("/master_node/left_bump", LEFT_BUMP);
        m_nh.getParam("/master_node/right_bump", RIGHT_BUMP);

        m_nh.getParam("/master_node/left_trig", LEFT_TRIG);
        m_nh.getParam("/master_node/right_trig", RIGHT_TRIG);
    }

    void send_cmd_motor()
    {
        sensor_msgs::JointState msg;
        msg.name = { "motor1", "motor2" , "motor3", "motor4", "motor5" };
        msg.velocity = m_arm_us_info->MotorVelocities.get();
        m_pub_motor.publish(msg);
    }

    void send_cmd_motor_stop()
    {
        sensor_msgs::JointState msg;
        msg.name = { "motor1", "motor2", "motor3", "motor4", "motor5" };
        msg.velocity = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        m_pub_motor.publish(msg);
        if (verbose)
        {
            ROS_WARN("All motors stopped");
        }
    }

    void send_gui_info()
    {
        arm_us::GuiInfo msg;

        msg.position = m_arm_us_info->MotorPositions.get();
        msg.velocity = m_arm_us_info->MotorVelocities.get();
        msg.connected = m_arm_us_info->MotorConnections.get();
        msg.limit_reached = m_arm_us_info->MotorLimits.get();
        m_pub_gui.publish(msg);
    }

    void send_3d_graph_info()
    {
        arm_us::GraphInfo msg;
        msg.angle = m_arm_us_info->JointAngles.get();
        m_pub_3d_graph.publish(msg);
    }

    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_input;
    ros::Subscriber m_sub_gui;
    ros::Subscriber m_sub_joint_states;

    ros::Publisher m_pub_motor;
    ros::Publisher m_pub_gui;
    ros::Publisher m_pub_3d_graph;

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

/******************** Main ********************/

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