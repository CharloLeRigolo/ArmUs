#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us/GuiFeedback.h"
#include "arm_us/GuiInfo.h"
#include "arm_us/GraphInfo.h"



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

    std::vector<float> get()
    {
        return { m1, m2, m3, m4, m5 };
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

    std::vector<bool> get()
    {
        return { m1, m2, m3, m4, m5 };
    }

    bool m1, m2, m3, m4, m5;
};

struct ArmUsInfo
{
    Vector5f MotorPositions;
    Vector5f MotorVelocities;
    Vector5f JointAngles;

    Vector5b MotorConnections;
    Vector5b MotorLimits;

    MovementMode MoveMode = MovementMode::Cartesian;
    
    int JointControlled = 1;

    float PositionDifference = 0.0f;
};

/******************** ArmUs ********************/

class ArmUs
{
public:
    
    void Run()
    {
        ros::Rate loop_rate(ROS_RATE);

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

        ros::shutdown();
    }
    
    void Initalize()
    {
        m_sub_input =         m_nh.subscribe("joy", 1, &ArmUs::subControllerCallback, this);
        m_sub_joint_states =  m_nh.subscribe("joint_states", 1, &ArmUs::sub_joint_states_callback, this);
        m_sub_gui =           m_nh.subscribe("gui_arm_us_chatter", 1, &ArmUs::sub_gui_callback, this);

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
        m_controller.Pad.set(data->axes[PAD_VERT], data->axes[PAD_HORI]);
        m_controller.Buttons.set(data->buttons[BUTTON_1], data->buttons[BUTTON_2], data->buttons[BUTTON_3], data->buttons[BUTTON_4]);
        m_controller.Bumpers.set(data->buttons[LEFT_BUMP], data->buttons[RIGHT_BUMP]);
        m_controller.Triggers.set(data->buttons[LEFT_TRIG], data->buttons[RIGHT_TRIG]);

        m_controller.DisplayControllerInputs();
    }

    void sub_gui_callback(const arm_us::GuiFeedback::ConstPtr &data)
    {
        if (data->joint)
        {
            m_arm_us_info.MoveMode = MovementMode::Joint;
        }
        else if (data->cartesian)
        {
            m_arm_us_info.MoveMode = MovementMode::Cartesian;
        }

        m_arm_us_info.JointControlled = data->joint_controlled;
    }

    void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data)
    {
        m_arm_us_info.MotorPositions.set(data->position[0], data->position[1], data->position[2], data->position[3], data->position[4]);
        m_arm_us_info.MotorVelocities.set(data->velocity[0], data->velocity[1], data->velocity[2], data->velocity[3], data->velocity[4]);

        m_arm_us_info.PositionDifference = data->position[0] - data->position[1];
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

    ros::NodeHandle m_nh;

    ros::Subscriber m_sub_input;
    ros::Subscriber m_sub_gui;
    ros::Subscriber m_sub_joint_states;

    ros::Publisher m_pub_motor;
    ros::Publisher m_pub_gui;
    ros::Publisher m_pub_3d_graph;

    Controller m_controller;

    ArmUsInfo m_arm_us_info;

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

    const int MAX_VEL = 4.8;
    const int MIN_DIFF = -6.8;
    const int MAX_DIFF = 5;

    const int ROS_RATE = 50;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "master_node");

    ArmUs arm_us;

    arm_us.Initalize();
    arm_us.Run();

    return 0;
}

/*
void send_cmd_motor();
void send_cmd_motor_stop();
void send_gui_info();
void send_3d_graph_info();





void calculate_motor_velocities();
void calculate_joint_angles();

float convert_motor_pos_to_deg(float current_pos, float min_input = 0, float max_input = 4095, float min_val = 0, float max_val = 360);








void loop()
{
    ros::Rate loop_rate(ROS_RATE);
    while (ros::ok())
    {
        

        //calculate_motor_velocities();
        //calculate_joint_angles();

        if(!ros::ok())
        {
            send_cmd_motor_stop();
            if (verbose)
            {
                ROS_WARN("All motors stopped (Message from loop)");
            }
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
    ROS_WARN("All motors stopped (Ros not ok)");

    ros::shutdown();
}





void send_cmd_motor()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2" , "motor3", "motor4", "motor5" };
    msg.velocity = { motorVelocitiesCmd.m1, motorVelocitiesCmd.m2, motorVelocitiesCmd.m3, motorVelocitiesCmd.m4, motorVelocitiesCmd.m5 };
    pub_motor.publish(msg);
}

void send_cmd_motor_stop()
{
    sensor_msgs::JointState msg;
    msg.name = { "motor1", "motor2", "motor3", "motor4", "motor5" };
    msg.velocity = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    pub_motor.publish(msg);
    ROS_WARN("All motors stopped");
}

void send_gui_info()
{
    arm_us::GuiInfo msg;

    msg.position = { g_armInfo.motorPositions[0], g_armInfo.motorPositions[1], g_armInfo.motorPositions[2], g_armInfo.motorPositions[3], g_armInfo.motorPositions[4] };
    msg.velocity = { g_armInfo.motorVelocities[0], g_armInfo.motorVelocities[1], g_armInfo.motorVelocities[2], g_armInfo.motorVelocities[3], g_armInfo.motorVelocities[4] };
    msg.connected = { g_armInfo.motorConnected[0], g_armInfo.motorConnected[1], g_armInfo.motorConnected[2], g_armInfo.motorConnected[3], g_armInfo.motorConnected[4] };
    msg.limit_reached = { g_armInfo.motorLimitReached[0], g_armInfo.motorLimitReached[1], g_armInfo.motorLimitReached[2], g_armInfo.motorLimitReached[3], g_armInfo.motorLimitReached[4] };
    
    pub_gui.publish(msg);
}

void send_3d_graph_info()
{
    arm_us::GraphInfo msg;
    msg.angle = { jointAngles.j1, jointAngles.j2, jointAngles.j3, jointAngles.j4, jointAngles.j5 };
    pub_3d_graph.publish(msg);
}

float convert_motor_pos_to_deg(float current_pos, float min_input, float max_input, float min_val, float max_val)
{
    return ((current_pos / (max_input - min_input)) * (max_val - min_val)) + min_val;
}

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

void calculate_joint_angles()
{
    jointAngles.j1 = convert_motor_pos_to_deg(g_armInfo.motorPositions[0] + g_armInfo.motorPositions[1]);
    jointAngles.j2 = convert_motor_pos_to_deg(g_armInfo.motorPositions[0] - g_armInfo.motorPositions[1]);
    // ROS_WARN("j1 : %f, j2 : %f", jointAngles.j1, jointAngles.j2);
}
*/