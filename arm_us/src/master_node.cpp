#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us/MotorControl.h"
#include "arm_us/GuiFeedback.h"
#include "arm_us/GuiInfo.h"
#include "arm_us/GraphInfo.h"

#include <string.h>

#define MAX_VEL 4.8 // Linked to config/yaml file (global_max_vel )
#define MIN_DIFF -6.8
#define MAX_DIFF 5

#define ROS_RATE 50

void init();
void loop();

void setParams(ros::NodeHandle &n);

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data);
void sub_gui_callback(const arm_us::GuiFeedback::ConstPtr &data);
void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data);
void send_cmd_motor();
void send_cmd_motor_stop();
void send_gui_info();
void send_3d_graph_info();

void calculate_motor_velocities();
void calculate_joint_angles();

float convert_motor_pos_to_deg(float current_pos, float min_input = 0, float max_input = 4095, float min_val = 0, float max_val = 360);

enum class MovementMode { Joint = 0, Cartesian = 1 };

bool verbose = 1;

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

ros::Subscriber sub_input;
ros::Subscriber sub_gui;
ros::Subscriber sub_joint_states;
ros::Publisher pub_motor;
ros::Publisher pub_gui;
ros::Publisher pub_3d_graph;

struct MotorVelocityCmd
{
    float m1, m2, m3, m4, m5;
} motorVelocitiesCmd;

struct JointAngle
{
    float j1, j2, j3, j4, j5;
} jointAngles;

class ArmInfo
{
public:

    float motorPositions[5] = {};
    float motorVelocities[5] = {};
    bool motorConnected[5] = {};
    bool motorLimitReached[5] = {};

    float positionDifference = 0.0f;

    MovementMode movementMode = MovementMode::Cartesian;
    int currentControlledJointMotor = 1;
};

class Controller
{
public:

    struct Joystick
    {
        void setJoystick(float v, float h)
        {
            vertical = v;
            horizontal = h;
        }
        float vertical;
        float horizontal;
    };

    struct DirectionpPad
    {   
        void setDirectionPad(int v, int h)
        {
            vertical = v;
            horizontal = h;
        }
        int vertical;
        int horizontal;
    };

    void DisplayControllerInputs()
    {
        if (verbose)
        {
            ROS_WARN("Left joy hori = %f, Left joy vert = %f", leftJoystick.horizontal, leftJoystick.vertical);
            ROS_WARN("Right joy hori = %f, Right joy vert = %f", rightJoystick.horizontal, rightJoystick.vertical);
            ROS_WARN("Left trigger = %d, Right trigger = %d", leftTrigger, rightTrigger);
            ROS_WARN("Hori pad = %d, Vert pad = %d", directionPad.horizontal, directionPad.vertical);
            ROS_WARN("Button 1 = %d, Button 2 = %d", button1, button2);
            ROS_WARN("Button 3 = %d, Button 4 = %d", button3, button4);
            ROS_WARN("Left bumper = %d, Right bumper = %d", leftBumper, rightBumper);
            ROS_WARN("--------------------------------------------------------------------------------------------------------------");
        }
    }

    Joystick leftJoystick;
    Joystick rightJoystick;
    DirectionpPad directionPad;

    int leftTrigger;
    int rightTrigger;
    int button1;
    int button2;
    int button3;
    int button4;
    int leftBumper;
    int rightBumper;
};

Controller g_controller;

ArmInfo g_armInfo;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "master_node");

    init();
    loop();

    return 0;
}

/* Code intialisation
    - Starts NodeHandle
    - Initialise joy subscriber and publisher to dynamixel topic
*/
void init()
{
    ros::NodeHandle n;
    sub_input =         n.subscribe("joy", 1, sub_input_callback);
    sub_joint_states =  n.subscribe("joint_states", 1, sub_joint_states_callback);
    sub_gui =           n.subscribe("gui_arm_us_chatter", 1, sub_gui_callback);

    pub_motor =         n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
    pub_gui =           n.advertise<arm_us::GuiInfo>("gui_info", 10);
    pub_3d_graph =      n.advertise<arm_us::GraphInfo>("graph_info", 10);

    setParams(n);
}

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

void setParams(ros::NodeHandle &n)
{   
    XmlRpc::XmlRpcValue controller;
    n.getParam("/master_node/controller", controller);
    
    n.getParam("/master_node/left_joy_hori", LEFT_JOY_HORI);
    n.getParam("/master_node/left_joy_vert", LEFT_JOY_VERT);
    n.getParam("/master_node/left_trig", LEFT_TRIG);
    n.getParam("/master_node/right_joy_hori", RIGHT_JOY_HORI);
    n.getParam("/master_node/right_joy_vert", RIGHT_JOY_VERT);
    n.getParam("/master_node/right_trig", RIGHT_TRIG);
    n.getParam("/master_node/pad_hori", PAD_HORI);
    n.getParam("/master_node/pad_vert", PAD_VERT);

    n.getParam("/master_node/button_1", BUTTON_1);
    n.getParam("/master_node/button_2", BUTTON_2);
    n.getParam("/master_node/button_3", BUTTON_3);
    n.getParam("/master_node/button_4", BUTTON_4);
    n.getParam("/master_node/left_bump", LEFT_BUMP);
    n.getParam("/master_node/right_bump", RIGHT_BUMP);

    if (verbose)
    {   
        ROS_WARN("LOADING CONTROLLER CONFIG .YAML FILE");

        ROS_WARN("LEFT_JOY_HORI = %d", LEFT_JOY_HORI);
        ROS_WARN("LEFT_JOY_VERT = %d", LEFT_JOY_VERT);
        ROS_WARN("LEFT_TRIG = %d", LEFT_TRIG);
        ROS_WARN("RIGHT_JOY_HORI = %d", RIGHT_JOY_HORI);
        ROS_WARN("RIGHT_JOY_VERT = %d", RIGHT_JOY_VERT);
        ROS_WARN("RIGHT_TRIG = %d", RIGHT_TRIG);
        ROS_WARN("PAD_HORI = %d", PAD_HORI);
        ROS_WARN("PAD_VERT = %d", PAD_VERT);

        ROS_WARN("BUTTON_1 = %d", BUTTON_1);
        ROS_WARN("BUTTON_2 = %d", BUTTON_2);
        ROS_WARN("BUTTON_3 = %d", BUTTON_3);
        ROS_WARN("BUTTON_4 = %d", BUTTON_4);
        ROS_WARN("LEFT_BUMP = %d", LEFT_BUMP);
        ROS_WARN("RIGHT_BUMP = %d", RIGHT_BUMP);
    }
}

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{
    g_controller.leftJoystick.setJoystick(data->axes[LEFT_JOY_VERT], data->axes[LEFT_JOY_HORI]);
    g_controller.rightJoystick.setJoystick(data->axes[RIGHT_JOY_VERT], data->axes[RIGHT_JOY_HORI]);
    g_controller.directionPad.setDirectionPad(data->axes[PAD_VERT], data->axes[PAD_HORI]);

    g_controller.button1 = data->buttons[BUTTON_1];
    g_controller.button2 = data->buttons[BUTTON_2];
    g_controller.button3 = data->buttons[BUTTON_3];
    g_controller.button4 = data->buttons[BUTTON_4];

    g_controller.leftBumper = data->buttons[LEFT_BUMP];
    g_controller.rightBumper = data->buttons[RIGHT_BUMP];

    g_controller.leftTrigger = data->buttons[LEFT_TRIG];
    g_controller.rightTrigger = data->buttons[RIGHT_TRIG];

    g_controller.DisplayControllerInputs();

    // ROS_WARN("%f, %f", g_controller.leftJoystick.horizontal, g_controller.leftJoystick.vertical);
}

void sub_gui_callback(const arm_us::GuiFeedback::ConstPtr &data)
{
    if (data->joint)
    {
        g_armInfo.movementMode = MovementMode::Joint;
        if (verbose)
        {
            ROS_WARN("Current movement mode : joint");
        }
    }
    else if (data->cartesian)
    {
        g_armInfo.movementMode = MovementMode::Cartesian;
        if (verbose)
        {
            ROS_WARN("Current movement mode : cartesian");
        }
    }

    g_armInfo.currentControlledJointMotor = data->current_controlled_joint;
    if (verbose && g_armInfo.movementMode == MovementMode::Joint)
    {
        ROS_WARN("Current joint controlled : %d", g_armInfo.currentControlledJointMotor);
    }
}

void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data)
{
    g_armInfo.motorPositions[0] = data->position[0];
    g_armInfo.motorPositions[1] = data->position[1];
    g_armInfo.motorPositions[2] = data->position[2];
    g_armInfo.motorPositions[3] = data->position[3];
    g_armInfo.motorPositions[4] = data->position[4];

    g_armInfo.motorVelocities[0] = data->velocity[0];
    g_armInfo.motorVelocities[1] = data->velocity[1];
    g_armInfo.motorVelocities[2] = data->velocity[2];
    g_armInfo.motorVelocities[3] = data->velocity[3];
    g_armInfo.motorVelocities[4] = data->velocity[4];

    g_armInfo.positionDifference = data->position[0] - data->position[1];
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