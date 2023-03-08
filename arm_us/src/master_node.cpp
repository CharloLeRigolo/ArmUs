#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#include "arm_us/MotorControl.h"
#include "arm_us/GuiFeedback.h"
#include "arm_us/GuiInfo.h"
#include "arm_us/GraphInfo.h"

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

enum class MovementMode { Joint = 0, Cartesian = 1 };

bool verbose = 0;

int LEFT_JOY_HORI;
int LEFT_JOY_VERT;
int LEFT_TRIG;
int RIGHT_JOY_HORI;
int RIGHT_JOY_VERT;
int RIGHT_TRIG;
int PAD_HORI;
int PAD_VERT;

int BUTTON_A;
int BUTTON_B;
int BUTTON_X;
int BUTTON_Y;
int LEFT_BUMP;
int RIGHT_BUMP;

ros::Subscriber sub_input;
ros::Subscriber sub_gui;
ros::Subscriber sub_joint_states;
ros::Publisher pub_motor;
ros::Publisher pub_gui;
ros::Publisher pub_3d_graph;

struct MotorVelocity
{
    float m1, m2, m3, m4, m5;
} motorVelocities;

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

    Joystick leftJoystick;
    Joystick rightJoystick;
    DirectionpPad directionPad;
    float leftTrigger;
    float rightTrigger;
    
    int buttonA;
    int buttonB;
    int buttonX;
    int buttonY;
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
    pub_3d_graph =      n.advertise<arm_us::GraphInfo>("3d_graph_info", 10);

    setParams(n);
}

void loop()
{
    ros::Rate loop_rate(ROS_RATE);

    while (ros::ok())
    {
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
    n.getParam("verbose", verbose);

    n.getParam("left_joy_hori", LEFT_JOY_HORI);
    n.getParam("left_joy_vert", LEFT_JOY_VERT);
    n.getParam("left_trig", LEFT_TRIG);
    n.getParam("right_joy_hori", RIGHT_JOY_HORI);
    n.getParam("right_joy_vert", RIGHT_JOY_VERT);
    n.getParam("right_trig", RIGHT_TRIG);
    n.getParam("pad_hori", PAD_HORI);
    n.getParam("pad_vert", PAD_VERT);

    n.getParam("button_a", BUTTON_A);
    n.getParam("button_b", BUTTON_B);
    n.getParam("button_x", BUTTON_X);
    n.getParam("button_y", BUTTON_Y);
    n.getParam("left_bump", LEFT_BUMP);
    n.getParam("right_bump", RIGHT_BUMP);
}

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{
    /*
    axes --> float32[]
    data->axes[0] --> Left joystick horizontal (1 left, -1 right)
    data->axes[1] --> Left joystick vertical (1 up, -1 down)
    data->axes[2] --> Left trigger (1 not pressed, -1 pressed)
    data->axes[3] --> Right joystick horizontal (1 left, -1 right)
    data->axes[4] --> Right joystick vertical (1 up, -1 down)
    data->axes[5] --> Right trigger (1 not pressed, -1 pressed)
    data->axes[6] --> Direction pad horizontal (1 left, -1 right, no in between)
    data->axes[7] --> Direction pad vertical (1 up, -1 down, no in between)

    buttons --> int32[]
    data->buttons[0] --> A button
    data->buttons[1] --> B button
    data->buttons[2] --> X button
    data->buttons[3] --> Y button
    data->buttons[4] --> Left bumper
    data->buttons[5] --> Right bumper
    */
    
    g_controller.leftJoystick.setJoystick(data->axes[LEFT_JOY_VERT], data->axes[LEFT_JOY_HORI]);
    g_controller.rightJoystick.setJoystick(data->axes[RIGHT_JOY_VERT], data->axes[RIGHT_JOY_HORI]);
    
    g_controller.leftTrigger = data->axes[LEFT_TRIG];
    g_controller.rightTrigger = data->axes[RIGHT_TRIG];

    g_controller.directionPad.setDirectionPad(data->axes[PAD_VERT], data->axes[PAD_HORI]);

    g_controller.buttonA = data->axes[BUTTON_A];
    g_controller.buttonB = data->axes[BUTTON_B];
    g_controller.buttonX = data->axes[BUTTON_X];
    g_controller.buttonY = data->axes[BUTTON_Y];

    g_controller.leftBumper = data->axes[LEFT_BUMP];
    g_controller.rightBumper = data->axes[RIGHT_BUMP];
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

    motorVelocities.m1 = (g_controller.leftJoystick.vertical + g_controller.rightJoystick.horizontal) / 2;
    motorVelocities.m2 = (g_controller.leftJoystick.vertical - g_controller.rightJoystick.horizontal) / 2;
    motorVelocities.m3 = 0;
    motorVelocities.m4 = 0;
    motorVelocities.m5 = 0;

    if (verbose)
    {
        ROS_WARN("%f", g_armInfo.positionDifference);
    }
        
    if (motorVelocities.m1 > 0 && g_armInfo.positionDifference < MIN_DIFF)
    {
        if (verbose)
        {
            ROS_WARN("At Limit, go the other way");
        }        
        motorVelocities.m1 = 0.0;
        motorVelocities.m2 = 0.0;
    }

    if (motorVelocities.m1 < 0 && g_armInfo.positionDifference > MAX_DIFF)
    {
        if (verbose)
        {
            ROS_WARN("At Limit, go the other way");
        }
        motorVelocities.m1 = 0.0;
        motorVelocities.m2 = 0.0;
    }

    msg.velocity = { motorVelocities.m1, motorVelocities.m2, motorVelocities.m3, motorVelocities.m4, motorVelocities.m5 };

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

    msg.position[0] = g_armInfo.motorPositions[0];
    msg.position[1] = g_armInfo.motorPositions[1];
    msg.position[2] = g_armInfo.motorPositions[2];
    msg.position[3] = g_armInfo.motorPositions[3];
    msg.position[4] = g_armInfo.motorPositions[4];
    
    msg.velocity[0] = g_armInfo.motorVelocities[0];
    msg.velocity[1] = g_armInfo.motorVelocities[1];
    msg.velocity[2] = g_armInfo.motorVelocities[2];
    msg.velocity[3] = g_armInfo.motorVelocities[3];
    msg.velocity[4] = g_armInfo.motorVelocities[4];

    msg.connected[0] = g_armInfo.motorConnected[0];
    msg.connected[1] = g_armInfo.motorConnected[1];
    msg.connected[2] = g_armInfo.motorConnected[2];
    msg.connected[3] = g_armInfo.motorConnected[3];
    msg.connected[4] = g_armInfo.motorConnected[4];

    msg.limit_reached[0] = g_armInfo.motorLimitReached[0];
    msg.limit_reached[1] = g_armInfo.motorLimitReached[1];
    msg.limit_reached[2] = g_armInfo.motorLimitReached[2];
    msg.limit_reached[3] = g_armInfo.motorLimitReached[3];
    msg.limit_reached[4] = g_armInfo.motorLimitReached[4];

    pub_gui.publish(msg);
}

void send_3d_graph_info()
{
    arm_us::GraphInfo msg;

    msg.angle[0] = 0;
    msg.angle[1] = 0;
    msg.angle[2] = 0;
    msg.angle[3] = 0;
    msg.angle[4] = 0;

    pub_3d_graph.publish(msg);
}