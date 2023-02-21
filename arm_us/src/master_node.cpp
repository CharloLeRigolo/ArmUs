#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "arm_us/MotorControl.h"

#define MAX_VEL 4.8 //Linked to config/yaml file (global_max_vel )
#define MIN_DIFF -3 //-8.75
#define MAX_DIFF 3.00

#define ROS_RATE 50

void init();
void loop();
void subInputCallback(const sensor_msgs::Joy::ConstPtr &data);
void subGuiCallback(const arm_us::MotorControl::ConstPtr &data);
void sendCmdMotor(bool sendZeros = 0);
void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data);

float joystickLeftVert = 0.0f;
float joystickLeftSide = 0.0f;
float joystickRightVert = 0.0f;
float joystickRightSide = 0.0f;
ros::Subscriber sub_input;
ros::Subscriber sub_gui;

float position_difference = 0.0f;
struct MotorVelocity
ros::Subscriber sub_joint_states;
{
    float m1 = 0.0f;
    float m2 = 0.0f;
    float m3 = 0.0f;
    float m4 = 0.0f;
} motorVelocity;
ros::Publisher pub_motor;

const float maxMotorSpeed = 4.8f;

arm_us::MotorControl motors[2];

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
    pub_motor =         n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
    sub_gui = n.subscribe("gui_arm_us_chatter", 1, subGuiCallback);
}

void loop()
{
    ros::Rate loop_rate(ROS_RATE);

    while (ros::ok())
    {
        sendCmdMotor();
        ros::spinOnce();
        loop_rate.sleep();
    }

    sendCmdMotor(true);

    ros::shutdown();
}

void subInputCallback(const sensor_msgs::Joy::ConstPtr &data)
{   
    joystickLeftVert =  data->axes[1] * MAX_VEL;
    joystickLeftSide =  data->axes[0] * MAX_VEL;
    joystickRightVert = data->axes[3] * MAX_VEL;
    joystickRightSide = data->axes[2] * MAX_VEL;

}

void subGuiCallback(const arm_us::MotorControl::ConstPtr &data)
void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data)
{
    motors[data->motor_id].velocity = data->velocity;
}


    if (motorVelocity.m1 < 0 && position_difference > MAX_DIFF)
    {
        ROS_WARN("At Limit, go the other way");
        motorVelocity.m1 = 0.0;
        motorVelocity.m2 = 0.0;
    }

    msg.velocity = { motorVelocity.m1, motorVelocity.m2 };
    // ROS_WARN("%f | %f", msg.velocity[0], msg.velocity[1]);

    //ROS_WARN("Motor 1 : %f, Motor 2 : %f", motors[0].velocity, motors[1].velocity);

    pub_motor.publish(msg);
}
