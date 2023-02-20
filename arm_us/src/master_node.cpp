#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#define MAX_VEL 4.8 //Linked to config/yaml file (global_max_vel )
#define MIN_DIFF -3 //-8.75
#define MAX_DIFF 3.00

void init();
void loop();
void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data);
void print();
void sendCmdMotor(bool sendZeros = 0);
void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data);

float joystickLeftVert = 0.0f;
float joystickLeftSide = 0.0f;
float joystickRightVert = 0.0f;
float joystickRightSide = 0.0f;

float position_difference = 0.0f;

struct MotorVelocity
{
    float m1 = 0.0f;
    float m2 = 0.0f;
    float m3 = 0.0f;
    float m4 = 0.0f;
} motorVelocity;

ros::Subscriber sub_input;
ros::Subscriber sub_joint_states;
ros::Publisher pub_motor;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "master_node");

    init();
    loop();

    return 0;
}

void init()
{
    ros::NodeHandle n;
    sub_input =         n.subscribe("joy", 1, sub_input_callback);
    sub_joint_states =  n.subscribe("joint_states", 1, sub_joint_states_callback);
    pub_motor =         n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);

}

void loop()
{
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        sendCmdMotor();
        ros::spinOnce();
        loop_rate.sleep();
    }

    sendCmdMotor(true);

    ros::shutdown();
}

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{   
    joystickLeftVert =  data->axes[1] * MAX_VEL;
    joystickLeftSide =  data->axes[0] * MAX_VEL;
    joystickRightVert = data->axes[3] * MAX_VEL;
    joystickRightSide = data->axes[2] * MAX_VEL;

}

void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &data)
{
    position_difference = data->position[0] - data->position[1];
}

void sendCmdMotor(bool sendZeros)
{    sensor_msgs::JointState msg;

    msg.name = { "motor1", "motor2" };

    if (sendZeros)
    {
        msg.velocity = { 0.0, 0.0 };
        pub_motor.publish(msg);
        ROS_WARN("Zeros sent");
        return;
    }

    motorVelocity.m1 = (joystickLeftVert + joystickRightSide)/2;
    motorVelocity.m2 = (joystickLeftVert - joystickRightSide)/2;

    if (motorVelocity.m1 > 0 && position_difference < MIN_DIFF)
    {
        ROS_WARN("At Limit, go the other way");
        motorVelocity.m1 = 0.0;
        motorVelocity.m2 = 0.0;
    }

    if (motorVelocity.m1 < 0 && position_difference > MAX_DIFF)
    {
        ROS_WARN("At Limit, go the other way");
        motorVelocity.m1 = 0.0;
        motorVelocity.m2 = 0.0;
    }

    msg.velocity = { motorVelocity.m1, motorVelocity.m2 };
    // ROS_WARN("%f | %f", msg.velocity[0], msg.velocity[1]);

    pub_motor.publish(msg);
}
