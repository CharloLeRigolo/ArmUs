#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

void init();
void loop();
void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data);
void print();
void sendCmdMotor();

float joystickLeftVert = 0.0f;
float joystickRightVert = 0.0f;

ros::Subscriber sub_input;
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
    sub_input = n.subscribe("joy", 1, sub_input_callback);
    pub_motor = n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
}

void loop()
{
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        print();
        sendCmdMotor();
    
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{   
    joystickLeftVert =  data->axes[1] * 5;
    joystickRightVert = data->axes[4] * 5;
}

void print()
{
    if (joystickLeftVert != 0)
    {
    	ROS_WARN_STREAM("Left joystick vertical : " << joystickLeftVert);
    }
    if (joystickRightVert != 0)
    {
        ROS_WARN_STREAM("Right joystick vertical : " << joystickRightVert);
    }
}

void sendCmdMotor()
{
    sensor_msgs::JointState msg;

    msg.name = { "motor1", "motor2" };
    msg.velocity = { joystickLeftVert, joystickRightVert };

    pub_motor.publish(msg);
}
