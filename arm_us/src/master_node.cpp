#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

#define ROS_RATE 50

void init();
void loop();
void subInputCallback(const sensor_msgs::Joy::ConstPtr &data);
void print();
void sendCmdMotor();

float joystick_left_vert = 0.0f;
float joystick_right_vert = 0.0f;

ros::Subscriber sub_input;
ros::Publisher pub_motor;

int main(int argc, char *argv[])
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
    sub_input = n.subscribe("joy", 1, subInputCallback);
    pub_motor = n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
}

void loop()
{
    ros::Rate loop_rate(ROS_RATE);

    while (ros::ok())
    {
        print();
        sendCmdMotor();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/* Callback from joy_node (controller input)
TODO : Change axes[x] values to args in roscpp + launchfiles

arg: Gets pointer of /sensor_msgs/Joy.msg as "data"

ret: Nothing, global struct/class is updated with the new data
*/
void subInputCallback(const sensor_msgs::Joy::ConstPtr &data)
{
    joystick_left_vert = data->axes[1] * 5;
    joystick_right_vert = data->axes[4] * 5;
}

void print()
{
    if (joystick_left_vert != 0)
    {
        ROS_WARN_STREAM("Left joystick vertical : " << joystick_left_vert);
    }
    if (joystick_right_vert != 0)
    {
        ROS_WARN_STREAM("Right joystick vertical : " << joystick_right_vert);
    }
}

/* Builds and publish motor message of type : sensor_msgs::JointState */
void sendCmdMotor()
{
    sensor_msgs::JointState msg;

    msg.name = {"motor1", "motor2"};
    msg.velocity = {joystick_left_vert, joystick_right_vert};

    pub_motor.publish(msg);
}
