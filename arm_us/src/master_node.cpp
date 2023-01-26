#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"

void init();
void loop();
void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data);
void update_btn1();
void sendCmdMotor();

bool btn1 = false;
float axe1 = 0.0;

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
        update_btn1();
        sendCmdMotor();
    
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{
    btn1 = data->buttons[0];
    axe1 = data->axes[2];

    axe1 = abs((axe1-1)/2 * 5);
}

void update_btn1()
{
    if (btn1)
    {
        ROS_WARN("Button 1 pressed");
    }
}

void sendCmdMotor()
{
    sensor_msgs::JointState msg;

    msg.name = {"motor1", "motor2"};
    msg.velocity = {axe1, axe1};

    pub_motor.publish(msg);
}
