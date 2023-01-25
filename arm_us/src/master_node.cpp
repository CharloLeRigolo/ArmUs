#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

void init();
void loop();
void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data);
void update_btn1();

bool btn1 = false;
ros::Subscriber sub_input;

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
}

void loop()
{
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        update_btn1();
    
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{
    btn1 = data->buttons[0];
}

void update_btn1()
{
    if (btn1)
    {
        ROS_WARN("Button 1 pressed");
    }
}