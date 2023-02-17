#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "arm_us/MotorControl.h"

void init();
void loop();

void subInputCallback(const sensor_msgs::Joy::ConstPtr &data);
void subGuiCallback(const arm_us::MotorControl::ConstPtr &data);

void sendCmdMotor();

ros::Subscriber sub_input;
ros::Subscriber sub_gui;

ros::Publisher pub_motor;

const float maxMotorSpeed = 5.0f;

arm_us::MotorControl motors[2];

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
    sub_input = n.subscribe("joy", 1, subInputCallback);
    sub_gui = n.subscribe("arm_us/msg", 1, subGuiCallback);

    pub_motor = n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);    
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
}

void subInputCallback(const sensor_msgs::Joy::ConstPtr &data)
{   
    motors[0].velocity.data = (float)data->axes[1] * maxMotorSpeed;
    motors[1].velocity.data = (float)data->axes[4] * maxMotorSpeed;
}

void subGuiCallback(const arm_us::MotorControl::ConstPtr &data)
{
    int id = (int)data->motor_id.data;
    motors[id].enabled.data = data->enabled.data;
    motors[id].velocity.data = data->velocity.data;
}

void sendCmdMotor()
{
    sensor_msgs::JointState msg;

    msg.name = { "motor1", "motor2" };
    msg.velocity = { motors[0].velocity.data, motors[1].velocity.data };

    ROS_WARN("Motor 1 : %f, Motor 2 : %f", motors[0].velocity.data, motors[1].velocity.data);

    pub_motor.publish(msg);
}
