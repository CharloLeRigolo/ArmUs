#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "arm_us/MotorControl.h"

#define ROS_RATE 50

void init();
void loop();

void subInputCallback(const sensor_msgs::Joy::ConstPtr &data);
void subGuiCallback(const arm_us::MotorControl::ConstPtr &data);

void sendCmdMotor();

ros::Subscriber sub_input;
ros::Subscriber sub_gui;

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
    sub_input = n.subscribe("joy", 1, subInputCallback);
    sub_gui = n.subscribe("gui_arm_us_chatter", 1, subGuiCallback);
    pub_motor = n.advertise<sensor_msgs::JointState>("desired_joint_states", 10);
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
}

void subInputCallback(const sensor_msgs::Joy::ConstPtr &data)
{   
    motors[0].velocity = data->axes[1] * 5;
    motors[1].velocity = data->axes[4] * 5;
}

void subGuiCallback(const arm_us::MotorControl::ConstPtr &data)
{
    motors[data->motor_id].velocity = data->velocity;
}

/* Builds and publish motor message of type : sensor_msgs::JointState */
void sendCmdMotor()
{
    sensor_msgs::JointState msg;

    msg.name = { "motor1", "motor2" };
    msg.velocity = { motors[0].velocity, motors[1].velocity };

    //ROS_WARN("Motor 1 : %f, Motor 2 : %f", motors[0].velocity, motors[1].velocity);

    pub_motor.publish(msg);
}
