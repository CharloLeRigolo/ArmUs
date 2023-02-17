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

const float maxMotorSpeed = 5.0f;

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
    sub_input = n.subscribe("joy", 1, sub_input_callback);
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

void sub_input_callback(const sensor_msgs::Joy::ConstPtr &data)
{   
    joystickLeftVert =  data->axes[1] * 5;
    joystickRightVert = data->axes[4] * 5;
}

void subGuiCallback(const arm_us::MotorControl::ConstPtr &data)
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

/* Builds and publish motor message of type : sensor_msgs::JointState */
void sendCmdMotor()
{
    sensor_msgs::JointState msg;

    msg.name = { "motor1", "motor2" };
    msg.velocity = { joystickLeftVert, joystickRightVert };

    ROS_WARN("Motor 1 : %f, Motor 2 : %f", motors[0].velocity.data, motors[1].velocity.data);

    pub_motor.publish(msg);
}
