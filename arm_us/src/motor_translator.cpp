#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <unordered_map>

#define NB_JOINT 5
const std::string NODE_NAME = "motor_translator";

void command_callback(const sensor_msgs::JointStateConstPtr &msg);
void state_callback(const sensor_msgs::JointStateConstPtr &msg);

ros::Publisher pub_command;
ros::Publisher pub_angles;
double joint_angles[NB_JOINT];

std::unordered_map<std::string, int> motor_map = {
    {"motor1", 0},
    {"motor2", 1},
    {"motor3", 2},
    {"motor4", 3},
    {"motor5", 4}
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::Subscriber sub_command = n.subscribe("raw_desired_joint_states", 1, command_callback);
    ros::Subscriber sub_state = n.subscribe("joint_states", 1, state_callback);
    pub_command = n.advertise<sensor_msgs::JointState>("motor_topic", 1);
    pub_angles = n.advertise<sensor_msgs::JointState>("angles_joint_state", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}

void command_callback(const sensor_msgs::JointStateConstPtr &msg)
{
    sensor_msgs::JointState cmd;

    // Updating parameters
    double max_angles[NB_JOINT];
    double min_angles[NB_JOINT];

    cmd = *msg;
    for (short i = 0; i < NB_JOINT; i++)
    {
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/max_limit", max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/min_limit", min_angles[i]);
        cmd.velocity[i] = 0.0;
    }

    // Checking motor limits
    for (auto i = 2; i < NB_JOINT; i++)
    {
        if ((msg->velocity[i] > 0.0 && joint_angles[i] >= max_angles[i]) || (msg->velocity[i] < 0.0 && joint_angles[i] <= min_angles[i]))
        {
            cmd.velocity[i] = 0.0;
            ROS_WARN("Joint #%d at limit", (i + 1));
        }
        else
        {
            cmd.velocity[i] = msg->velocity[i];
        }
    }

    pub_command.publish(cmd);
}

void state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
    sensor_msgs::JointState angle_feedback;

    double max_angles[NB_JOINT];
    double min_angles[NB_JOINT];
    double pos_max_angles[NB_JOINT];
    double pos_min_angles[NB_JOINT];

    for (short i = 0, j = 0; i < NB_JOINT; i++)
    {
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/pos_max_angle", pos_max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/pos_min_angle", pos_min_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/max_limit", max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/min_limit", min_angles[i]);
    }

    angle_feedback = *msg;
    for (auto i = 2; i < NB_JOINT; i++)
    {
        joint_angles[i] = min_angles[i] + ((msg->position[i] - pos_min_angles[i]) * (max_angles[i] - min_angles[i]) / (pos_max_angles[i] - pos_min_angles[i]));
        angle_feedback.position[i] = joint_angles[i];
    }

    pub_angles.publish(angle_feedback);
}
