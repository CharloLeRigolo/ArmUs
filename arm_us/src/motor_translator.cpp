#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <unordered_map>
#include "std_msgs/Header.h"

#define NB_JOINT 5
const std::string NODE_NAME = "motor_translator";

void commandCallback(const sensor_msgs::JointStateConstPtr &msg);
void stateCallback(const sensor_msgs::JointStateConstPtr &msg);

ros::Publisher pub_command;
ros::Publisher pub_angles;
double joint_angles[NB_JOINT];
bool flag_no_connection = 0;

std::unordered_map<std::string, int> motor_map = {
    {"motor1", 0},
    {"motor2", 1},
    {"motor3", 2},
    {"motor4", 3},
    {"motor5", 4}
};

short motor_order[5] = {0, 1, 2, 3, 4};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    ros::Subscriber sub_command = n.subscribe("raw_desired_joint_states", 1, commandCallback);
    ros::Subscriber sub_state = n.subscribe("joint_states", 1, stateCallback);
    pub_command = n.advertise<sensor_msgs::JointState>("desired_joint_states", 1);
    pub_angles = n.advertise<sensor_msgs::JointState>("angles_joint_state", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}

void commandCallback(const sensor_msgs::JointStateConstPtr &msg)
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

    //No limit set for differential
    for (auto i = 0; i < 2; i++)
    {
        cmd.velocity[i] = msg->velocity[i];
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

void stateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    sensor_msgs::JointState angle_feedback;

    if (msg->position.size() < NB_JOINT)
    {
        if (!flag_no_connection)
        {
            ROS_WARN("Lost connection to a motor, only %d motor detected", msg->position.size());
            flag_no_connection = 1;
        }
        return;
    }
    flag_no_connection = 0;
    
    double max_angles[NB_JOINT];
    double min_angles[NB_JOINT];
    double pos_max_angles[NB_JOINT];
    double pos_min_angles[NB_JOINT];

    for (short i = 0; i < NB_JOINT; i++)
    {
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/pos_max_angle", pos_max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/pos_min_angle", pos_min_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/max_limit", max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/min_limit", min_angles[i]);
    }

    for (auto i = 0, motor_index = 0; i < NB_JOINT; i++)
    {
        motor_index = motor_map[msg->name[i]];
        // ROS_INFO("Motor index order: %d", motor_index);
        joint_angles[motor_index] = min_angles[motor_index] + ((msg->position[i] - pos_min_angles[motor_index]) * (max_angles[motor_index] - min_angles[motor_index]) / (pos_max_angles[motor_index] - pos_min_angles[motor_index]));
    }

    angle_feedback.name = {"motor1", "motor2", "motor3", "motor4", "motor5"};

    for (auto i=0; i < NB_JOINT; i++)
    {
        angle_feedback.position.push_back(joint_angles[i]);
        // angle_feedback.velocity[i] = 0.0;
        // angle_feedback.effort[i] = 0.0;
    }

    std_msgs::Header head;
    head.stamp = ros::Time::now();
    angle_feedback.header = head;

    pub_angles.publish(angle_feedback);
}
