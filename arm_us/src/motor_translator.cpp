#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <unordered_map>

#define NB_JOINT 5
const std::string NODE_NAME = "motor_translator";

void command_callback(const sensor_msgs::JointStateConstPtr &msg);
void state_callback(const sensor_msgs::JointStateConstPtr &msg);
void simulate_state_callback();
void set_params(ros::NodeHandle n);

ros::Publisher pub_command;
ros::Publisher pub_angles;

double joint_angles[NB_JOINT];
double joint_positions[NB_JOINT];

enum class ControlMode { Real = 0, Simulation = 1 };
ControlMode g_control_mode;

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

    set_params(n);

    ros::Subscriber sub_command = n.subscribe("raw_desired_joint_states", 1, command_callback);
    if (g_control_mode == ControlMode::Real)
    {
        ros::Subscriber sub_state = n.subscribe("joint_states", 1, state_callback);
    }

    pub_command = n.advertise<sensor_msgs::JointState>("motor_topic", 1);
    pub_angles = n.advertise<sensor_msgs::JointState>("angles_joint_state", 1);

    while (ros::ok())
    {
        if (g_control_mode == ControlMode::Simulation)
        {
            // Replace position callback from motors
            simulate_state_callback();
        }
        ros::spinOnce();
    }
}

void command_callback(const sensor_msgs::JointStateConstPtr &msg)
{
    sensor_msgs::JointState cmd;
    cmd = *msg;

    // Updating parameters
    double max_angles[NB_JOINT];
    double min_angles[NB_JOINT];

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

    cmd.velocity[0] = msg->velocity[0];
    cmd.velocity[1] = msg->velocity[1];

    // ROS_INFO("Command with joint limits :");
    // ROS_INFO("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", cmd.velocity[0], cmd.velocity[1], cmd.velocity[2], cmd.velocity[3], cmd.velocity[4]);

    if (g_control_mode == ControlMode::Simulation)
    {
        joint_positions[0] += cmd.velocity[0];
        joint_positions[1] += cmd.velocity[1];
        joint_positions[2] += cmd.velocity[2];
        joint_positions[3] += cmd.velocity[3];
        joint_positions[4] += cmd.velocity[4];
        ROS_INFO("Joint positions with joint limits :");
        ROS_INFO("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4]);
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

void simulate_state_callback()
{   
    sensor_msgs::JointState angle_feedback;
    angle_feedback.name = { "motor1", "motor2", "motor3", "motor4", "motor5" };
    angle_feedback.position = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    double max_angles[NB_JOINT];
    double min_angles[NB_JOINT];
    double pos_max_angles[NB_JOINT];
    double pos_min_angles[NB_JOINT];

    for (int i = 0, j = 0; i < NB_JOINT; i++)
    {
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/pos_max_angle", pos_max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/pos_min_angle", pos_min_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/max_limit", max_angles[i]);
        ros::param::get("/" + NODE_NAME + "/j" + std::to_string(i + 1) + "/min_limit", min_angles[i]);
    }
    
    for (auto i = 0; i < NB_JOINT; i++)
    {
        joint_angles[i] = min_angles[i] + ((joint_positions[i] - pos_min_angles[i]) * (max_angles[i] - min_angles[i]) / (pos_max_angles[i] - pos_min_angles[i]));
        angle_feedback.position[i] = joint_angles[i];
    }

    ROS_INFO("Joint angles with joint limits :");
    ROS_INFO("j1 = %f, j2 = %f, j3 = %f, j4 = %f, j5 = %f", angle_feedback.position[0], angle_feedback.position[1], angle_feedback.position[2], angle_feedback.position[3], angle_feedback.position[4]);

    pub_angles.publish(angle_feedback);
}

void set_params(ros::NodeHandle n)
{
    int controlMode = -1;
    n.getParam("/master_node/control_mode", controlMode);

    switch (controlMode)
    {
        // Real
        case 0:
        {
            g_control_mode = ControlMode::Real;
            ROS_INFO("Started interface node in real control mode");
            break;
        }
        // Simulation
        case 1:
        {
            g_control_mode = ControlMode::Simulation;
            ROS_INFO("Started interface node in simulation control mode");
            break;
        }
        // Default
        default:
        {
            g_control_mode = ControlMode::Real;
            ROS_ERROR("Control mode parameter not read from launch file");
            break;
        }
    }

    joint_positions[0] = 2048.0;
    joint_positions[1] = 2048.0;
    joint_positions[2] = 0.0;
    joint_positions[3] = 0.0;
    joint_positions[4] = 0.0;
}