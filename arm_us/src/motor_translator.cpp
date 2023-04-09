#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <unordered_map>
#include "std_msgs/Header.h"
// #include "ArmUs.hpp" Cannot build with this include

#define NB_JOINT 5                                // numbers of Joints
const std::string NODE_NAME = "motor_translator"; // used for getting rosparam with correct name

void commandCallback(const sensor_msgs::JointStateConstPtr &msg);
void stateCallback(const sensor_msgs::JointStateConstPtr &msg);
void simulateStateCallback();
void setParams();

// Global variables
ros::Publisher pub_command;
ros::Publisher pub_angles;

double joint_angles[NB_JOINT];
double joint_velocity[NB_JOINT];
double joint_effort[NB_JOINT];
double joint_positions[NB_JOINT]; // For simulation purposes only
bool flag_no_connection = 0;
double max_speed;

enum class ControlMode
{
    Real = 0,
    Simulation = 1
};
ControlMode g_control_mode;

std::unordered_map<std::string, int> motor_map = {
    {"motor1", 0},
    {"motor2", 1},
    {"motor3", 2},
    {"motor4", 3},
    {"motor5", 4}};

short motor_order[5] = {0, 1, 2, 3, 4};

int main(int argc, char *argv[])
{
   
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    setParams();

    ros::Subscriber sub_command = n.subscribe("raw_desired_joint_states", 1, commandCallback);
    ros::Subscriber sub_state = n.subscribe("joint_states", 1, stateCallback);

    pub_command = n.advertise<sensor_msgs::JointState>("desired_joint_states", 1);
    pub_angles = n.advertise<sensor_msgs::JointState>("angles_joint_state", 1);

    while (ros::ok())
    {
        if (g_control_mode == ControlMode::Simulation)
        {
            // Replace position callback from motors
            simulateStateCallback();
        }
        ros::spinOnce();
    }
}

/**
 * @brief Get's raw JointState comming from Arm_us
 * and checks joint limits and add a speed limiter
 * correction factor to keep cartesian control accurate
 *
 * @param msg
 */
void commandCallback(const sensor_msgs::JointStateConstPtr &msg)
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

    // No limit set for differential
    for (auto i = 0; i < 2; i++)
    {
        cmd.velocity[i] = msg->velocity[i];
    }

    // Checking other motor limits
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

    // ROS_WARN("Command with joint limits :");
    // ROS_WARN("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", cmd.velocity[0], cmd.velocity[1], cmd.velocity[2], cmd.velocity[3], cmd.velocity[4]);
    
    // Applying speed limiter
    // NEEDS to be tested
    double cmd_max_speed = *std::max_element(std::begin(cmd.velocity), std::end(cmd.velocity));
    if (cmd_max_speed > max_speed)
    {
        double factor = max_speed/cmd_max_speed;
        for (auto i = 0; i < NB_JOINT; i++)
        {
            cmd.velocity[i] *= factor;
        }
    }

    // ROS_WARN("Command with joint limits and speed limiter :");
    // ROS_WARN("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", cmd.velocity[0], cmd.velocity[1], cmd.velocity[2], cmd.velocity[3], cmd.velocity[4]);

    // Building and sending cmd msg
    if (g_control_mode == ControlMode::Simulation)
    {
        joint_positions[0] += cmd.velocity[0];
        joint_positions[1] += cmd.velocity[1];
        joint_positions[2] += cmd.velocity[2];
        joint_positions[3] += cmd.velocity[3];
        joint_positions[4] += cmd.velocity[4];
        // ROS_WARN("Joint positions with joint limits :");
        // ROS_WARN("m1 = %f, m2 = %f, m3 = %f, m4 = %f, m5 = %f", joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3], joint_positions[4]);
    }

    pub_command.publish(cmd);
}

/**
 * @brief Callback to translate raw position value of motor to angle
 * and reorganise msg with sorted motor order (1, 2, 3, ...)
 * 
 * @param msg 
 */
void stateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    sensor_msgs::JointState angle_feedback;

    int msg_size = msg->position.size();
    if ((msg_size < NB_JOINT) || (msg->position.size() != msg->effort.size()) || (msg->position.size() != msg->velocity.size()))
    {
        if (!flag_no_connection)
        {
            ROS_WARN("Lost connection to a motor, only %d motor detected", msg_size);
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
        joint_velocity[motor_index] = msg->velocity[i];
        joint_effort[motor_index] = msg->effort[i];
    }

    angle_feedback.name = {"motor1", "motor2", "motor3", "motor4", "motor5"};

    for (auto i = 0; i < NB_JOINT; i++)
    {
        angle_feedback.position.push_back(joint_angles[i]);
        angle_feedback.velocity.push_back(joint_velocity[i]);
        angle_feedback.effort.push_back(joint_effort[i]);
    }

    std_msgs::Header head;
    head.stamp = ros::Time::now();
    angle_feedback.header = head;

    // ROS_WARN("Joint angles with joint limits :");
    // ROS_WARN("j1 = %f, j2 = %f, j3 = %f, j4 = %f, j5 = %f", angle_feedback.position[0], angle_feedback.position[1], angle_feedback.position[2], angle_feedback.position[3], angle_feedback.position[4]);

    pub_angles.publish(angle_feedback);
}

/**
 * @brief Angle callback when in simulation mode (bypass normal callback)
 * 
 */
void simulateStateCallback()
{
    sensor_msgs::JointState angle_feedback;
    angle_feedback.name = {"motor1", "motor2", "motor3", "motor4", "motor5"};
    angle_feedback.position = {0.0, 0.0, 0.0, 0.0, 0.0};

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

    // ROS_WARN("Joint angles with joint limits :");
    // ROS_WARN("j1 = %f, j2 = %f, j3 = %f, j4 = %f, j5 = %f", angle_feedback.position[0], angle_feedback.position[1], angle_feedback.position[2], angle_feedback.position[3], angle_feedback.position[4]);

    pub_angles.publish(angle_feedback);
}

/**
 * @brief Gets and set the needed ros::Params as global variable
 * 
 */
void setParams()
{
    int controlMode = -1;

    ros::param::get("/dynamixel_interface_node/global_max_vel", max_speed);
    ros::param::get("/master_node/control_mode", controlMode);

    switch (controlMode)
    {
    // Real
    case 0:
    {
        g_control_mode = ControlMode::Real;
        ROS_WARN("Started interface node in real control mode");
        break;
    }
    // Simulation
    case 1:
    {
        g_control_mode = ControlMode::Simulation;
        ROS_WARN("Started interface node in simulation control mode");
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
