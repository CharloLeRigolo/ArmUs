#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#define NB_MARKER 3

// fct definitions
void sub_shoulder_angles_cb(const std_msgs::Int16MultiArray::ConstPtr &data);
float map(int val, int min_input, int max_input, float min_val, float max_val);

class markerObject
{
public:
    visualization_msgs::Marker msg;
    markerObject(std::string ns, int id, uint32_t shape)
    {
        msg.header.frame_id = "arm_us";
        msg.header.stamp = ros::Time::now();

        msg.ns = "basic_shapes";
        msg.id = id;
        msg.type = shape;
        msg.action = visualization_msgs::Marker::ADD;

        msg.pose.position.x = 0;
        msg.pose.position.y = 0;
        msg.pose.position.z = 0;

        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;

        msg.scale.x = 1.0;
        msg.scale.y = 1.0;
        msg.scale.z = 1.0;

        msg.color.r = 0.0f;
        msg.color.g = 1.0f;
        msg.color.b = 0.0f;
        msg.color.a = 1.0;
    }
    void sendMsg(ros::Publisher *pub)
    {
        msg.header.stamp = ros::Time::now();
        msg.lifetime = ros::Duration();

        pub->publish(msg);
    }
};

ros::Subscriber sub_shoulder_angles;

float j1_angle_x = 0.0;
float j1_angle_y = 0.0;
float j1_angle_z = 0.0;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    sub_shoulder_angles = n.subscribe("accel_pos_wrist", 1, sub_shoulder_angles_cb);
    ros::Rate rate(10);

    markerObject p_shoulder_marker("arm_us", 0, visualization_msgs::Marker::SPHERE);
        p_shoulder_marker.msg.scale.x = 0.1;
        p_shoulder_marker.msg.scale.y = 0.1;
        p_shoulder_marker.msg.scale.z = 0.1; 

    markerObject l_shoulder_marker("arm_us", 1, visualization_msgs::Marker::MESH_RESOURCE);
        l_shoulder_marker.msg.mesh_resource = "file:///home/phil/catkin_ws/src/arm_us/arm_us/model/Arm_Joint.dae";
        l_shoulder_marker.msg.scale.x = 0.0005;
        l_shoulder_marker.msg.scale.y = 0.001;
        l_shoulder_marker.msg.scale.z = 0.0005;

    l_shoulder_marker.msg.pose.position.z = -l_shoulder_marker.msg.scale.z / 2;

    markerObject *markerObjects[NB_MARKER] = {&p_shoulder_marker, &l_shoulder_marker};

    while (!ros::isShuttingDown())
    {   

        tf2::Quaternion quart1;
        quart1.setEuler(0.0, j1_angle_x, 0.0);

        l_shoulder_marker.msg.pose.orientation.x = quart1.getX();
        l_shoulder_marker.msg.pose.orientation.y = quart1.getY();
        l_shoulder_marker.msg.pose.orientation.z = quart1.getZ();
        l_shoulder_marker.msg.pose.orientation.w = quart1.getW();

        for (auto i = 0; i < NB_MARKER; i++)
        {
            if (markerObjects[i] != NULL)
            {
                markerObjects[i]->sendMsg(&pub_marker);
            }
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

void sub_shoulder_angles_cb(const std_msgs::Int16MultiArray::ConstPtr &data)
{
    j1_angle_x = map(data->data[0], -32768, 32767, -M_PI, M_PI);  
    j1_angle_y = map(data->data[1], -32768, 32767, -M_PI, M_PI);  
    j1_angle_z = map(data->data[2], -32768, 32767, -M_PI, M_PI);  
    // ROS_WARN("%d = %f", data->data[0], j1_angle_x);
}

float map(int i_val, int i_min_input, int i_max_input, float min_val, float max_val)
{
    float val = float(i_val);
    float min_input = float(i_min_input);
    float max_input = float(i_max_input);

    return (val - i_min_input) * (max_val - min_val) / (i_max_input - i_min_input) + min_val;
}
