#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#define NB_MARKER 3

// fct definitions
void sub_shoulder_angles_cb(const std_msgs::Int16MultiArray::ConstPtr &data);

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    sub_shoulder_angles = n.subscribe("accel_pos", 1, sub_shoulder_angles_cb);

    markerObject p_shoulder_marker("arm_us", 0, visualization_msgs::Marker::SPHERE);
        p_shoulder_marker.msg.scale.x = 0.1;
        p_shoulder_marker.msg.scale.y = 0.1;
        p_shoulder_marker.msg.scale.z = 0.1; 
    markerObject l_shoulder_marker("arm_us", 1, visualization_msgs::Marker::CUBE);
        l_shoulder_marker.msg.scale.x = 0.1;
        l_shoulder_marker.msg.scale.y = 0.1;
        l_shoulder_marker.msg.scale.z = 1;

    l_shoulder_marker.msg.pose.position.z = -l_shoulder_marker.msg.scale.z / 2;

    markerObject *markerObjects[NB_MARKER] = {&p_shoulder_marker, &l_shoulder_marker};
    float angle = 90.0;

    while (!ros::isShuttingDown())
    {   
        float angle = M_PI_2;

        tf2::Quaternion quart1;
        quart1.setEuler(angle, 0.0, 0.0);

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

        r.sleep();
    }
}

void sub_shoulder_angles_cb(const std_msgs::Int16MultiArray::ConstPtr &data)
{
}
