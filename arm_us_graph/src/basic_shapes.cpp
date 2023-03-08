#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#include "arm_us/GraphInfo.h"

void update_graph();
void sub_angle_callback(const arm_us::GraphInfo::ConstPtr &msg);

struct Angles
{
  float theta1;
  float theta2;
  float theta3;
  float theta4;
  float theta5;
} angles;

const float lx1 = 2;
const float ly1 = 0.5;
const float lz1 = 0.5;

const float lx2 = 2;
const float ly2 = 0.5;
const float lz2 = 0.5;

ros::Publisher marker_pub;
ros::Subscriber angle_sub;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  angle_sub = n.subscribe("graph_info", 1, sub_angle_callback);

  while (ros::ok())
  {
    // angles.theta1 += 10;
    // angles.theta2 += 10;

    update_graph();
    r.sleep();
  }
}


void sub_angle_callback(const arm_us::GraphInfo::ConstPtr &data)
{
  angles.theta1 = data->angle[0];
  angles.theta1 = data->angle[1];
  angles.theta1 = data->angle[2];
  angles.theta1 = data->angle[3];
  angles.theta1 = data->angle[4];

  update_graph();
}


void update_graph()
{
  visualization_msgs::Marker arm1, arm2;
  // Set the frame ID and timestamp
  arm1.header.frame_id = "my_frame";
  arm1.header.stamp = ros::Time::now();

  arm2.header.frame_id = "my_frame";
  arm2.header.stamp = ros::Time::now();

  // Set the namespace and id for this arm1.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  arm1.ns = "basic_shapes";
  arm1.id = 0;

  arm2.ns = "basic_shapes";
  arm2.id = 1;

  // Set the marker type
  arm1.type = visualization_msgs::Marker::CUBE;
  arm2.type = visualization_msgs::Marker::CUBE;

  // Set the marker action
  arm1.action = visualization_msgs::Marker::ADD;
  arm1.action = visualization_msgs::Marker::ADD;

  // Set the pose
  float pos1x = lx1/2 * cos(angles.theta1 * M_PI / 180);
  float pos1y = 0;
  float pos1z = lx1/2 * sin(angles.theta1 * M_PI / 180);

  float pos2x = sqrt(((lx1 + lz2/2) * (lx1 + lz2/2))) * cos(angles.theta1 * M_PI / 180);
  float pos2y = 0;
  float pos2z = sqrt(((lx1 + lz2/2) * (lx1 + lz2/2))) * sin(angles.theta1 * M_PI / 180);

  tf2::Quaternion quat1, quat2;
  quat1.setEuler(-angles.theta1 * M_PI / 180, angles.theta2 * M_PI / 180, 0);
  quat2.setEuler((-angles.theta1 - 90) * M_PI / 180, 0, -angles.theta2 * M_PI / 180);

  arm1.pose.position.x = pos1x;
  arm1.pose.position.y = pos1y;
  arm1.pose.position.z = pos1z;

  arm1.pose.orientation.x = quat1.getX();
  arm1.pose.orientation.y = quat1.getY();
  arm1.pose.orientation.z = quat1.getZ();
  arm1.pose.orientation.w = quat1.getW();

  arm2.pose.position.x = pos2x;
  arm2.pose.position.y = pos2y;
  arm2.pose.position.z = pos2z;

  arm2.pose.orientation.x = quat2.getX();
  arm2.pose.orientation.y = quat2.getY();
  arm2.pose.orientation.z = quat2.getZ();
  arm2.pose.orientation.w = quat2.getW();

  // Set the scale
  arm1.scale.x = lx1;
  arm1.scale.y = ly1;
  arm1.scale.z = lz1;

  arm2.scale.x = lx2;
  arm2.scale.y = ly2;
  arm2.scale.z = lz2;

  // Set the color -- be sure to set alpha to something non-zero!
  arm1.color.r = 0.0f;
  arm1.color.g = 0.0f;
  arm1.color.b = 1.0f;
  arm1.color.a = 1.0;

  arm2.color.r = 1.0f;
  arm2.color.g = 0.0f;
  arm2.color.b = 0.0f;
  arm2.color.a = 1.0;

  arm1.lifetime = ros::Duration();
  arm2.lifetime = ros::Duration();

  marker_pub.publish(arm1);
  marker_pub.publish(arm2);
}