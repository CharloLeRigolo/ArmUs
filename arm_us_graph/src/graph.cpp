#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

#include "arm_us_msg/GraphInfo.h"

const int ROS_RATE = 50;

void UpdateGraph();
void CalculatePositions();
void sub_angle_callback(const arm_us_msg::GraphInfo::ConstPtr &msg);

const float J1x = 2.0548;
const float J1y = 0;
const float J1z = 0;

const float J2x = 0;
const float J2y = -2.25;
const float J2z = 0;

const float J3x = 0;
const float J3y = 1.73;
const float J3z = 0;

const float J4x = 0;
const float J4y = 0.40;
const float J4z = 0;

struct Vector3
{
  Vector3(float xx = 0, float yy = 0, float zz = 0) : x(xx), y(yy), z(zz) {}
  void set(float xx = 0, float yy = 0, float zz = 0) { x = xx; y = yy; z = zz; }
  float x, y, z;
};

struct Arm
{
  Vector3 J1, J2, J3, J4;
  Vector3 P1, P2, P3, P4;
  float q1, q2, q3, q4, q5;
};

Arm arm;

ros::Publisher marker_pub;
ros::Subscriber angle_sub;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "arm_us_graph");
  ros::NodeHandle n;
  ros::Rate loop_rate(ROS_RATE);
 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
  angle_sub = n.subscribe("graph_info", 1, sub_angle_callback);

  arm.J1.set(J1x, J1y, J1z);
  arm.J2.set(J2x, J2y, J2z);
  arm.J3.set(J3x, J3y, J3z);
  arm.J4.set(J4x, J4y, J4z);

  while (ros::ok())
  {
    CalculatePositions();
    UpdateGraph();

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
}

void sub_angle_callback(const arm_us_msg::GraphInfo::ConstPtr &data)
{
  arm.q1 = data->angle[0];
  arm.q2 = data->angle[1];
  arm.q3 = data->angle[2];
  arm.q4 = data->angle[3];
  arm.q5 = -1 * data->angle[4];
  // ROS_WARN("j1 = %f, j2 = %f, j3 = %f, j4 = %f, j5 = %f", arm.q1, arm.q2, arm.q3, arm.q4, arm.q5);
}

void UpdateGraph()
{
  visualization_msgs::Marker points, line_strip;

  points.header.frame_id = line_strip.header.frame_id = "my_frame";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "robot_arm";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = 0.2;
  points.scale.y = 0.2;

  line_strip.scale.x = 0.1;

  points.color.b = 1.0f;
  points.color.a = 1.0f;

  line_strip.color.g = 1.0f;
  line_strip.color.a = 1.0f;

  Vector3 temp_pos[5] = { Vector3(), arm.P1, arm.P2, arm.P3, arm.P4 };
  for (Vector3 pos : temp_pos)
  {
    geometry_msgs::Point p;
    p.x = pos.x;
    p.y = pos.y;
    p.z = pos.z;

    //ROS_WARN("x : %f, y : %f, z : %f", pos.x, pos.y, pos.z);

    points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);
}

void CalculatePositions()
{
  float q1 = arm.q1 * M_PI / 180;
  float q2 = arm.q2 * M_PI / 180;
  float q3 = arm.q3 * M_PI / 180;
  float q4 = arm.q4 * M_PI / 180;
  float q5 = arm.q5 * M_PI / 180;

  arm.P1.x = J1x*cos(q1) + J1z*sin(q1);
  arm.P1.y = J1y;
  arm.P1.z = J1z*cos(q1) - J1x*sin(q1);

  arm.P2.x = J1z*sin(q1) + cos(q1)*(J1x+J2x) + J2y*sin(q1)*sin(q2) + J2z*sin(q1)*cos(q2);
  arm.P2.y = J1y + J2y*cos(q2) - J2z*sin(q2);
  arm.P2.z = J1z*cos(q1) + J2y*sin(q2)*cos(q1) + J2z*cos(q1)*cos(q2) - sin(q1)*(J1x+J2x);

  arm.P3.x = J1z*sin(q1) + cos(q1)*(J1x+J2x) + J2y*sin(q1)*sin(q2) + sin(q1)*cos(q2)*(J2z+J3z) + J3x*(cos(q1)*cos(q3)+sin(q1)*sin(q2)*sin(q3))- J3y*(sin(q3)*cos(q1)-sin(q1)*sin(q2)*cos(q3));
  arm.P3.y = J1y + J2y*cos(q2) + J3x*sin(q3)*cos(q2) + J3y*cos(q2)*cos(q3)- sin(q2)*(J2z+J3z);
  arm.P3.z = J1z*cos(q1) + J2y*sin(q2)*cos(q1) + cos(q1)*cos(q2)*(J2z+J3z)+ J3y*(sin(q1)*sin(q3)+sin(q2)*cos(q1)*cos(q3)) - sin(q1)*(J1x+J2x)- J3x*(sin(q1)*cos(q3)-sin(q2)*sin(q3)*cos(q1));

  arm.P4.x = J1z*sin(q1) + cos(q1)*(J1x+J2x) + J2y*sin(q1)*sin(q2) + sin(q1)*cos(q2)*(J2z+J3z) + J3x*(cos(q1)*cos(q3)+sin(q1)*sin(q2)*sin(q3))+ J4x*(cos(q1)*cos(q3)*cos(q4)-sin(q1)*(sin(q4)*cos(q2)-sin(q2)*sin(q3)*cos(q4))) + J4z*(cos(q1)*(sin(q3)*sin(q5)+sin(q4)*cos(q3)*cos(q5))+sin(q1)*(cos(q2)*cos(q4)*cos(q5)-sin(q2)*(sin(q5)*cos(q3)-sin(q3)*sin(q4)*cos(q5)))) - J3y*(sin(q3)*cos(q1)-sin(q1)*sin(q2)*cos(q3)) - J4y*(cos(q1)*(sin(q3)*cos(q5)-sin(q4)*sin(q5)*cos(q3))-sin(q1)*(sin(q5)*cos(q2)*cos(q4)+sin(q2)*(cos(q3)*cos(q5)+sin(q3)*sin(q4)*sin(q5))));
  arm.P4.y = J1y + J2y*cos(q2) + J3x*sin(q3)*cos(q2) + J3y*cos(q2)*cos(q3)+ J4x*(sin(q2)*sin(q4)+sin(q3)*cos(q2)*cos(q4)) - sin(q2)*(J2z+J3z)- J4y*(sin(q2)*sin(q5)*cos(q4)-cos(q2)*(cos(q3)*cos(q5)+sin(q3)*sin(q4)*sin(q5))) - J4z*(sin(q2)*cos(q4)*cos(q5)+cos(q2)*(sin(q5)*cos(q3)-sin(q3)*sin(q4)*cos(q5)));
  arm.P4.z = J1z*cos(q1) + J2y*sin(q2)*cos(q1) + cos(q1)*cos(q2)*(J2z+J3z)+ J3y*(sin(q1)*sin(q3)+sin(q2)*cos(q1)*cos(q3)) + J4y*(sin(q1)*(sin(q3)*cos(q5)-sin(q4)*sin(q5)*cos(q3))+cos(q1)*(sin(q5)*cos(q2)*cos(q4)+sin(q2)*(cos(q3)*cos(q5)+sin(q3)*sin(q4)*sin(q5)))) - sin(q1)*(J1x+J2x)- J3x*(sin(q1)*cos(q3)-sin(q2)*sin(q3)*cos(q1)) - J4x*(sin(q1)*cos(q3)*cos(q4)+cos(q1)*(sin(q4)*cos(q2)-sin(q2)*sin(q3)*cos(q4))) - J4z*(sin(q1)*(sin(q3)*sin(q5)+sin(q4)*cos(q3)*cos(q5))-cos(q1)*(cos(q2)*cos(q4)*cos(q5)-sin(q2)*(sin(q5)*cos(q3)-sin(q3)*sin(q4)*cos(q5))));
  
  // ROS_WARN("P1 : x = %f, y = %f, z = %f", arm.P1.x, arm.P1.y, arm.P1.z);
  // ROS_WARN("P2 : x = %f, y = %f, z = %f", arm.P2.x, arm.P2.y, arm.P2.z);
  // ROS_WARN("P3 : x = %f, y = %f, z = %f", arm.P3.x, arm.P3.y, arm.P3.z);
  // ROS_WARN("P4 : x = %f, y = %f, z = %f", arm.P4.x, arm.P4.y, arm.P4.z);
}
