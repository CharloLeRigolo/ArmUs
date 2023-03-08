#include "Arduino.h"
#include "Wire.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <arm_us_accel_lib.h>

//Constant / Macros
#define ACCEL_ARRAY_SIZE 3
#define ACCEL_ADDRESS_SHOULDER 0x69
unsigned long MSG_PERIOD_MS = 100; //ms

// Fct prototypes
void send_msg();

// Global objects
ros::NodeHandle n;
std_msgs::Int16MultiArray accel_msg;
ros::Publisher pub_accel_pos("accel_pos", &accel_msg);
arm_us::accel_lib accel_shoulder(ACCEL_ADDRESS_SHOULDER);

//Global variables
unsigned long prev_millis = 0;

void setup()
{
    // ROS
    n.initNode();
    n.advertise(pub_accel_pos);

    accel_shoulder.init();  
}

void loop()
{
    accel_shoulder.acquire_accel_values();
    
    if (millis() - prev_millis > MSG_PERIOD_MS)
    {
        send_msg();
        n.spinOnce();
        prev_millis = millis();
    }
}

void send_msg()
{
    std_msgs::Int16MultiArray msg;
    
    int16_t angle_values[ACCEL_ARRAY_SIZE];

    for (short i = 0; i < ACCEL_ARRAY_SIZE; i++)
    {
        angle_values[i] = accel_shoulder.get_accel_values_processed(i);
    }

    msg.data = angle_values;
    msg.data_length = ACCEL_ARRAY_SIZE;

    pub_accel_pos.publish(&msg);
}
