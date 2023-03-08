#include "Arduino.h"
#include "Wire.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define ACCEL_I2C_ADDRESS_SHOULDER 0x69
#define ACCEL_ARRAY_SIZE 3
#define NOISE_FILTER_SIZE 10

int32_t sum = 0;
long readings[NOISE_FILTER_SIZE];
short index = 0;

// Fct prototypes
void get_accel_values(const int I2C_ADDRESS, int16_t angle_values[ACCEL_ARRAY_SIZE]);
void send_msg(int16_t angle_values[ACCEL_ARRAY_SIZE]);
int16_t processData(int newData);

// Global objects
ros::NodeHandle n;

std_msgs::Int16MultiArray accel_msg;
ros::Publisher pub_accel_pos("accel_pos", &accel_msg);

void setup()
{
    // ROS
    n.initNode();
    n.advertise(pub_accel_pos);

    // Accel
    Wire.begin();
    Wire.beginTransmission(ACCEL_I2C_ADDRESS_SHOULDER); // Start com with Accel
    Wire.write(0x6B);                                   // PWR_MGMT_1 register
    Wire.write(0);                                      // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    for (short i = 0; i < NOISE_FILTER_SIZE; i++)
    {
        readings[i] = 0;
    }
}

void loop()
{
    int16_t accel_angles[ACCEL_ARRAY_SIZE];

    get_accel_values(ACCEL_I2C_ADDRESS_SHOULDER, accel_angles);
    send_msg(accel_angles);
    n.spinOnce();
    delay(100);
}

void get_accel_values(const int I2C_ADDRESS, int16_t angle_values[ACCEL_ARRAY_SIZE])
{
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(ACCEL_I2C_ADDRESS_SHOULDER, 3 * 2, true);

    int16_t sensor_data[3]; // Array for capturing accel values before calculations

    sensor_data[0] = Wire.read() << 8 | Wire.read(); // angles x
    sensor_data[1] = Wire.read() << 8 | Wire.read(); // angles y
    sensor_data[2] = Wire.read() << 8 | Wire.read(); // angles z

    angle_values[0] = processData(sensor_data[0]);
    angle_values[1] = sensor_data[1];
    angle_values[2] = sensor_data[2];
}

void send_msg(int16_t angle_values[ACCEL_ARRAY_SIZE])
{
    std_msgs::Int16MultiArray msg;

    msg.data = angle_values;
    msg.data_length = ACCEL_ARRAY_SIZE;

    pub_accel_pos.publish(&msg);
}

int16_t processData(int newData)
{
    sum -= readings[index];
    readings[index] = newData;
    sum += newData;
    index = (index+1) % NOISE_FILTER_SIZE;

    return sum / NOISE_FILTER_SIZE;
}
