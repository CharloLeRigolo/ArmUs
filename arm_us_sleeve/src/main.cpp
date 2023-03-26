#include <Arduino.h>
#include <ros.h>
#include <arm_us/accel_pos.h>

#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

#define BNO_ADDRESS 0x28
#define BNO_ID 55
#define PRINT_RATE 20

Adafruit_BNO055 bno(BNO_ID, BNO_ADDRESS, &Wire);

void setup(void)
{
    ros::NodeHandle n;
    arm_us::accel_pos accel_msg;
    ros::Publisher pub_shoulder("pos_shoulder", &accel_msg);

    n.initNode();
    n.advertise(pub_shoulder);

    // Serial.begin(115200);
    // while(!Serial); //Waiting for Serial

    if (!bno.begin())
    {
        n.logerror("Error with BNO, going in while loop");
        while(1);
    }

    bno.setExtCrystalUse(true);

    imu::Quaternion quat;
    float x, y, z; 
    unsigned long prev_millis = 0;

    // Main
    while (1)
    {
        if (millis() - prev_millis > PRINT_RATE)
        {
            quat = bno.getQuat();
            arm_us::accel_pos msg;
            msg.w = quat.w();
            msg.x = quat.x();
            msg.y = quat.y();
            msg.z = quat.z();
            pub_shoulder.publish(&msg);

            n.spinOnce();
        }
    }
}

void loop(void)
{
    return;
}
