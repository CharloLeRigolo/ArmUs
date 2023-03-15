#ifndef ARM_US_ACCEL_LIB_H
#define ARM_US_ACCEL_LIB_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"

#define NOISE_FILTER_SIZE 50
#define NB_DATA 3

// Calib results[-1167,-1167] --> [-6,27]	[-31,-31] --> [-10,88]	[2312,2313] --> [16354,16385]	[-40,-39] --> [-2,1]	[-101,-100] --> [0,1]	[-55,-54] --> [-1,3]
namespace arm_us
{
    enum FifoState
    {
        ok,
        overflow,
        sync_error
    };

    class movingAverage
    {
    private:
        long sum = 0;
        int datas[NOISE_FILTER_SIZE];
        short currIndex = 0;

    public:
        movingAverage();
        void addData(short newData);
        short getCurrAverage();
        short getLastData();
    };

    // Adapted from https://forum.arduino.cc/t/good-news-dmp-from-mpu6050-can-be-used-without-interrupt-pin/393797
    class AccelLib : public MPU6050
    {
    private:
        FifoState fifo_state = ok;
        uint8_t fifoBuffer[1024];
        Quaternion last_data;
        int packetSize = 42;
        int fifoCount = 0;

    public:
        AccelLib(uint8_t addresse) : MPU6050(addresse) {};
        void init();
        FifoState refresh_loop();
        Quaternion getLastData() { return last_data; };
    };
}

#endif
