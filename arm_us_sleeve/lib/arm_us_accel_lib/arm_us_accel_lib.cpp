#include "arm_us_accel_lib.h"

using namespace arm_us;

movingAverage::movingAverage()
{
    // Array init
    for (short i = 0; i < NOISE_FILTER_SIZE; i++)
    {
        datas[i] = 0;
    }
}

void movingAverage::addData(short newData)
{
    sum -= datas[currIndex];
    datas[currIndex] = newData;
    sum += newData;
    // currIndex = currIndex >= NOISE_FILTER_SIZE? 0 : currIndex;
    currIndex = (currIndex + 1) % NOISE_FILTER_SIZE;
}

short movingAverage::getCurrAverage()
{
    return sum / NOISE_FILTER_SIZE;
}

short movingAverage::getLastData()
{
    return datas[currIndex];
}

void AccelLib::init()
{
    // initialize();
    // dmpInitialize();
    // // Those value come from running the calibration code in MPU6050's library: "/examples/IMU_Zero/IMU_Zero.ino"
    // setXAccelOffset(0);
    // setYAccelOffset(0);
    // setZAccelOffset(0);
    // setXGyroOffset(0);
    // setYGyroOffset(0);
    // setZGyroOffset(0);
    // packetSize = dmpGetFIFOPacketSize();
    // fifoCount = getFIFOCount();
}

FifoState AccelLib::refresh_loop()
{
    while (fifoCount < packetSize)
    {
        fifoCount = getFIFOCount();
    }

    if (fifoCount == 1024)
    {
        resetFIFO();
        fifo_state = overflow;
    }
    else
    {
        if (fifoCount % packetSize != 0)
        {
            resetFIFO();
            fifo_state = sync_error;
        }
        else
        {
            while (fifoCount >= packetSize)
            {
                getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
            }

            dmpGetQuaternion(&last_data, fifoBuffer);
            fifo_state = ok;
        }
    }

    return fifo_state;
}
