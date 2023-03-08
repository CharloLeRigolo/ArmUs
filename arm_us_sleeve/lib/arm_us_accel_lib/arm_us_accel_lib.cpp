#include <arm_us_accel_lib.h>
#include <Arduino.h>
#include <Wire.h>

using namespace arm_us;

accel_lib::accel_lib(int i_address)
{
    address = i_address;
    for(short i=0; i < NB_DATA; i++)
    {
        noise_filters[i] = arm_us::movingAverage();
    }
}

void accel_lib::init()
{
    //Init accel com
    Wire.begin();
    Wire.beginTransmission(address); // Start com with Accel
    Wire.write(0x6B);                // PWR_MGMT_1 register
    Wire.write(0);                   // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void accel_lib::acquire_accel_values()
{
    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address, NB_DATA * 2, true); //Get the 6th next register values (see doc)

    int16_t sensor_data[NB_DATA]; // Array for capturing accel values before calculations

    sensor_data[0] = Wire.read() << 8 | Wire.read(); // angles x
    sensor_data[1] = Wire.read() << 8 | Wire.read(); // angles y
    sensor_data[2] = Wire.read() << 8 | Wire.read(); // angles z

    noise_filters[0].addData(sensor_data[0]);
    noise_filters[1].addData(sensor_data[1]);
    noise_filters[2].addData(sensor_data[2]); 
}

short accel_lib::get_accel_values_processed(short index)
{
    return noise_filters[index].getCurrAverage();
}

short accel_lib::get_accel_values(short index)
{
    return noise_filters[index].getLastData();
}

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
    //currIndex = currIndex >= NOISE_FILTER_SIZE? 0 : currIndex;
    currIndex = (currIndex+1) % NOISE_FILTER_SIZE;
}

short movingAverage::getCurrAverage()
{
    return sum / NOISE_FILTER_SIZE;
}

short movingAverage::getLastData()
{
    return datas[currIndex];
}
