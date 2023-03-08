#pragma once

#define NOISE_FILTER_SIZE 50
#define NB_DATA 3

namespace arm_us
{
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

    class accel_lib
    {
    private:
        int address = 0;
        movingAverage noise_filters[NB_DATA];

    public:
        accel_lib(int i_address);
        
        void init();
        void acquire_accel_values();
        short get_accel_values_processed(short index);
        short get_accel_values(short index);
    };

}
