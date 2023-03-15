#include "Arduino.h"
#include "Wire.h"
// #include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include "ros.h"
#include "arm_us/accel_pos.h"

namespace arm_us
{
    enum FifoState
    {
        ok,
        overflow,
        sync_error
    };

    class MpuDmp : public MPU6050
    {
    private:
        FifoState fifo_state_ = ok;
        uint8_t fifo_buffer_[1024];
        Quaternion last_data_;
        int packet_size_ = 42;
        int fifo_count_ = 0;

    public:
        MpuDmp(uint8_t addresse) : MPU6050(addresse){};
        void init(int xao, int yao, int zao, int xgo, int ygo, int zgo);
        FifoState refreshLoop();
        Quaternion getLastData() { return last_data_; };
    };

    void MpuDmp::init(int xao, int yao, int zao, int xgo, int ygo, int zgo)
    {
        // initialize();
        // dmpInitialize();

        // Those value come from running the calibration code in MPU6050's library: "/examples/IMU_Zero/IMU_Zero.ino"
        setXAccelOffset(xao);
        setYAccelOffset(yao);
        setZAccelOffset(zao);
        setXGyroOffset(xgo);
        setYGyroOffset(ygo);
        setZGyroOffset(zgo);
        setDMPEnabled(true);
        packet_size_ = dmpGetFIFOPacketSize();
        fifo_count_ = getFIFOCount();
    }

    FifoState MpuDmp::refreshLoop()
    {
        resetFIFO();
        fifo_count_ = getFIFOCount();

        while (fifo_count_ < packet_size_)
        {
            fifo_count_ = getFIFOCount();
        }

        if (fifo_count_ == 1024)
        {
            resetFIFO();
            fifo_state_ = overflow;
        }
        else
        {
            if (fifo_count_ % packet_size_ != 0)
            {
                resetFIFO();
                fifo_state_ = sync_error;
            }
            else
            {
                while (fifo_count_ >= packet_size_)
                {
                    getFIFOBytes(fifo_buffer_, packet_size_);
                    fifo_count_ -= packet_size_;
                }

                dmpGetQuaternion(&last_data_, fifo_buffer_);
                fifo_state_ = ok;
            }
        }

        return fifo_state_;
    }
}

#define ACCEL_ADDRESS_SHOULDER 0x69
#define ACCEL_ADDRESS_WRIST 0x68
#define NB_ACCEL 2
#define FLEX_SENSOR_PIN A2
#define ACCEL_ARRAY_SIZE 4
const unsigned short MSG_PERIOD_MS = 100;

// Fct prototypes
void refreshAccels();
void send_msg(arm_us::MpuDmp *accelerometer, ros::Publisher *pub);

ros::NodeHandle n;

arm_us::accel_pos accel_msg;
ros::Publisher pub_wrist("pos_wrist", &accel_msg);
ros::Publisher pub_shoulder("pos_shoulder", &accel_msg);

arm_us::MpuDmp mpu_wrist(0x68);
arm_us::MpuDmp mpu_shoulder(0x69);

// arm_us::MpuDmp *mpus[NB_ACCEL] = {&mpu_shoulder, &mpu_wrist};

void setup()
{
    // ROS setup
    n.initNode();
    n.advertise(pub_shoulder);
    n.advertise(pub_wrist);

    Wire.begin();
    mpu_wrist.dmpInitialize();
    mpu_shoulder.dmpInitialize();

    // [-1167,-1167] --> [-6,27]	[-31,-31] --> [-10,88]	[2312,2313] --> [16354,16385]	[-40,-39] --> [-2,1]	[-101,-100] --> [0,1]	[-55,-54] --> [-1,3]
    mpu_wrist.init(-1167, -31, 2313, -39, -101, -55);
    // [-1891,-1890] --> [-25,6]	[-1443,-1443] --> [0,3]	[2468,2469] --> [16373,16412]	[-39,-38] --> [-4,4]	[-101,-100] --> [-2,1]	[-55,-54] --> [0,3]
    mpu_shoulder.init(-1890, -1443, 2469, -38, -100, -55);
}

void loop()
{
    n.spinOnce();
    mpu_wrist.refreshLoop();
    mpu_shoulder.refreshLoop();

    send_msg(&mpu_shoulder, &pub_shoulder);
    send_msg(&mpu_wrist, &pub_wrist);
    // arm_us::accel_pos msg;
    // Quaternion q = mpu_wrist.getLastData();
    // msg.w = q.w;
    // msg.x = q.x;
    // msg.y = q.y;
    // msg.z = q.z;
    // pub_wrist.publish(&msg);

    // q = mpu_shoulder.getLastData();
    // msg.w = q.w;
    // msg.x = q.x;
    // msg.y = q.y;
    // msg.z = q.z;
    // pub_shoulder.publish(&msg);
}

void refreshAccels()
{
    // arm_us::FifoState accel_state = arm_us::ok;
    // for (auto i = 0; i < NB_ACCEL; i++)
    // {
    //     accel_state = mpus[i]->refreshLoop();

    //     if (accel_state == arm_us::ok)
    //     {
    //         break;
    //     }
    //     else if (accel_state == arm_us::overflow)
    //     {
    //         n.logwarn("MPU DMP Fifo Buffer Overflow --> Buffer reseted");
    //     }
    //     else
    //     {
    //         n.logwarn("Fifo Buffer sync error --> Buffer reseted");
    //     }
    // }
}

void send_msg(arm_us::MpuDmp *accelerometer, ros::Publisher *pub)
{
    arm_us::accel_pos msg;
    Quaternion q = accelerometer->getLastData();
    msg.w = q.w;
    msg.x = q.x;
    msg.y = q.y;
    msg.z = q.z;

    pub->publish(&msg);
}
