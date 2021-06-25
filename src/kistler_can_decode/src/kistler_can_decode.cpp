#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "ICANCmd.h"
#include "kistler_msgs/kistler.h"

using namespace std;
#define NEWFLAG -100000
void kistlerDataReset(kistler_msgs::kistler &kistlerData)
{
    kistlerData.sensor_time = NEWFLAG;
    kistlerData.vel = NEWFLAG;
    kistlerData.distance = NEWFLAG;
}

bool kistlerDataIsNew(const kistler_msgs::kistler &kistlerData)
{
    return (kistlerData.sensor_time != NEWFLAG &&
            kistlerData.vel != NEWFLAG &&
            kistlerData.distance != NEWFLAG);
}

/**
 * @brief ONLY FOR KISTLER
 * ! data都是整字节长度 INTEL 顺序
 * @param mode: 0 - 原码, 1 - 反码
 */
double canDecode(CAN_DataFrame rawData, int lsb, int length, double factor, double offset, int mode)
{
    int lsbByte = lsb / 8;
    int lsbBit = lsb % 8;
    int msbBit = (lsbBit + length - 1) % 8;
    int msbByte = (lsb + length - 1) / 8;

    // printf("num_byte : %d ,lsb_byte : %d, lsb_bit : %d, msb_bit : %d \n",num_byte, lsb_byte, lsb_bit, msb_bit);
    //std::cout<<"lsb byte : "<<lsbByte<<"\t lsb bit  "<<lsbBit<<"\t msb byte : "<<msbByte<<"\t msb bit : "<<msbBit<<"\t length: "<<length<<std::endl;
    int data = 0;
    int deviation = 0;

    for (int iByte = lsbByte; iByte <= msbByte; iByte++)
    {
        data += (int)((rawData.arryData[iByte]) << deviation);
        deviation += 8;
    }
    if (mode == 1)
    {
        int tmp = round(pow(2, msbBit));
        if ((rawData.arryData[msbByte] & tmp) == tmp)
        {
            data = (-1) * (pow(2, length) - data - 1);
        }
    }

    data = factor * data + offset;
    return data;
}

DWORD dwDeviceHandle;
CAN_InitConfig config;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kistler");
    ros::NodeHandle nh;

    if ((dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B, 0, 0)) == 1)
    {
        ROS_INFO_STREAM(" >>open device success!");
    }
    else
    {
        ROS_ERROR_STREAM(" >>open device error!");
        return 0;
        exit(1);
    }

    CAN_InitConfig config;
    config.dwAccCode = 0;
    config.dwAccMask = 0xffffffff;
    config.nFilter = 0;     // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
    config.bMode = 0;       // 工作模式(0表示正常模式,1表示只听模式)
    config.nBtrType = 1;    // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
    config.dwBtr[0] = 0x00; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
    config.dwBtr[1] = 0x1c; // BTR1
    config.dwBtr[2] = 0;
    config.dwBtr[3] = 0;

    if (CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK)
    {
        ROS_ERROR_STREAM(" >>Init CAN0 error!");

        return 0;
    }

    if (CAN_ChannelStart(dwDeviceHandle, 1, &config) != CAN_RESULT_OK)
    {
        ROS_ERROR_STREAM(" >>Init CAN1 error!");

        return 0;
    }

    int reclen = 0;
    CAN_DataFrame rec[3000]; //buffer
    int CANInd = 0;          //CAN1=0, CAN2=1

    // ros::Publisher rtk_pub = nh.advertise<location_msgs::RTK>("rtk_data", 1000);
    ros::Publisher kistler_pub = nh.advertise<kistler_msgs::kistler>("/kistler_data", 1000);

    kistler_msgs::kistler kistler_data;
    kistlerDataReset(kistler_data);

    bool init = 0;

    while (ros::ok())
    {
        // ROS_INFO("begin collect can data");
        if ((reclen = CAN_ChannelReceive(dwDeviceHandle,0, rec, 3000, 10)) > 0) //调用接收函数,得到数据
        {
            // ROS_INFO("begin decode");
            for (int i = 0; i < reclen; i++)
            {

                // kistler_data
                if (rec[i].uID == 0x7E2)
                {
                    kistler_data.pitch       = canDecode(rec[i], 0, 16, 1e-2, 0,1 );
                    kistler_data.roll        = canDecode(rec[i], 16, 16, 1e-2, 0, 1);
                    kistler_data.sensor_time = canDecode(rec[i], 32, 16, 1.0, 0, 0);
                    kistler_data.radius      = canDecode(rec[i], 48, 16, 1e-2, 0, 0);
                }
                if (rec[i].uID == 0x7E1)
                {
                    kistler_data.distance = canDecode(rec[i], 0, 32, 1e-3, 0, 1);
                }
                if (rec[i].uID == 0x7E0)
                {
                    kistler_data.velX  = canDecode(rec[i], 0, 16, 0.036, 0, 1);
                    kistler_data.velY  = canDecode(rec[i], 16, 16, 0.036, 0, 1);
                    kistler_data.vel   = canDecode(rec[i], 32, 16, 0.036, 0, 1);
                    kistler_data.angle = canDecode(rec[i], 48, 16, 1e-2, 0, 1);
                }
                if (rec[i].uID == 0x7E7)
                {
                    kistler_data.angVelBodyX = canDecode(rec[i], 0, 16, 0.02, 0, 1);
                    kistler_data.angVelBodyY = canDecode(rec[i], 16, 16, 0.02, 0, 1);
                    kistler_data.angVelBodyZ = canDecode(rec[i], 32, 16, 0.02, 0, 1);
                }
            }
            // ROS_INFO("finished decode");
        }

        if (kistlerDataIsNew(kistler_data))
        {
            // ROS_INFO("begin publish");
            kistler_data.ros_time = ros::Time::now();

            kistler_pub.publish(kistler_data);
            kistlerDataReset(kistler_data);
            // ROS_INFO("finished publish");
        }

        // ROS_INFO("enter next loop");
    }
    CAN_DeviceClose(dwDeviceHandle);
    return 0;
}
