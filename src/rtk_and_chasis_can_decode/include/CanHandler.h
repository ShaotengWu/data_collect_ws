#ifndef CanHandler_H
#define CanHandler_H

#include <thread>
#include <mutex>
#include <iostream>
#include "ICANCmd.h"
#include <glog/logging.h>

// CanProcess命名空间， 使用 using namespace CanProcess; 或者 CanProcess::something
namespace CanProcess
{
    // 接受线程的参数， run默认值必须为 true， 否则将立即退出接受线程
    struct recv_args
    {
        bool run = true;
        DWORD channel = 0;
    };
    extern recv_args recvArg;

    // 发送线程的参数， run同理
    struct send_args
    {
        bool run = true;
        std::mutex datalock; //
        CAN_DataFrame *data = nullptr;
        DWORD sndType = 2;
        DWORD channel = 1;
        bool msg_arrived = false;
        int num_frames = 4;
        int interval = 20;
    };

    //  sendArg50 是间隔为50ms的线程， sendArg对应间隔为20ms的线程
    extern send_args sendArg;
    extern send_args sendArg50;

    // 用于测试的线程锁
    extern std::mutex testLock;
    extern int test;

    class CanHandler
    {
    public:
        /**
         * Create a CAN processor
        */
        CanHandler();

        /**
         * Create a CAN processor
         * @param DevType 设备类型
         * @param Idex USB索引， 从0开始
         * @param Channel 对应CAN卡上的CAN通道， 1代表 CAN0， 2代表 CAN1， 3代表CAN0和CAN1
         * @param config CAN卡的设定， 具体参考CanHandler.h 
        */
        CanHandler(int DevType, int Idex, int Channel, CAN_InitConfig config); // Channel为选择第几路CAN

        /**
         * 析构函数， 关闭Can通道
        */
        ~CanHandler()
        {
            if ((DeviceChannel & 1) == 1)
            {
                printf("CAN_ChannelStop 0\r\n");
                CAN_ChannelStop(DeviceHandle, 0);
            }

            if ((DeviceChannel & 2) == 2)
            {
                printf("CAN_ChannelStop 1\r\n");
                CAN_ChannelStop(DeviceHandle, 1);
            }

            CAN_DeviceClose(DeviceHandle);
        }

        /**
         * 开启CAN卡设备， 并激活相应的通道 （参数是构造函数确定的）
        */
        DWORD OpenDevice()
        {
            DeviceHandle = CAN_DeviceOpen(DeviceType, DeviceIdx, 0);
            std::cout<<"device usb id:"<<DeviceIdx<<std::endl;
            if (!DeviceHandle)
                LOG(ERROR) << "Open Device Error\n";
            else
                LOG(INFO) << "Device Opened\n";

            if ((DeviceChannel & 1) == 1)
            {
                if (CAN_ChannelStart(DeviceHandle, 0, &Can_Config) != CAN_RESULT_OK)
                    LOG(ERROR) << "Start CAN " << DeviceChannel - 1 << "ERROR\n";
                else
                    LOG(INFO) << "CAN " << DeviceChannel - 1 << "Started!!\n";
            }

            if ((DeviceChannel & 2) == 2)
            {
                if (CAN_ChannelStart(DeviceHandle, 1, &Can_Config) != CAN_RESULT_OK)
                    LOG(ERROR) << "Start CAN " << DeviceChannel - 1 << "ERROR\n";
                else
                    LOG(INFO) << "CAN " << DeviceChannel - 1 << "Started!!\n";
            }

            if (CAN_GetDeviceInfo(DeviceHandle, &DevInfo) != CAN_RESULT_OK)
            {
                LOG(ERROR) << "GetDeviceInfo error\n";
            }
            sleep(1);
            return DeviceHandle;
        }

        /**
         * 接受数据的线程
         * @arg 接受数据的参数
        */
        void recvLoop(recv_args *arg);

        /**
         * 发送数据的线程，
        */
        void sendLoop(send_args *arg);

        /**
         * 等待发送和接受结束， 作用类似于 ros::Spin();
        */
        void join()
        {
            recvThread.join();
            sendThread.join();
            sendThread50.join();
        }

    private:
        // 线程们
        std::thread recvThread;
        std::thread sendThread;
        std::thread sendThread50;

    public:
        // 接收到的数据
        std::mutex recv_data_lock;
        CAN_DataFrame recv[2][100];
        int reclen = 0;


    private:
        DWORD DeviceHandle = 0;
        DWORD DeviceType = USBCAN_E_1CH;
        DWORD DeviceIdx = 1;
        DWORD DeviceChannel = 1;

        CAN_InitConfig Can_Config;

        CAN_DeviceInformation DevInfo;

        // testCan程序中遗留的参数，似乎没什么用
        DWORD CanSendType;
        DWORD CanSendFrames;
        DWORD CanSendTimes;
        DWORD CanSendDelay;
    };

    /**
     * Encode data to can message
     * @param data 可以选择不同类型的数据， 例如 bool int double 等等
     * @param lsb @param msb @param size @param offset @param factor 与 excel表中的消息属性对应
    */
    template <typename T>  // 模板函数必须定义在头文件中， 否则无法链接
    void encodeMsg(T data, int lsb, int msb, int size, double offset, double factor, BYTE *output)
    {
        data = (data - offset) / factor;
        // 确定起始的字节位置，也就是layout表上，消息开始的行数（不是bit位置）
        BYTE *startByte = output + msb / 8;
        int length = lsb / 8 - msb / 8 + 1;

        uint64_t temp(data);
        // 截断该数据， 防止数据溢出
        uint64_t i = (1 << size) - 1;
        temp &= i;

        // 移动位置到合适的位置（按照lsb移动，确保数据在合适的位置）
        temp <<= (lsb % 8);

        for (int i = 0; i < length; i++)
        {
            int idx = length - 1 - i;
            startByte[idx] |= ((temp >> (8 * i)) & (255));
        }
    }

    /**
     * Decode can message to data, 上面全部反向操作就行了
     * @param data 可以选择不同类型的数据， 例如 bool int double 等等
     * @param lsb @param msb @param size @param offset @param factor 与 excel表中的消息属性对应
    */
    template <typename T>
    T decodeMsg(BYTE *msg, int lsb, int msb, int size, double offset, double factor)
    {
        uint64_t temp = 0;
        BYTE *startByte = msg + msb / 8;
        int length = lsb / 8 - msb / 8 + 1;

        for (int i = 0; i < length; i++)
        {
            int idx = length - 1 - i;
            temp += (startByte[i] << 8 * idx);
        }
        temp >>= (lsb % 8);
        uint64_t i = ((1 << size) - 1);
        temp &= i;
        return T((temp * factor + offset));
    }
}

#endif
