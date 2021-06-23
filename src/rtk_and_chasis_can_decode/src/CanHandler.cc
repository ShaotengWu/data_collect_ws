#include "CanHandler.h"
#include "ICANCmd.h"
#include "CanMsgDefs.h"
#include <iostream>

using namespace std;
namespace CanProcess
{
    struct recv_args;
    struct send_args;

    // 接受线程的参数
    recv_args recvArg;

    // 发送线程的参数， sendArg50 是间隔为50ms的线程， sendArg对应间隔为20ms的线程
    send_args sendArg;
    send_args sendArg50;

    // 用于测试的线程锁
    std::mutex testLock;
    int test = 0;

    // 用于控制线程阻塞时间，从而达到定频率运行的功能，
    auto now() { return std::chrono::steady_clock::now(); }
    // a为毫秒
    auto awake_time(int a) { return now() + std::chrono::milliseconds(a); }

    /**
     * Create a CAN processor
    */
    CanHandler::CanHandler()
    {
        recvThread = std::thread(&CanHandler::recvLoop, this, &recvArg);
        sleep(1);
        sendThread = std::thread(&CanHandler::sendLoop, this, &sendArg);
        sendThread50 = std::thread(&CanHandler::sendLoop, this, &sendArg50);
    }

    /**
     * Create a CAN processor
     * @param DevType 设备类型
     * @param Idex USB索引， 从0开始
     * @param Channel 对应CAN卡上的CAN通道， 1代表 CAN0， 2代表 CAN1， 3代表CAN0和CAN1
     * @param config CAN卡的设定， 具体参考CanHandler.h 
    */
    CanHandler::CanHandler(int DevType, int Idex, int Channel, CAN_InitConfig config) : DeviceType(DevType), DeviceIdx(Idex), DeviceChannel(Channel), Can_Config(config)
    {
        recvThread = std::thread(&CanHandler::recvLoop, this, &recvArg);
        printf("recv thread created!!");
        sleep(1);
        sendThread = std::thread(&CanHandler::sendLoop, this, &sendArg);
        sendThread50 = std::thread(&CanHandler::sendLoop, this, &sendArg50);
        printf("send thread created!!");
    }

    /**
     * 接受数据的线程
     * @arg 接受数据的参数
    */
    void CanHandler::recvLoop(recv_args *arg)
    {
        while (arg->run)
        {
            if (!DeviceHandle)
                continue;
            DWORD channel = arg->channel;
            reclen = 0;
            // get information from can channel
            {
                std::unique_lock<std::mutex> lock(recv_data_lock);
                reclen = CAN_ChannelReceive(DeviceHandle, channel, recv[channel], __countof(recv[channel]), 200);
            }
            if (reclen)
            {
                // LOG(INFO) << reclen << " messages received!\n";
                canToHuman(recv[channel], reclen);
            }
            else
            {
                CAN_ErrorInformation err;
                if (CAN_GetErrorInfo(DeviceHandle, channel, &err) == CAN_RESULT_OK)
                {
                    // LOG(INFO) << "Error ouccured!!\n"; // process err
                }
                else
                {
                    // LOG(INFO) << "no can msg received\n";
                }
            }
        }
        return;
    }

    /**
     * 发送数据的线程，
    */
    void CanHandler::sendLoop(send_args *arg)
    {
        while (arg->run)
        {
            if (!DeviceHandle)
                continue;
            if (!arg->msg_arrived)
                continue;
            // LOG(INFO) << "Message Arrived! \n Send interval: " << arg->interval << "\n";
            // LOG(INFO) << "Message Count: " << arg->num_frames;
            int msgSend = 0;
            {
                std::unique_lock<mutex> lock(arg->datalock);
                //! Send control msg to chasis
                  msgSend = CAN_ChannelSend(DeviceHandle, arg->channel - 1, arg->data, arg->num_frames);
            }
            // LOG(INFO) << msgSend << " messages were sent!\n";
            // for (int i = 0; i < arg->num_frames; i++)
            // {
            //     char str_x[100];
            //     sprintf(str_x, "message %04X is send to CAN%d, Content is: ", arg->data[i].uID, arg->channel);
            //     LOG(INFO) << str_x;
            //     for (int j = 0; j < arg->data[i].nDataLen; j++)
            //         printf("%02X ", arg->data[i].arryData[j]);
            //     printf("\n");
            // }

            this_thread::sleep_until(awake_time(arg->interval));
        }
        return;
    }

}