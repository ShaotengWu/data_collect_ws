#define CAN_Device USBCAN_E_1CH
#include <iostream>
#include <ros/ros.h>
#include "CanMsgDefs.h"
#include "CanCommonFcn.h"
#include "CanHandler.h"
#include "ICANCmd.h"
#include "canstream.h"
#include "rtknavigation.h"
#include "uwb.h"
#include "math.h"


#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

using namespace CanProcess;

DWORD dwDeviceHandle1;
DWORD dwDeviceHandle2;
CAN_InitConfig config;
std::vector<canstreamPtr> vcanstreamPtr;

// 第一次触发方向盘控制
bool first_hit = true;

// 定义状态全局变量
double global_curSpd = 0.0;
double global_curAngle = 0.0;
double global_preAngle = 0.0;

// 定义控制全局变量
double global_cmd_accel = 0.0;
double global_cmd_angle = 0.0;
int global_cmd_D_R = 2;

// 接受cmd，并存储等待发送
void cmdCallback(const geometry_msgs::Vector3::ConstPtr &cmdMsg);

// 调试模型，记得改过来
bool debug = false; 

// 使用案例
int main(int argc, char **argv)
{
    ros::init(argc, argv, "CAN");
    
    ros::NodeHandle n;
    ros::Subscriber subCmd = n.subscribe("/carCmd", 1, &cmdCallback);
    ros::Publisher pubSpd = n.advertise<geometry_msgs::TwistStamped>("/current_vehicle_can_info", 1, true);

    // 初始化消息
    initRecvMessage();
    initSendMessages();

    // 打开CAN口
    CAN_InitConfig CanConfig;
    CanHandler CanIO(CAN_Device, 1, 3, CanConfig);
    DWORD opened;
    opened = CanIO.OpenDevice();

    /*****************************************************************************************/
    // 打开第二个CHANNEL
    CAN_InitConfig config;
    config.dwAccCode = 0;
    config.dwAccMask = 0xffffffff;
    config.nFilter = 0;
    config.bMode = 0;
    config.nBtrType = 1;
    config.dwBtr[0] = 0x00; // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
    config.dwBtr[1] = 0x1c; // BTR1
    config.dwBtr[2] = 0;
    config.dwBtr[3] = 0;
    if (CAN_ChannelStart(opened, 1, &config) != CAN_RESULT_OK)
    {
        ROS_ERROR_STREAM(" >>Init CAN1 error!");
        return 0;
    }
    else
    {
        std::cout << "Init CAN11 success : " << opened << "," << 1 << std::endl;
        vcanstreamPtr.push_back(boost::shared_ptr<canstream>{new canstream(opened, 1)});
    }

    rtknavigation_Ptr prtknavigation = NULL;
    uwb_Ptr uwbPtr = NULL;
    /******************************************************************************************/

    // 获取这几个参数的地址， 用于后续控制发送什么消息
    send_args *send = &sendArg;
    send_args *send50 = &sendArg50;
    recv_args *recv = &recvArg;
    recv->channel = 0; // 0 是channel 0,，默认为0

    // 设置发送进程的间隔， 单位是毫秒上升沿
    send->interval = 20;
    send50->interval = 5000;

    // 进入自动驾驶模式
    CAN_Message *msg0x111 = &all_can_send[0x111];
    if (opened != 0)
    {
        // 第一次发送,发送5次
        send->num_frames = 4;
        send->data = new CAN_DataFrame[send->num_frames];
        send_all_safe(send);
        send->msg_arrived = true;
        ros::Rate r(50);
        for (int i=0;i<9;i++)
        {
        send_all_safe(send);
        r.sleep();
        }
        
        // 第二次发送
        double angle = getTurn();
        //LOG(INFO) << "Initial Angle: " << angle << "\n\n";
        setTurn(angle);
        enterTurn();
        enterDrive();
        enterGear();
        setGear(global_cmd_D_R);
        for (int i=0;i<9;i++)
        {
        send_all_safe(send);
        r.sleep();
        }
       // LOG(INFO) << "Turn Status: " << getTurnStatus() << "\n\n";
    }
    
    // 循环接受指令，发布速度
    ros::Rate loop_rate(100); // 休眠了两次，所以实际为50Hz

    while (ros::ok())
    {
        if (opened == 0)
        {
            ROS_ERROR("THE CAN IS NOT OPEN!!!");
            continue;
        }

        // 从底层读取方向盘转角并记录
        if (first_hit)
        {
            global_preAngle = getTurn();
            first_hit = false;
        }
        else
        {
            global_preAngle = global_curAngle; 
        }
        global_curAngle = getTurn();

        // 从底层读取速度并发布
        global_curSpd = getSpd();
        geometry_msgs::TwistStamped msg;
        msg.twist.linear.x  = global_curSpd;
        msg.twist.angular.z = global_curAngle;
        msg.header.stamp    = ros::Time::now();

        pubSpd.publish(msg);
        loop_rate.sleep();

        // 向can发送报文
        // setAcc(0.0);
        setAcc(global_cmd_accel);                                               // 设置加速度
        // double angleUpperBound = global_curAngle + 100.0;                       // 限制转向角在当前转向角正负10°内
        // double angleLowerBound = global_curAngle - 100.0;
        double angleUpperBound = global_curAngle + 90.0;                       
        double angleLowerBound = global_curAngle - 90.0;
        global_cmd_angle = global_cmd_angle > angleUpperBound ? angleUpperBound : global_cmd_angle;
        global_cmd_angle = global_cmd_angle < angleLowerBound ? angleLowerBound : global_cmd_angle;
        setTurn(global_cmd_angle);                                              // 设置加速度方向盘转角 deg
        // setGear(2);     
        setGear(global_cmd_D_R);                                                // 设置档位 0-P 1-R 2-N 3-D
        // LOG(INFO) << "Cmd Accel: " << global_cmd_accel << "\n";
        // LOG(INFO) << "Cmd Angle: " << global_cmd_angle << "\n";    
        // LOG(INFO) << "Cmd Gear: "  << global_cmd_D_R   << "\n\n";       
        //LOG(INFO) << "Turn Status: " << getTurnStatus() << "\n\n";                               
        send_all(send->data);   // or send_all_safe(send);
        loop_rate.sleep();

        /************************************************************************************/
        // 从rtk/uwb接受数据并发送
        for (size_t i = 0; i < vcanstreamPtr.size(); i++)
        {
            canstreamPtr canPtr = vcanstreamPtr[i];
            if (canPtr != NULL)
            {
                ROS_INFO("can_io : %d , %d  \n", canPtr->dwDeviceHandle_, canPtr->can_id_);
                // 接收can报文
                
                // usleep(100);
                //kongjian
                if ((canPtr->reclen_ = CAN_ChannelReceive(canPtr->dwDeviceHandle_, canPtr->can_id_, canPtr->rec_, 30, 10)) > 0)
                {
                    ROS_INFO("%s receive msg num : %d \n", canPtr->sensor_name_.c_str(), canPtr->reclen_);
                    // 判断报文是rtk还是uwb
                    if (canPtr->data_in())
                    {
                        // rtk报文
                        if (canPtr->sensor_name_ == "rtknavigation")
                        {
                            if (!prtknavigation)
                            {
                                prtknavigation = boost::shared_ptr<rtknavigation>{new rtknavigation(n, canPtr)};
                                prtknavigation->publishrtk();
                            }
                            else
                                prtknavigation->publishrtk();
                        }

                        // uwb报文
                        if (canPtr->sensor_name_ == "uwb")
                        {
                            ROS_INFO("uwb报文   \n");
                            if (!uwbPtr)
                            {
                                uwbPtr = boost::shared_ptr<uwb>{new uwb(n, canPtr)};
                                uwbPtr->publishrtk();
                            }
                            else
                                uwbPtr->publishrtk();
                        }
                    }
                }
                else
                {
                    CAN_ErrorInformation err;
                    if (CAN_GetErrorInfo(canPtr->dwDeviceHandle_, canPtr->can_id_, &err) == CAN_RESULT_OK)
                    {
                        // LOG(INFO) << "Error ouccured!!\n"; // process err
                    }
                    else
                    {
                        // LOG(INFO) << "no can msg received\n";
                    }
                }
                CAN_ClearReceiveBuffer(canPtr->dwDeviceHandle_, canPtr->can_id_);

            }
        }

        canToHuman(CanIO.recv[0], CanIO.reclen);
        ros::spinOnce();

        // // // 可以直接从 all_can_recv中取出想要看的消息
        // LOG(INFO) << "accel data: " << all_can_recv[0x243].all_double_data["LongitudeAcce"].data << "\n\n";
        // LOG(INFO) << "FLWheelSpd: " << all_can_recv[0x243].all_double_data["FLWheelSpd"].data << "\n\n";
        // LOG(INFO) << "velocity: " << all_can_recv[0x240].all_double_data["VehicleSpd"].data << "\n\n";
        // LOG(INFO) << "EPS Streeing mode:  " << all_can_recv[0x237].all_int_data["EPS_StreeingMode"].data << "\n\n";
        //LOG(INFO) << "Current angle: " << getTurn() << "\n\n";
        /**************************************************************************************/
    }

    // 退出自动驾驶
    ROS_INFO("Program Exit!");
    exitDrive();
    exitTurn();
    exitGear();
    send_all(send->data);   // or send_all_safe(send);
    sleep(1);

    if (!ros::ok())
        exit(0);

    delete[] send->data;
    send->run = false;
    recv->run = false;
    CanIO.join();
    return 0;
}

void cmdCallback(const geometry_msgs::Vector3::ConstPtr &cmdMsg)
{
    ROS_INFO("CMD Received From PID");
    global_cmd_accel = (double) cmdMsg->x;
    global_cmd_angle = (double) cmdMsg->y;
    global_cmd_D_R   = (int)    cmdMsg->z;
    // std::cerr<<"\e[31m"<<"expected angle = "<<global_cmd_angle<<"\e[0m"<<std::endl;
}
