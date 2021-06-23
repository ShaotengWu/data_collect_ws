#ifndef CANCOMMONFCN_H
#define CANCOMMONFCN_H

#include "CanMsgDefs.h"

// 这里按照说明书定义了一些常用函数， 可以自行添加，
// 注意这些函数只修改了预存的数据，
// 并没有修改发送的数据
namespace CanProcess
{

    void send_by_id(DWORD id, CAN_DataFrame *data)
    {
        if (all_can_send.find(id) != all_can_send.end())
        {
            all_can_send[id].toDataFrame(data);
        }
    }

    void send_all(CAN_DataFrame *data)
    {
        int i = 0;
        for (auto &msg_pair : all_can_send)
        {
            auto &msg = msg_pair.second;
            msg.toDataFrame(data + i);
            i++;
        }
    }

    void send_all_safe(send_args *send)
    {
        if (send)
        {
            std::unique_lock<std::mutex> lock(send->datalock);
            send_all(send->data);
        }
    }

    // 转向类
    /**
     * 进入自动驾驶模式
    */
    void enterTurn()
    {
        all_can_send[0x113].all_int_data["ADS_EPSMode"].data = 2;
    }
    /**
     * 退出自动驾驶模式
    */
    void exitTurn()
    {
        all_can_send[0x113].all_int_data["ADS_EPS_Mode"].data = 0;
    }

    /**
     * 发送控制信号
    */
    void setTurn(double angle)
    {
        all_can_send[0x113].all_double_data["ADS_ReqEPSTargetAngle"].data = angle;
    }

    /**
     * 获取方向盘转角
    */
    double getTurn()
    {
        return -all_can_recv[0x235].all_double_data["SteerWheelAngle"].data * (all_can_recv[0x235].all_bool_data["SteerWheelAngleSign"].data * 2 - 1);
    }

    /**
     * 获取状态
    */
    int getTurnStatus()
    {
        return all_can_recv[0x237].all_int_data["EPS_StreeingMode"].data;
    }

    // 驱动类
    /**
     * 进入
    */
    void enterDrive()
    {
        all_can_send[0x111].all_bool_data["ADS_Driveoff_Req"].data = 1;
        all_can_send[0x111].all_int_data["ADS_Mode"].data = 3;
    }

    void exitDrive()
    {
        all_can_send[0x111].all_bool_data["ADS_DecToStop"].data = 1;
        all_can_send[0x111].all_bool_data["ADS_Driveoff_Req"].data = 0;
        all_can_send[0x111].all_int_data["ADS_Mode"].data = 0;
    }

    void setAcc(double data)
    {
        all_can_send[0x111].all_double_data["ADS_TarAcce"].data = data;
    }

    double getAcc()
    {
        return all_can_recv[0x243].all_double_data["LongitudeAcce"].data;
    }
    
    double getSpd()
    {
        return all_can_recv[0x240].all_double_data["VehicleSpd"].data;
    }

    int getDriveStatus()
    {
        return all_can_recv[0x310].all_int_data["LongitudeDrivingMode"].data;
    }

    void AEBStop()
    {
        all_can_send[0x111].all_bool_data["ADS_AEB_TgtDecel_Req"].data = 1;
        all_can_send[0x111].all_double_data["ADS_AEB_TarAcce"].data = 2;
    }

    // 换挡类

    void enterGear()
    {
        all_can_send[0x115].all_int_data["ADS_ShiftMode"].data = 1;
    }

    void exitGear()
    {
        all_can_send[0x115].all_int_data["ADS_ShiftMode"].data = 0;
    }

    void setGear(int tarGear)
    {
        all_can_send[0x115].all_int_data["ADS_TargetGear"].data = tarGear;
    }

    // 车身类

    void enterBCM()
    {
        all_can_send[0x38E].all_int_data["ADS_BCM_WorkSts"].data = 2;
        all_can_send[0x38E].all_bool_data["ADS_BCMWorkStsValid"].data = 1;
        all_can_send[0x38E].all_bool_data["ADS_ReqControlBCM"].data = 1;
    }

    void exitBCM()
    {
        all_can_send[0x38E].all_bool_data["ADS_ReqControlBCM"].data = 0;
    }

    void exitAll()
    {
        exitDrive();
        exitTurn();
        exitGear();
        exitBCM();
    }
}
#endif