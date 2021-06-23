#include "CanMsgDefs.h"
#include "map"

namespace CanProcess
{
    std::map<DWORD, CAN_Message> all_can_send;
    std::map<DWORD, CAN_Message> all_can_recv;

    void initSendMessages()
    {
        DWORD ids[4] = {0x111, 0x113, 0x115, 0x38E};
        for (auto i : ids)
        {
            all_can_send[i] = CAN_Message(i);
        }
        // all messages in message 0x111
        CAN_Message *msg0x111 = &all_can_send[0x111];
        msg0x111->all_int_data["ADS_Mode"] = SingleMsg_int("ADS_Mode", 3, 7, 5);
        msg0x111->all_double_data["ADS_TarAcce"] = SingleMsg_double("ADS_TarAcce", 8, 15, 8, -7, 0.05);
        msg0x111->all_bool_data["ADS_Driveoff_Req"] = SingleMsg_bool("ADS_Driveoff_Req", 1, 1, 1);
        msg0x111->all_double_data["ADS_AEB_TarAcce"] = SingleMsg_double("ADS_AEB_TarAcce", 40, 39, 16, -16, 0.00049);
        msg0x111->all_bool_data["ADS_DecToStop"] = SingleMsg_bool("ADS_DecToStop", 17, 17, 1, 0, 1);
        msg0x111->all_bool_data["ADS_AEB_TgtDecel_Req"] = SingleMsg_bool("ADS_AEB_TgtDecel_Req", 31, 31, 1);

        // all message in message 0x113
        CAN_Message *msg0x113 = &all_can_send[0x113];
        msg0x113->all_int_data["ADS_EPSMode"] = SingleMsg_int("ADS_EPSMode", 6, 7, 2);
        msg0x113->all_double_data["ADS_ReqEPSTargetAngle"] = SingleMsg_double("ADS_ReqEPSTargetAngle", 18, 15, 16, -800, 0.1);

        // all message in message 0x115
        CAN_Message *msg0x115 = &all_can_send[0x115];
        msg0x115->all_int_data["ADS_ShiftMode"] = SingleMsg_int("ADS_ShiftMode", 6, 7, 2);
        msg0x115->all_int_data["ADS_TargetGear"] = SingleMsg_int("ADS_TargetGear", 17, 19, 3);

        // all message in message 0x38E
        CAN_Message *msg0x38E = &all_can_send[0x38E];
        msg0x38E->all_int_data["ADS_BCM_WorkSts"] = SingleMsg_int("ADS_BCM_WorkSts", 5, 6, 2);
        msg0x38E->all_bool_data["ADS_BCMWorkStsValid"] = SingleMsg_bool("ADS_BCMWorkStsValid", 7, 7, 1);
        msg0x38E->all_bool_data["ADS_ReqControlBCM"] = SingleMsg_bool("ADS_ReqControlBCM", 8, 8, 1);
        msg0x38E->all_bool_data["HighBeamON"] = SingleMsg_bool("HighBeamON", 11, 11, 1);
        msg0x38E->all_bool_data["DippedBeamON"] = SingleMsg_bool("DippedBeamON", 12, 12, 1);
        msg0x38E->all_int_data["TurnLightON"] = SingleMsg_int("TurnLightON", 16, 17, 2);
        msg0x38E->all_bool_data["EmergencyLightON"] = SingleMsg_bool("EmergencyLightON", 45, 45, 1);
        msg0x38E->all_bool_data["FFogLampON"] = SingleMsg_bool("FFogLampON", 46, 46, 1);
        msg0x38E->all_bool_data["RFogLampON"] = SingleMsg_bool("RFogLampON", 47, 47, 1);
        msg0x38E->all_bool_data["BrakeLight"] = SingleMsg_bool("BrakeLight", 48, 48, 1);
        msg0x38E->all_bool_data["HornON"] = SingleMsg_bool("HornON", 49, 49, 1);
        msg0x38E->all_bool_data["FWiper"] = SingleMsg_bool("FWiper", 50, 50, 1);
        msg0x38E->all_bool_data["RWiper"] = SingleMsg_bool("RWiper", 60, 60, 1);
    }

    void initRecvMessage()
    {
        DWORD ids[7] = {0x243, 0x240, 0x237, 0x235, 0x201, 0x310, 0x241};
        for (auto i : ids)
        {
            all_can_recv[i] = CAN_Message(i);
        }

        // all message in message 0x243
        CAN_Message *msg0x243 = &all_can_recv[0x243];
        msg0x243->all_double_data["LongitudeAcce"] = SingleMsg_double("LongitudeAcce", 8, 7, 16, -21.592, 0.00098);
        msg0x243->all_double_data["LateralAcce"] = SingleMsg_double("LateralAcce", 24, 23, 16, -21.592, 0.00098);
        msg0x243->all_double_data["VehDynYawRate"] = SingleMsg_double("VehDynYawRate", 40, 39, 16, -2.093, 0.00023999999999999998);
        msg0x243->all_double_data["FLWheelSpd"] = SingleMsg_double("FLWheelSpd", 59, 55, 13, 0.0, 0.05625);
        msg0x243->all_int_data["FRWheelDirection"] = SingleMsg_int("FRWheelDirection", 56, 57, 2, 0.0, 1.0);

        // all message in message 0x240
        CAN_Message *msg0x240 = &all_can_recv[0x240];
        msg0x240->all_int_data["FLWheelDirection"] = SingleMsg_int("FLWheelDirection", 56, 57, 2, 0.0, 1.0);
        msg0x240->all_double_data["FRWheelSpd"] = SingleMsg_double("FRWheelSpd", 11, 7, 13, 0.0, 0.05625);
        msg0x240->all_int_data["RLWheelDriveDirection"] = SingleMsg_int("RLWheelDriveDirection", 8, 9, 2, 0.0, 1.0);
        msg0x240->all_double_data["RLWheelSpd"] = SingleMsg_double("RLWheelSpd", 27, 23, 13, 0.0, 0.05625);
        msg0x240->all_int_data["RRWheelDirection"] = SingleMsg_int("RRWheelDirection", 24, 25, 2, 0.0, 1.0);
        msg0x240->all_double_data["RRWheelSpd"] = SingleMsg_double("RRWheelSpd", 43, 39, 13, 0.0, 0.05625);
        msg0x240->all_double_data["VehicleSpd"] = SingleMsg_double("VehicleSpd", 59, 55, 13, 0.0, 0.05625);

        // all message in message 0x237
        CAN_Message *msg0x237 = &all_can_recv[0x237];
        msg0x237->all_double_data["EngSpd"] = SingleMsg_double("EngSpd", 8, 7, 16, 0.0, 0.125);
        msg0x237->all_double_data["AccPedalPos"] = SingleMsg_double("AccPedalPos", 16, 23, 8, 0.0, 0.3937);
        msg0x237->all_int_data["EPBSwtichPosition"] = SingleMsg_int("EPBSwtichPosition", 30, 31, 2, 0.0, 1.0);
        msg0x237->all_int_data["EPS_StreeingMode"] = SingleMsg_int("EPS_StreeingMode", 32, 34, 3, 0.0, 1.0);
        msg0x237->all_int_data["CurrentGear"] = SingleMsg_int("CurrentGear", 38, 39, 2, 0.0, 1.0);
        msg0x237->all_double_data["EPSDrvInputTrqValue"] = SingleMsg_double("EPSDrvInputTrqValue", 40, 47, 8, -22.78, 0.1794);
        msg0x237->all_double_data["EPSConsumedCurrValue"] = SingleMsg_double("EPSConsumedCurrValue", 48, 55, 8, 0.0, 0.5);
        msg0x237->all_int_data["EPSCurrMod"] = SingleMsg_int("EPSCurrMod", 59, 61, 3, 0.0, 1.0);

        // all message in message 0x235
        CAN_Message *msg0x235 = &all_can_recv[0x235];
        msg0x235->all_double_data["SteerWheelAngle"] = SingleMsg_double("SteerWheelAngle", 17, 15, 15, 0.0, 0.1);
        msg0x235->all_double_data["SteerWheelSpd"] = SingleMsg_double("SteerWheelSpd", 41, 39, 15, 0.0, 0.1);
        msg0x235->all_bool_data["SteerWheelSpdSign"] = SingleMsg_bool("SteerWheelSpdSign", 40, 40, 1, 0.0, 1.0);
        msg0x235->all_bool_data["SteerWheelAngleSign"] = SingleMsg_bool("SteerWheelAngleSign", 16, 16, 1, 0.0, 1.0);
        msg0x235->all_double_data["FL_EdgesSum"] = SingleMsg_double("FL_EdgesSum", 0, 7, 8, 0.0, 1.0);
        msg0x235->all_double_data["FR_EdgesSum"] = SingleMsg_double("FR_EdgesSum", 24, 31, 8, 0.0, 1.0);
        msg0x235->all_double_data["RL_EdgesSum"] = SingleMsg_double("RL_EdgesSum", 48, 55, 8, 0.0, 1.0);
        msg0x235->all_double_data["RR_EdgesSum"] = SingleMsg_double("RR_EdgesSum", 56, 63, 8, 0.0, 1.0);

        // all message in message 0x201
        CAN_Message *msg0x201 = &all_can_recv[0x201];
        msg0x201->all_int_data["TCS_Level"] = SingleMsg_int("TCS_Level", 4, 7, 4, 0.0, 1.0);
        msg0x201->all_int_data["ReversGearSts"] = SingleMsg_int("ReversGearSts", 14, 15, 2, 0.0, 1.0);
        msg0x201->all_int_data["VehicleStandstill"] = SingleMsg_int("VehicleStandstill", 22, 23, 2, 0.0, 1.0);
        msg0x201->all_bool_data["ACC_DecSetSpd_SetSw"] = SingleMsg_bool("ACC_DecSetSpd_SetSw", 28, 28, 1, 0.0, 1.0);
        msg0x201->all_bool_data["ACC_IncSetSpd_ResuSw"] = SingleMsg_bool("ACC_IncSetSpd_ResuSw", 29, 29, 1, 0.0, 1.0);
        msg0x201->all_bool_data["ACC_CancelSw"] = SingleMsg_bool("ACC_CancelSw", 30, 30, 1, 0.0, 1.0);
        msg0x201->all_bool_data["ACC_ON_OFFSw"] = SingleMsg_bool("ACC_ON_OFFSw", 31, 31, 1, 0.0, 1.0);
        msg0x201->all_int_data["WiperSwitchSts"] = SingleMsg_int("WiperSwitchSts", 41, 43, 3, 0.0, 1.0);
        msg0x201->all_bool_data["HazardLightSwSts"] = SingleMsg_bool("HazardLightSwSts", 45, 45, 1, 0.0, 1.0);
        msg0x201->all_bool_data["RightTurnSWSts"] = SingleMsg_bool("RightTurnSWSts", 46, 46, 1, 0.0, 1.0);
        msg0x201->all_bool_data["LeftTurnSWSts"] = SingleMsg_bool("LeftTurnSWSts", 47, 47, 1, 0.0, 1.0);
        msg0x201->all_bool_data["TrunkSts"] = SingleMsg_bool("TrunkSts", 51, 51, 1, 0.0, 1.0);
        msg0x201->all_bool_data["RRDoorSts"] = SingleMsg_bool("RRDoorSts", 52, 52, 1, 0.0, 1.0);
        msg0x201->all_bool_data["RLDoorSts"] = SingleMsg_bool("RLDoorSts", 53, 53, 1, 0.0, 1.0);
        msg0x201->all_bool_data["PassengerDoorSts"] = SingleMsg_bool("PassengerDoorSts", 54, 54, 1, 0.0, 1.0);
        msg0x201->all_bool_data["DriverDoorSts"] = SingleMsg_bool("DriverDoorSts", 55, 55, 1, 0.0, 1.0);
        msg0x201->all_bool_data["PFold_UnfoldSts"] = SingleMsg_bool("PFold_UnfoldSts", 59, 59, 1, 0.0, 1.0);
        msg0x201->all_bool_data["DFold_UnfoldSts"] = SingleMsg_bool("DFold_UnfoldSts", 60, 60, 1, 0.0, 1.0);
        msg0x201->all_bool_data["PassSBR"] = SingleMsg_bool("PassSBR", 62, 62, 1, 0.0, 1.0);
        msg0x201->all_bool_data["DriverSBR"] = SingleMsg_bool("DriverSBR", 63, 63, 1, 0.0, 1.0);

        // all message in message 0x310
        CAN_Message *msg0x310 = &all_can_recv[0x310];
        msg0x310->all_bool_data["LongitudeAccValid"] = SingleMsg_bool("LongitudeAccValid", 15, 15, 1, 0.0, 1.0);
        msg0x310->all_bool_data["LateralAcceValid"] = SingleMsg_bool("LateralAcceValid", 7, 7, 1, 0.0, 1.0);
        msg0x310->all_bool_data["VehDynYawRateValid"] = SingleMsg_bool("VehDynYawRateValid", 6, 6, 1, 0.0, 1.0);
        msg0x310->all_bool_data["FLWheelSpdValid"] = SingleMsg_bool("FLWheelSpdValid", 5, 5, 1, 0.0, 1.0);
        msg0x310->all_bool_data["FRWheelSpdValid"] = SingleMsg_bool("FRWheelSpdValid", 53, 53, 1, 0.0, 1.0);
        msg0x310->all_bool_data["RLWheelSpdValid"] = SingleMsg_bool("RLWheelSpdValid", 3, 3, 1, 0.0, 1.0);
        msg0x310->all_bool_data["RRWheelSpdValid"] = SingleMsg_bool("RRWheelSpdValid", 2, 2, 1, 0.0, 1.0);
        msg0x310->all_bool_data["VehicleSpdValid"] = SingleMsg_bool("VehicleSpdValid", 0, 0, 1, 0.0, 1.0);
        msg0x310->all_int_data["LongitudeDrivingMode"] = SingleMsg_int("LongitudeDrivingMode", 13, 14, 2, 0.0, 1.0);
        msg0x310->all_int_data["EngSpdValid"] = SingleMsg_int("EngSpdValid", 11, 12, 2, 0.0, 1.0);
        msg0x310->all_bool_data["AccePedalOverride"] = SingleMsg_bool("AccePedalOverride", 19, 19, 1, 0.0, 1.0);
        msg0x310->all_int_data["BrakePedalStatus"] = SingleMsg_int("BrakePedalStatus", 8, 9, 2, 0.0, 1.0);
        msg0x310->all_bool_data["ESPBrakeLightSts"] = SingleMsg_bool("ESPBrakeLightSts", 29, 29, 1, 0.0, 1.0);
        msg0x310->all_bool_data["EPBSwtPositionValid"] = SingleMsg_bool("EPBSwtPositionValid", 20, 20, 1, 0.0, 1.0);
        msg0x310->all_int_data["EPBSts"] = SingleMsg_int("EPBSts", 17, 18, 2, 0.0, 1.0);
        msg0x310->all_bool_data["CurrentGearValid"] = SingleMsg_bool("CurrentGearValid", 25, 25, 1, 0.0, 1.0);
        msg0x310->all_bool_data["EPSTrqSnsrSts"] = SingleMsg_bool("EPSTrqSnsrSts", 31, 31, 1, 0.0, 1.0);
        msg0x310->all_bool_data["EPS_InterferDetdValid"] = SingleMsg_bool("EPS_InterferDetdValid", 38, 38, 1, 0.0, 1.0);
        msg0x310->all_bool_data["EPSHandsDetnSts"] = SingleMsg_bool("EPSHandsDetnSts", 27, 27, 1, 0.0, 1.0);
        msg0x310->all_bool_data["EPS_HandsDetnStsValid"] = SingleMsg_bool("EPS_HandsDetnStsValid", 34, 34, 1, 0.0, 1.0);
        msg0x310->all_int_data["FrontFogLmpSts"] = SingleMsg_int("FrontFogLmpSts", 42, 43, 2, 0.0, 1.0);
        msg0x310->all_bool_data["RearFogLmpSts"] = SingleMsg_bool("RearFogLmpSts", 51, 51, 1, 0.0, 1.0);
        msg0x310->all_bool_data["LowBeamSts"] = SingleMsg_bool("LowBeamSts", 49, 49, 1, 0.0, 1.0);
        msg0x310->all_bool_data["HighBeamSts"] = SingleMsg_bool("HighBeamSts", 63, 63, 1, 0.0, 1.0);
        msg0x310->all_bool_data["LeftTurnLampSts"] = SingleMsg_bool("LeftTurnLampSts", 62, 62, 1, 0.0, 1.0);
        msg0x310->all_bool_data["RightTurnLampSts"] = SingleMsg_bool("RightTurnLampSts", 60, 60, 1, 0.0, 1.0);
        msg0x310->all_int_data["BCM_AvailSts"] = SingleMsg_int("BCM_AvailSts", 57, 58, 2, 0.0, 1.0);
        msg0x310->all_bool_data["BrakeLmpSts"] = SingleMsg_bool("BrakeLmpSts", 56, 56, 1, 0.0, 1.0);

        // all message in message 0x241
        CAN_Message *msg0x241 = &all_can_recv[0x241];
        msg0x241->all_bool_data["EngFail"] = SingleMsg_bool("EngFail", 7, 7, 1, 0.0, 1.0);
        msg0x241->all_bool_data["ESPFail"] = SingleMsg_bool("ESPFail", 14, 14, 1, 0.0, 1.0);
        msg0x241->all_int_data["EPBFail"] = SingleMsg_int("EPBFail", 34, 35, 2, 0.0, 1.0);
        msg0x241->all_int_data["ShiftFail"] = SingleMsg_int("ShiftFail", 28, 31, 4, 0.0, 1.0);
        msg0x241->all_bool_data["EPSFail"] = SingleMsg_bool("EPSFail", 21, 21, 1, 0.0, 1.0);

        // // rtk messages
        // CAN_Message *msg0x804 = &all_can_recv[0x804];
        // msg0x804->all_double_data["PosLat"] = SingleMsg_double("PosLat", )
    }

    void canToHuman(CAN_DataFrame *df, int size)
    {
        for (int i = 0; i < size; i++)
        {
            if (all_can_recv.find(df[i].uID) == all_can_recv.end())
                continue;

            CAN_Message *temp = &all_can_recv[df[i].uID];
            for (auto &msg_pair : temp->all_bool_data)
            {
                auto &msg = msg_pair.second;
                msg.data = decodeMsg<bool>(df[i].arryData, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor);
            }

            for (auto &msg_pair : temp->all_int_data)
            {
                auto &msg = msg_pair.second;
                msg.data = decodeMsg<int>(df[i].arryData, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor);
            }

            for (auto &msg_pair : temp->all_double_data)
            {
                auto &msg = msg_pair.second;
                msg.data = decodeMsg<double>(df[i].arryData, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor);
            }
        }
    }

    void CAN_Message::toDataFrame(CAN_DataFrame *dataframe)
    {
        if (!dataframe)
            return;
        dataframe->uID = id;
        for (int i = 0; i < 8; i++)
        {
            dataframe->arryData[i] = 0;
        }
        for (auto &msg_pair : all_double_data)
        {
            auto &msg = msg_pair.second;
            // LOG(INFO) << "double message encoded: " << msg.data;
            encodeMsg<double>(msg.data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor, dataframe->arryData);
        }

        for (auto &msg_pair : this->all_int_data)
        {
            auto &msg = msg_pair.second;
            // LOG(INFO) << "int message encoded: " << msg.data;
            encodeMsg<int>(msg.data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor, dataframe->arryData);
        }

        for (auto &msg_pair : this->all_bool_data)
        {
            auto &msg = msg_pair.second;
            // LOG(INFO) << "bool message encoded: " << msg.data;
            encodeMsg<bool>(msg.data, msg.lsb, msg.msb, msg.size, msg.offset, msg.factor, dataframe->arryData);
        }
    }
}