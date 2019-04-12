/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef CANUSBoard_INCLUDEDEF_H
#define CANUSBoard_INCLUDEDEF_H

//#include <Mutex.h>
// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <can_msgs/Frame.h>

//-----------------------------------------------

/**
 * Driver class for communication with a Neobotix USBoard.
 * Uses ROS-CANOpen Interface for receiving and transmitting messages
 */

class CANUSBoard
{
    public:
        CANUSBoard();
        ~CANUSBoard();

        bool init(ros::NodeHandle nh, unsigned int CANID);

        //ROS-CAN Interface Callback Functions
        void callbackReceivedCANMessage(can_msgs::Frame msg);

        //ParameterSet request
        int reqestParameterSet();
        bool receivedParameterSet();

        //Write ParameterSet
        int writeParamset(int iPart, bool bWriteToEEPROM, unsigned char ucMode, unsigned char ucInterval, unsigned char ucSensorsActive1To8,
                          unsigned char ucSensorsActive9To16, unsigned char ucWarnDist[], unsigned char ucAlarmDist[]);

        bool confirmedParamsetPartX(int iPart);

        //Set Channel Active
        void setChannelActive(bool *ChannelActive);

        //Get Data 1To8
        void requestData1To8();
        bool receivedData1To8();
        bool getData1To8(int *iSensorDistCM);

        //Get Data 9To16
        void requestData9To16();
        bool receivedData9To16();
        bool getData9To16(int *iSensorDistCM);

        //Get AnalogIN
        void requestAnalogInputData();
        bool receivedAnalogInputData();
        bool getAnalogInputData(int *iAnalogIn);

    private:

        //Mutex m_Mutex;

        bool m_bComInit;

        //ROS Topics
        //ROS-CAN-Interface
        ros::Subscriber m_topicSubCANRecMsgs;
        ros::Publisher m_topicPubCANSendMsgs;

        // USBoard
        unsigned int m_iCANID;

        // USBoard send commands
        enum USBoardCmd
        {
            CMD_CONNECT= 0,
            CMD_SET_CHANNEL_ACTIVE = 1,
            CMD_GET_DATA_1TO8 = 2,
            CMD_GET_DATA_9TO16 = 3,
            CMD_WRITE_PARAMSET = 4,
            CMD_WRITE_PARAMSET_TO_EEPROM = 5,
            CMD_READ_PARAMSET = 6,
            CMD_GET_ANALOGIN = 7,
            CMD_SET_DEBUG_PARA = 8,
            CMD_GET_DEBUG_PARA = 9,
            CMD_UNKNOWN = 10
        };

        enum USBoardRespIDs
        {
            RESP_CONNECT= 0,
            RESP_GET_DATA_1TO8_PART_1 = 2,
            RESP_GET_DATA_1TO8_PART_2 = 3,
            RESP_GET_DATA_9TO16_PART_1 = 4,
            RESP_GET_DATA_9TO16_PART_2 = 5,
            RESP_READ_PARAMSET = 6,
            RESP_GET_ANALOGIN = 7,
            RESP_WRITE_PARAMSET = 8,
            RESP_WRITE_PARAMSET_TO_EEPROM = 9
        };

        struct USBoardParameterSet
        {
            unsigned char ucBaudRate;
            unsigned char ucCANAddr[4];
            unsigned char ucCANExtended;
            unsigned char ucTransmissionMode;
            unsigned char ucTransmissionInterval;
            unsigned char ucSensorsActive1To8;
            unsigned char ucSensorsActive9To16;
            unsigned char ucWarnDist[16];
            unsigned char ucAlarmDist[16];
            unsigned char ucSerialNumber[3];
        };

        enum USBoardCANIDs
        {
            CANID_SET_CHANNEL_ACTIVE = 0
        };

        //USBoard Data Readings
        int m_iCmdConnectAns[7];
        int m_iReadAnsFormat;
        int m_iSensorData1To4[4];
        int m_iSensorData5To8[4];
        int m_iSensorData9To12[4];
        int m_iSensorData13To16[4];
        int m_iAnalogInDataCh1To4LowByte[4];
        int m_iAnalogInDataCh1To4HighBits[2];


        //CAN-Message Handling Flags
        bool m_bReceivedParameterSetPart[9];
        bool m_bReceiveParameterSetCompleted;
        bool m_bWroteParameterSetPart[9];
        bool m_bWriteParameterSetCompleted;
        int m_iCurrentParamSetPart;
        bool m_bReceivedData1To8;
        bool m_bReceivedData1To8Msg1;
        bool m_bReceivedData1To8Msg2;
        bool m_bReceivedData9To16;
        bool m_bReceivedData9To16Msg1;
        bool m_bReceivedData9To16Msg2;
        bool m_bReceivedAnalogInputData;

        USBoardParameterSet m_ParameterSet;


        //Private CAN msg handling functions
        int HandleReadParameterSetResponse(can_msgs::Frame msg);
        int HandleWriteParameterSetResponse(can_msgs::Frame msg);
        int HandleReadSensorData1To8(can_msgs::Frame msg);
        int HandleReadSensorData9To16(can_msgs::Frame msg);

};

#endif
