/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Neobotix GmbH
 *  All rights reserved.
 *
 * Author: Jan-Niklas Nieland
 *
 * Date of creation: July 2017
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


#include <stdio.h>
#include <math.h>
#include "../include/CANUSBoard.h"


/*************************************************
 * Constructor
 ************************************************/
CANUSBoard::CANUSBoard()
{
    m_bComInit = false;

    for(int i = 0; i < 4; i++)
    {
        m_iSensorData1To4[i] = 0;
        m_iSensorData5To8[i] = 0;
        m_iSensorData9To12[i] = 0;
        m_iSensorData13To16[i] = 0;
    }

    m_bReceivedData1To8Msg1 = false;
    m_bReceivedData1To8Msg2 = false;
    m_bReceivedData9To16Msg1 = false;
    m_bReceivedData9To16Msg2 = false;

    m_bReceiveParameterSetCompleted = false;
    m_bWriteParameterSetCompleted = false;

    m_iCANID = 0x400; //default CANID
}

/*************************************************
 * Deconstructor
 ************************************************/
CANUSBoard::~CANUSBoard()
{
    m_bComInit = false;
}

/*************************************************
 * Initialize CAN USBoard
 ************************************************/
bool CANUSBoard::init(ros::NodeHandle nh, unsigned int CANID)
{

    m_iCANID = CANID;

    //ROS Topics
    m_topicSubCANRecMsgs = nh.subscribe("/received_messages",100,&CANUSBoard::callbackReceivedCANMessage,this);
    m_topicPubCANSendMsgs = nh.advertise<can_msgs::Frame>("/sent_messages",20);

    m_bComInit = true;

    return m_bComInit;

}

/*************************************************
 * ROS Callback for incomming CAN-Messages
 ************************************************/
void CANUSBoard::callbackReceivedCANMessage(can_msgs::Frame msg)
{
    //ROS_INFO("CAN Msg Callback!");
    if(msg.id == m_iCANID + RESP_READ_PARAMSET)
    {
        //ROS_INFO("Read read param set at ID %d", m_iCANID + RESP_READ_PARAMSET);
        HandleReadParameterSetResponse(msg);
    }
    else if(msg.id == m_iCANID + RESP_GET_DATA_1TO8_PART_1)
    {
        //ROS_INFO("Read sensor data 1 - 8 response at ID %d", m_iCANID + RESP_GET_DATA_1TO8_PART_1);
        HandleReadSensorData1To8(msg);
    }
    else if(msg.id == m_iCANID + RESP_GET_DATA_1TO8_PART_2)
    {
        //ROS_INFO("Read sensor data 1 - 8 response at ID %d", m_iCANID + RESP_GET_DATA_1TO8_PART_2);
        HandleReadSensorData1To8(msg);
    }
    else if(msg.id == m_iCANID + RESP_GET_DATA_9TO16_PART_1)
    {
        //ROS_INFO("Read sensor data 9 - 16 response at ID %d", m_iCANID + RESP_GET_DATA_9TO16_PART_1);
        HandleReadSensorData9To16(msg);
    }
    else if(msg.id == m_iCANID + RESP_GET_DATA_9TO16_PART_2)
    {
        //ROS_INFO("Read sensor data 9 - 16 response at ID %d", m_iCANID + RESP_GET_DATA_9TO16_PART_2);
        HandleReadSensorData9To16(msg);
    }
    else if(msg.id == m_iCANID + RESP_WRITE_PARAMSET)
    {
        //ROS_INFO("Wrote ParamSet at ID %d", m_iCANID + RESP_WRITE_PARAMSET);
        HandleWriteParameterSetResponse(msg);
    }
    else
    {
         ROS_WARN("Unknown CAN Msg: %d", msg.id);
    }
    
}

/*************************************************************************************************
 *
 *
 *
 *                        PUBLIC FUNCTIONS
 *
 *
 *
 * ***********************************************************************************************/

/*************************************************
 * public function for requesting sensor data 1 - 8
 ************************************************/
void CANUSBoard::requestData1To8()
{
    //ROS_INFO("req read sensor data 1 - 8");
    m_bReceivedData1To8Msg1 = false;
    m_bReceivedData1To8Msg2 = false;

    can_msgs::Frame reqSensorData1To8;
    reqSensorData1To8.header.stamp = ros::Time::now();
    reqSensorData1To8.id = m_iCANID;
    reqSensorData1To8.data[0] = CMD_GET_DATA_1TO8;
    reqSensorData1To8.data[1] = 0;
    reqSensorData1To8.data[2] = 0;
    reqSensorData1To8.data[3] = 0;
    reqSensorData1To8.data[4] = 0;
    reqSensorData1To8.data[5] = 0;
    reqSensorData1To8.data[6] = 0;
    reqSensorData1To8.data[7] = 0;
    reqSensorData1To8.dlc = 8;
    m_topicPubCANSendMsgs.publish(reqSensorData1To8);


}
/*************************************************
 * public function to see if data 1 - 8 is available
 ************************************************/
bool CANUSBoard::receivedData1To8()
{
    if(m_bReceivedData1To8Msg1 && m_bReceivedData1To8Msg2)
    {
        m_bReceivedData1To8Msg1 = false;
        m_bReceivedData1To8Msg2 = false;
        //ROS_INFO("-----------received data 1 - 8------------");
        return true;
    }
    else
    {
        return false;
    }
}
/*************************************************
 * public function to get most recent data 1 - 8
 ************************************************/
bool CANUSBoard::getData1To8(int *iSensorDistCM)
{
    iSensorDistCM[0] = m_iSensorData1To4[0];
    iSensorDistCM[1] = m_iSensorData1To4[1];
    iSensorDistCM[2] = m_iSensorData1To4[2];
    iSensorDistCM[3] = m_iSensorData1To4[3];
    iSensorDistCM[4] = m_iSensorData5To8[0];
    iSensorDistCM[5] = m_iSensorData5To8[1];
    iSensorDistCM[6] = m_iSensorData5To8[2];
    iSensorDistCM[7] = m_iSensorData5To8[3];
    return true;
}
/*************************************************
 * public function for requesting sensor data 9 - 16
 ************************************************/
void CANUSBoard::requestData9To16()
{
    //ROS_INFO("req read sensor data 9 - 16");
    m_bReceivedData9To16Msg1 = false;
    m_bReceivedData9To16Msg2 = false;

    can_msgs::Frame reqSensorData9To16;
    reqSensorData9To16.header.stamp = ros::Time::now();
    reqSensorData9To16.id = m_iCANID;
    reqSensorData9To16.data[0] = CMD_GET_DATA_9TO16;
    reqSensorData9To16.data[1] = 0;
    reqSensorData9To16.data[2] = 0;
    reqSensorData9To16.data[3] = 0;
    reqSensorData9To16.data[4] = 0;
    reqSensorData9To16.data[5] = 0;
    reqSensorData9To16.data[6] = 0;
    reqSensorData9To16.data[7] = 0;
    reqSensorData9To16.dlc = 8;
    m_topicPubCANSendMsgs.publish(reqSensorData9To16);

}

/*************************************************
 * public function to see if data 1 - 8 is available
 ************************************************/
bool CANUSBoard::receivedData9To16()
{
    if(m_bReceivedData9To16Msg1 && m_bReceivedData9To16Msg2)
    {
        //ROS_INFO("-----------received data 9 - 16------------");
        m_bReceivedData9To16Msg1 = false;
        m_bReceivedData9To16Msg2 = false;
        return true;
    }
    else
    {
        return false;
    }
}
/*************************************************
 * public function to get most recent data 9 - 16
 ************************************************/
bool CANUSBoard::getData9To16(int *iSensorDistCM)
{
    iSensorDistCM[0] = m_iSensorData9To12[0];
    iSensorDistCM[1] = m_iSensorData9To12[1];
    iSensorDistCM[2] = m_iSensorData9To12[2];
    iSensorDistCM[3] = m_iSensorData9To12[3];
    iSensorDistCM[4] = m_iSensorData13To16[0];
    iSensorDistCM[5] = m_iSensorData13To16[1];
    iSensorDistCM[6] = m_iSensorData13To16[2];
    iSensorDistCM[7] = m_iSensorData13To16[3];
    return true;
}
/*************************************************
 * public function for requesting ParameterSet
 ************************************************/
int CANUSBoard::reqestParameterSet()
{
    m_bReceiveParameterSetCompleted = false;
    can_msgs::Frame reqParamSetMsg;
    reqParamSetMsg.header.stamp = ros::Time::now();
    reqParamSetMsg.id = m_iCANID;
    reqParamSetMsg.data[0] = CMD_READ_PARAMSET;
    reqParamSetMsg.data[1] = 0;
    reqParamSetMsg.data[2] = 0;
    reqParamSetMsg.data[3] = 0;
    reqParamSetMsg.data[4] = 0;
    reqParamSetMsg.data[5] = 0;
    reqParamSetMsg.data[6] = 0;
    reqParamSetMsg.data[7] = 0;
    reqParamSetMsg.dlc = 8;
    m_topicPubCANSendMsgs.publish(reqParamSetMsg);
}

/*************************************************
 * public function to see if paramset is available
 ************************************************/
bool CANUSBoard::receivedParameterSet()
{
    return m_bReceiveParameterSetCompleted;
}

/*************************************************
 * public function for writing ParameterSet
 ************************************************/
int CANUSBoard::writeParamset(int iPart, bool bWriteToEEPROM, unsigned char ucMode, unsigned char ucInterval, unsigned char ucSensorsActive1To8,
                              unsigned char ucSensorsActive9To16, unsigned char ucWarnDist[], unsigned char ucAlarmDist[])
{
    //ROS_INFO("writing param set part %d", iPart);

    m_bWriteParameterSetCompleted = false;

    m_iCurrentParamSetPart = iPart;

    /*ROS_INFO("DATA: ---------------");

    ROS_INFO("SensorsActive 1-8: %d", (int)ucSensorsActive1To8);
    ROS_INFO("SensorsActive 9-16: %d", (int)ucSensorsActive9To16);
    for(int i = 0; i < 16; i++)
    {
        ROS_INFO("Warn Dist %d: %d", i, (int)ucWarnDist[i]);
        ROS_INFO("Alarm Dist %d: %d", i, (int)ucAlarmDist[i]);
    }
    return 1;*/

    //----------------------------MSG 1 ---------------------
    if(iPart == 0)
    {
        can_msgs::Frame writeParamSetMsg1;
        writeParamSetMsg1.header.stamp = ros::Time::now();
        writeParamSetMsg1.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg1.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg1.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg1.data[1] = 0;
        writeParamSetMsg1.data[2] = m_ParameterSet.ucBaudRate; //CAN-Baudrate
        writeParamSetMsg1.data[3] = m_ParameterSet.ucCANAddr[0];
        writeParamSetMsg1.data[4] = m_ParameterSet.ucCANAddr[1];
        writeParamSetMsg1.data[5] = m_ParameterSet.ucCANAddr[2];
        writeParamSetMsg1.data[6] = m_ParameterSet.ucCANAddr[3];
        writeParamSetMsg1.data[7] = m_ParameterSet.ucCANExtended;
        writeParamSetMsg1.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg1);
    }

    //----------------------------MSG 2 ---------------------
    if(iPart == 1)
    {
        can_msgs::Frame writeParamSetMsg2;
        writeParamSetMsg2.header.stamp = ros::Time::now();
        writeParamSetMsg2.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg2.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg2.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg2.data[1] = 1;
        writeParamSetMsg2.data[2] = ucMode;
        writeParamSetMsg2.data[3] = ucInterval;
        writeParamSetMsg2.data[4] = ucSensorsActive1To8;
        writeParamSetMsg2.data[5] = ucSensorsActive9To16;
        writeParamSetMsg2.data[6] = ucWarnDist[0];
        writeParamSetMsg2.data[7] = ucWarnDist[1];
        writeParamSetMsg2.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg2);
    }

    //----------------------------MSG 3 ---------------------
    if(iPart == 2)
    {
        can_msgs::Frame writeParamSetMsg3;
        writeParamSetMsg3.header.stamp = ros::Time::now();
        writeParamSetMsg3.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg3.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg3.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg3.data[1] = 2;
        writeParamSetMsg3.data[2] = ucWarnDist[2]; //CAN-Baudrate
        writeParamSetMsg3.data[3] = ucWarnDist[3];
        writeParamSetMsg3.data[4] = ucWarnDist[4];
        writeParamSetMsg3.data[5] = ucWarnDist[5];
        writeParamSetMsg3.data[6] = ucWarnDist[6];
        writeParamSetMsg3.data[7] = ucWarnDist[7];
        writeParamSetMsg3.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg3);
    }

    //----------------------------MSG 4 ---------------------
    if(iPart == 3)
    {
        can_msgs::Frame writeParamSetMsg4;
        writeParamSetMsg4.header.stamp = ros::Time::now();
        writeParamSetMsg4.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg4.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg4.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg4.data[1] = 3;
        writeParamSetMsg4.data[2] = ucWarnDist[8];
        writeParamSetMsg4.data[3] = ucWarnDist[9];
        writeParamSetMsg4.data[4] = ucWarnDist[10];
        writeParamSetMsg4.data[5] = ucWarnDist[11];
        writeParamSetMsg4.data[6] = ucWarnDist[12];
        writeParamSetMsg4.data[7] = ucWarnDist[13];
        writeParamSetMsg4.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg4);
    }

    //----------------------------MSG 5 ---------------------
    if(iPart == 4)
    {
        can_msgs::Frame writeParamSetMsg5;
        writeParamSetMsg5.header.stamp = ros::Time::now();
        writeParamSetMsg5.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg5.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg5.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg5.data[1] = 4;
        writeParamSetMsg5.data[2] = ucWarnDist[14];
        writeParamSetMsg5.data[3] = ucWarnDist[15];
        writeParamSetMsg5.data[4] = ucAlarmDist[0];
        writeParamSetMsg5.data[5] = ucAlarmDist[1];
        writeParamSetMsg5.data[6] = ucAlarmDist[2];
        writeParamSetMsg5.data[7] = ucAlarmDist[3];
        writeParamSetMsg5.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg5);
    }

    //----------------------------MSG 6 ---------------------
    if(iPart == 5)
    {
        can_msgs::Frame writeParamSetMsg6;
        writeParamSetMsg6.header.stamp = ros::Time::now();
        writeParamSetMsg6.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg6.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg6.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg6.data[1] = 5;
        writeParamSetMsg6.data[2] = ucAlarmDist[4];
        writeParamSetMsg6.data[3] = ucAlarmDist[5];
        writeParamSetMsg6.data[4] = ucAlarmDist[6];
        writeParamSetMsg6.data[5] = ucAlarmDist[7];
        writeParamSetMsg6.data[6] = ucAlarmDist[8];
        writeParamSetMsg6.data[7] = ucAlarmDist[9];
        writeParamSetMsg6.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg6);
    }

    //----------------------------MSG 7 ---------------------
    if(iPart == 6)
    {
        can_msgs::Frame writeParamSetMsg7;
        writeParamSetMsg7.header.stamp = ros::Time::now();
        writeParamSetMsg7.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg7.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg7.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg7.data[1] = 6;
        writeParamSetMsg7.data[2] = ucAlarmDist[10];
        writeParamSetMsg7.data[3] = ucAlarmDist[11];
        writeParamSetMsg7.data[4] = ucAlarmDist[12];
        writeParamSetMsg7.data[5] = ucAlarmDist[13];
        writeParamSetMsg7.data[6] = ucAlarmDist[14];
        writeParamSetMsg7.data[7] = ucAlarmDist[15];
        writeParamSetMsg7.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg7);
    }

    //----------------------------MSG 8 ---------------------
    if(iPart == 7)
    {
        can_msgs::Frame writeParamSetMsg8;
        writeParamSetMsg8.header.stamp = ros::Time::now();
        writeParamSetMsg8.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg8.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg8.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg8.data[1] = 7;
        writeParamSetMsg8.data[2] = '0';   //43
        writeParamSetMsg8.data[3] = '0';   //44
        writeParamSetMsg8.data[4] = '0';   //45
        writeParamSetMsg8.data[5] = '0';   //46
        writeParamSetMsg8.data[6] = '0';   //47
        writeParamSetMsg8.data[7] = '0';   //48
        writeParamSetMsg8.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg8);
    }

    //----------------------------MSG 9 ---------------------
    if(iPart == 8)
    {
        can_msgs::Frame writeParamSetMsg9;
        writeParamSetMsg9.header.stamp = ros::Time::now();
        writeParamSetMsg9.id = m_iCANID;
        if(bWriteToEEPROM)
        {
            writeParamSetMsg9.data[0] = CMD_WRITE_PARAMSET_TO_EEPROM;
        }
        else
        {
            writeParamSetMsg9.data[0] = CMD_WRITE_PARAMSET;
        }
        writeParamSetMsg9.data[1] = 8;
        writeParamSetMsg9.data[2] = '0';   //49
        writeParamSetMsg9.data[3] = '0';   //50
        writeParamSetMsg9.data[4] = '0';   //51
        writeParamSetMsg9.data[5] = '0';   //52
        writeParamSetMsg9.data[6] = '0';   //53
        writeParamSetMsg9.data[7] = '0';   //54
        writeParamSetMsg9.dlc = 8;
        m_topicPubCANSendMsgs.publish(writeParamSetMsg9);
    }
}

bool CANUSBoard::confirmedParamsetPartX(int iPart)
{
    //ROS_INFO("is Part %d confirmed? %s", iPart, m_bWroteParameterSetPart[iPart] ? "true" : "false");
    return m_bWroteParameterSetPart[iPart];
}

/*************************************************************************************************
 *
 *
 *
 *                        PRIVATE FUNCTIONS
 *
 *
 *
 * ***********************************************************************************************/


/*************************************************
 * private function to handle
 * responses to CMD_GET_DATA_1TO8
 ************************************************/
int CANUSBoard::HandleReadSensorData1To8(can_msgs::Frame msg)
{

    if(msg.data[1] == 0)
    {
        //msg one
        //ROS_INFO("received part one of CMD_GET_DATA_1TO8");
        m_iSensorData1To4[0] = msg.data[2];
        m_iSensorData1To4[1] = msg.data[3];
        m_iSensorData1To4[2] = msg.data[4];
        m_iSensorData1To4[3] = msg.data[5];
        m_bReceivedData1To8Msg1 = true;
    }
    else if(msg.data[1] == 1)
    {
        //msg two
        //ROS_INFO("received part two of CMD_GET_DATA_1TO8");
        m_iSensorData5To8[0] = msg.data[2];
        m_iSensorData5To8[1] = msg.data[3];
        m_iSensorData5To8[2] = msg.data[4];
        m_iSensorData5To8[3] = msg.data[5];
        m_bReceivedData1To8Msg2 = true;
    }

    return 1;
}

/*************************************************
 * private function to handle
 * responses to CMD_GET_DATA_9TO16
 ************************************************/
int CANUSBoard::HandleReadSensorData9To16(can_msgs::Frame msg)
{
    if(msg.data[0] == CMD_GET_DATA_9TO16)
    {
        if(msg.data[1] == 0)
        {
            //msg one
            //ROS_INFO("received part one of CMD_GET_DATA_9TO16");
            m_iSensorData9To12[0] = msg.data[2];
            m_iSensorData9To12[1] = msg.data[3];
            m_iSensorData9To12[2] = msg.data[4];
            m_iSensorData9To12[3] = msg.data[5];
            m_bReceivedData9To16Msg1 = true;
        }
        else if(msg.data[1] == 1)
        {
            //msg two
            //ROS_INFO("received part two of CMD_GET_DATA_9TO16");
            m_iSensorData13To16[0] = msg.data[2];
            m_iSensorData13To16[1] = msg.data[3];
            m_iSensorData13To16[2] = msg.data[4];
            m_iSensorData13To16[3] = msg.data[5];
            m_bReceivedData9To16Msg2 = true;
        }
    }
    else
    {
        //ROS_WARN("Wrong CMD_BYTE in CMD_GET_DATA_9TO16 response");
        return 99; //Wrong CMD_BYTE
    }
    return 1;
}

/*************************************************
 * private function to handle
 * responses to CMD_READ_PARAMSET
 ************************************************/
int CANUSBoard::HandleReadParameterSetResponse(can_msgs::Frame msg)
{
    if(msg.data[0] == CMD_READ_PARAMSET)
    {
        for(int iParamSetResponseCnt = 0; iParamSetResponseCnt < 9; iParamSetResponseCnt++)
        {
            if(!m_bReceivedParameterSetPart[iParamSetResponseCnt])
            {
                m_bReceivedParameterSetPart[iParamSetResponseCnt] = true;
                //ROS_INFO("Read ParamSet response %d /9 received", iParamSetResponseCnt+1);
                //ROS_INFO("ID: %d", msg.data[1]);

                if(msg.data[1] == 0)
                {
                    m_ParameterSet.ucBaudRate = msg.data[2];
                    m_ParameterSet.ucCANAddr[0] = msg.data[3];
                    m_ParameterSet.ucCANAddr[1] = msg.data[4];
                    m_ParameterSet.ucCANAddr[2] = msg.data[5];
                    m_ParameterSet.ucCANAddr[3] = msg.data[6];
                    m_ParameterSet.ucCANExtended = msg.data[7];
                }
                else if(msg.data[1] == 1)
                {
                    m_ParameterSet.ucTransmissionMode = msg.data[2];
                    m_ParameterSet.ucTransmissionInterval = msg.data[3];
                    m_ParameterSet.ucSensorsActive9To16 = msg.data[4];
                    m_ParameterSet.ucSensorsActive9To16 = msg.data[5];
                    m_ParameterSet.ucWarnDist[0] = msg.data[6];
                    m_ParameterSet.ucWarnDist[1] = msg.data[7];
                }
                else if(msg.data[1] == 2)
                {
                    m_ParameterSet.ucWarnDist[2] = msg.data[2];
                    m_ParameterSet.ucWarnDist[3] = msg.data[3];
                    m_ParameterSet.ucWarnDist[4] = msg.data[4];
                    m_ParameterSet.ucWarnDist[5] = msg.data[5];
                    m_ParameterSet.ucWarnDist[6] = msg.data[6];
                    m_ParameterSet.ucWarnDist[7] = msg.data[7];
                }
                else if(msg.data[1] == 3)
                {
                    m_ParameterSet.ucWarnDist[8] = msg.data[2];
                    m_ParameterSet.ucWarnDist[9] = msg.data[3];
                    m_ParameterSet.ucWarnDist[10] = msg.data[4];
                    m_ParameterSet.ucWarnDist[11] = msg.data[5];
                    m_ParameterSet.ucWarnDist[12] = msg.data[6];
                    m_ParameterSet.ucWarnDist[13] = msg.data[7];
                }
                else if(msg.data[1] == 4)
                {
                    m_ParameterSet.ucWarnDist[14] = msg.data[2];
                    m_ParameterSet.ucWarnDist[15] = msg.data[3];
                    m_ParameterSet.ucAlarmDist[0] = msg.data[4];
                    m_ParameterSet.ucAlarmDist[1] = msg.data[5];
                    m_ParameterSet.ucAlarmDist[2] = msg.data[6];
                    m_ParameterSet.ucAlarmDist[3] = msg.data[7];
                }
                else if(msg.data[1] == 5)
                {
                    m_ParameterSet.ucAlarmDist[4] = msg.data[2];
                    m_ParameterSet.ucAlarmDist[5] = msg.data[3];
                    m_ParameterSet.ucAlarmDist[6] = msg.data[4];
                    m_ParameterSet.ucAlarmDist[7] = msg.data[5];
                    m_ParameterSet.ucAlarmDist[8] = msg.data[6];
                    m_ParameterSet.ucAlarmDist[9] = msg.data[7];
                }
                else if(msg.data[1] == 6)
                {
                    m_ParameterSet.ucAlarmDist[10] = msg.data[2];
                    m_ParameterSet.ucAlarmDist[11] = msg.data[3];
                    m_ParameterSet.ucAlarmDist[12] = msg.data[4];
                    m_ParameterSet.ucAlarmDist[13] = msg.data[5];
                    m_ParameterSet.ucAlarmDist[14] = msg.data[6];
                    m_ParameterSet.ucAlarmDist[15] = msg.data[7];
                }
                else if(msg.data[1] == 7)
                {
                    /*m_ParameterSet.ucBaudRate = msg.data[2];
                    m_ParameterSet.ucCANAddr[0] = msg.data[3];
                    m_ParameterSet.ucCANAddr[1] = msg.data[4];
                    m_ParameterSet.ucCANAddr[2] = msg.data[5];
                    m_ParameterSet.ucCANAddr[3] = msg.data[6];
                    m_ParameterSet.ucCANExtended = msg.data[7];*/
                }
                else if(msg.data[1] == 8)
                {
                    /*m_ParameterSet.ucBaudRate = msg.data[2];
                    m_ParameterSet.ucCANAddr[0] = msg.data[3];
                    m_ParameterSet.ucCANAddr[1] = msg.data[4];*/
                    m_ParameterSet.ucSerialNumber[0] = msg.data[5];
                    m_ParameterSet.ucSerialNumber[1] = msg.data[6];
                    m_ParameterSet.ucSerialNumber[2] = msg.data[7];
                }
                else
                {
                    ROS_INFO("wrong answer id");
                }

                if(iParamSetResponseCnt == 8)
                {
                    //this was the last ParameterSet message
                    //ParameterSet is complete
                    //ROS_INFO("ParamSet complete");

                    //Print ParamSet
                    /*ROS_INFO("Baud rate code: %d", m_ParameterSet.ucBaudRate);
                    ROS_INFO("Transmission mode: %d", m_ParameterSet.ucTransmissionMode);
                    ROS_INFO("Transmission interval: %d", m_ParameterSet.ucTransmissionInterval);
                    ROS_INFO("Sensor Active 1-8: %d", m_ParameterSet.ucSensorsActive1To8);
                    ROS_INFO("Sensor Active 9-16: %d", m_ParameterSet.ucSensorsActive9To16);
                    ROS_INFO("Warn Dist1: %d", m_ParameterSet.ucWarnDist[0]);
                    ROS_INFO("Warn Dist2: %d", m_ParameterSet.ucWarnDist[1]);
                    ROS_INFO("Warn Dist3: %d", m_ParameterSet.ucWarnDist[2]);
                    ROS_INFO("Warn Dist4: %d", m_ParameterSet.ucWarnDist[3]);
                    ROS_INFO("Warn Dist5: %d", m_ParameterSet.ucWarnDist[4]);
                    ROS_INFO("Warn Dist6: %d", m_ParameterSet.ucWarnDist[5]);
                    ROS_INFO("Warn Dist7: %d", m_ParameterSet.ucWarnDist[6]);
                    ROS_INFO("Warn Dist8: %d", m_ParameterSet.ucWarnDist[7]);
                    ROS_INFO("Warn Dist9: %d", m_ParameterSet.ucWarnDist[8]);
                    ROS_INFO("Warn Dist10: %d", m_ParameterSet.ucWarnDist[9]);
                    ROS_INFO("Warn Dist11: %d", m_ParameterSet.ucWarnDist[10]);
                    ROS_INFO("Warn Dist12: %d", m_ParameterSet.ucWarnDist[11]);
                    ROS_INFO("Warn Dist13: %d", m_ParameterSet.ucWarnDist[12]);
                    ROS_INFO("Warn Dist14: %d", m_ParameterSet.ucWarnDist[13]);
                    ROS_INFO("Warn Dist15: %d", m_ParameterSet.ucWarnDist[14]);
                    ROS_INFO("Warn Dist16: %d", m_ParameterSet.ucWarnDist[15]);

                    ROS_INFO("Alarm Dist1: %d", m_ParameterSet.ucAlarmDist[0]);
                    ROS_INFO("Alarm Dist2: %d", m_ParameterSet.ucAlarmDist[1]);
                    ROS_INFO("Alarm Dist3: %d", m_ParameterSet.ucAlarmDist[2]);
                    ROS_INFO("Alarm Dist4: %d", m_ParameterSet.ucAlarmDist[3]);
                    ROS_INFO("Alarm Dist5: %d", m_ParameterSet.ucAlarmDist[4]);
                    ROS_INFO("Alarm Dist6: %d", m_ParameterSet.ucAlarmDist[5]);
                    ROS_INFO("Alarm Dist7: %d", m_ParameterSet.ucAlarmDist[6]);
                    ROS_INFO("Alarm Dist8: %d", m_ParameterSet.ucAlarmDist[7]);
                    ROS_INFO("Alarm Dist9: %d", m_ParameterSet.ucAlarmDist[8]);
                    ROS_INFO("Alarm Dist10: %d", m_ParameterSet.ucAlarmDist[9]);
                    ROS_INFO("Alarm Dist11: %d", m_ParameterSet.ucAlarmDist[10]);
                    ROS_INFO("Alarm Dist12: %d", m_ParameterSet.ucAlarmDist[11]);
                    ROS_INFO("Alarm Dist13: %d", m_ParameterSet.ucAlarmDist[12]);
                    ROS_INFO("Alarm Dist14: %d", m_ParameterSet.ucAlarmDist[13]);
                    ROS_INFO("Alarm Dist15: %d", m_ParameterSet.ucAlarmDist[14]);
                    ROS_INFO("Alarm Dist16: %d", m_ParameterSet.ucAlarmDist[15]);

                    int iSerialNumber = ( m_ParameterSet.ucCANAddr[2] << 16) | ( m_ParameterSet.ucCANAddr[1] << 8) | m_ParameterSet.ucCANAddr[0];
                    ROS_INFO("Serial Number: %d", iSerialNumber);*/

                    //reset cnt
                    for(int a = 0; a < 9; a++)
                    {
                        m_bReceivedParameterSetPart[a] = false;
                    }

                    m_bReceiveParameterSetCompleted = true;

                    return 1; //complete

                }
                else
                {
                    //handle data
                    return 2; //Data handled
                }
            }
        }
    }
    else
    {
        ROS_WARN("Wrong CMD_BYTE in read parameter set response");
        return 99; //Wrong CMD_BYTE
    }

    return 1;
}

/*************************************************
 * private function to handle
 * responses to CMD_WRITE_PARAMSET
 ************************************************/
int CANUSBoard::HandleWriteParameterSetResponse(can_msgs::Frame msg)
{
    if(msg.data[0] == CMD_WRITE_PARAMSET)
    {
        //ROS_INFO("Write ParamSet response %d /9 received", m_iCurrentParamSetPart);
        m_bWroteParameterSetPart[m_iCurrentParamSetPart] = true;
    }

    return 1;
}
