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
    m_topicPubCANSendMsgs = nh.advertise<can_msgs::Frame>("/sent_messages",1);

    m_bComInit = true;

    return m_bComInit;

}

/*************************************************
 * ROS Callback for incomming CAN-Messages
 ************************************************/
void CANUSBoard::callbackReceivedCANMessage(can_msgs::Frame msg)
{
    //ROS_INFO("CAN Msg Callback!");
    if(msg.id == m_iCANID + RESP_READ_PARASET)
    {
        //ROS_INFO("Read read param set at ID %d", m_iCANID + RESP_READ_PARASET);
        //HandleReadParameterSetResponse(msg);
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
    else
    {
         ROS_WARN("Unknown CAN Msg: %d", msg.id);
    }
    
}
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
    //ROS_INFO("req read param set");
    can_msgs::Frame reqParamSetMsg;
    reqParamSetMsg.header.stamp = ros::Time::now();
    reqParamSetMsg.id = m_iCANID;
    reqParamSetMsg.data[0] = CMD_READ_PARASET;
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
 * responses to CMD_READ_PARASET
 ************************************************/
int CANUSBoard::HandleReadParameterSetResponse(can_msgs::Frame msg)
{
    if(msg.data[0] == CMD_READ_PARASET)
    {
        for(int iParamSetResponseCnt = 0; iParamSetResponseCnt < 9; iParamSetResponseCnt++)
        {
            if(!m_bReceivedParameterSetPart[iParamSetResponseCnt])
            {
                m_bReceivedParameterSetPart[iParamSetResponseCnt] = true;
                ROS_INFO("Read ParamSet response %d /9 received", iParamSetResponseCnt);

                if(iParamSetResponseCnt == 8)
                {
                    //this was the last ParameterSet message
                    //ParameterSet is complete
                    m_bReceiveParameterSetCompleted = true;

                    //reset cnt
                    for(int a = 0; a < 9; a++)
                    {
                        m_bReceivedParameterSetPart[a] = false;
                    }
                    return 1; //complete

                }
                else
                {
                    //hanhdle data
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
