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
bool CANUSBoard::init(ros::NodeHandle nh, int CANID)
{

    m_iCANID = CANID;

    //ROS Topics
    m_topicSubCANRecMsgs = nh.subscribe("/received_messages",100,&CANUSBoard::callbackReceivedCANMessage,this) ;
    m_topicPubCANSendMsgs = nh.advertise<can_msgs::Frame>("/sent_messages",1);

    m_bComInit = true;

    return m_bComInit;

}

/*************************************************
 * ROS Callback for incomming CAN-Messages
 ************************************************/
void CANUSBoard::callbackReceivedCANMessage(can_msgs::Frame msg)
{
    if(msg.id == m_iCANID + 6)
    {
        ROS_INFO("Read ParamSet response at ID + 6");
        HandleReadParameterSetResponse(msg);
    }
}
/*************************************************
 * public function for requesting ParameterSet
 ************************************************/
int CANUSBoard::reqestParameterSet()
{
    ROS_INFO("req read param set");
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
