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

#ifndef SerUSBoard_INCLUDEDEF_H
#define SerUSBoard_INCLUDEDEF_H


#include <SerialIO.h>
#include <Mutex.h>

//-----------------------------------------------

/**
 * Driver class for communication with a Neobotix USBoard.
 * Uses RS232 with 19,2 kBaud.
 */

class SerUSBoard
{
    public:
        SerUSBoard();
        ~SerUSBoard();

        bool init(const char* sNumComPort);

        // evaluate received buffer data
        int eval_RXBuffer();

        // send commands to USBoard
        int sendCmd();

        int sendCmdConnect();
        int sendCmdSetChannelActive();
        int sendCmdGetData1To8();
        int sendCmdGetData9To16();
        int sendCmdGetAnalogIn();

        // read received sensor data from buffer
        int getTransModeData();
        int getSensorData(int *iSensorDistCM);
        int getSensorData1To4(int *iSensorDistCM);
        int getSensorData5To8(int *iSensorDistCM);
        int getSensorData9To12(int *iSensorDistCM);
        int getSensorData13To16(int *iSensorDistCM);
        int getAnalogInCh1To4Data(int *iAnalogInCh1To4Data);

        //Logging
        void enable_logging();
        void disable_logging();
        void log_to_file(int direction, unsigned char cMsg[]);

        // USBoard send commands
        enum USBoardCmd
        {
            CMD_CONNECT= 0,
            CMD_SET_CHANNEL_ACTIVE = 1,
            CMD_GET_DATA_1TO8 = 2,
            CMD_GET_DATA_9TO16 = 3,
            CMD_WRITE_PARASET = 4,
            CMD_WRITE_PARASET_TO_EEPROM = 5,
            CMD_READ_PARASET = 6,
            CMD_GET_ANALOGIN = 7,
            CMD_SET_DEBUG_PARA = 8,
            CMD_GET_DEBUG_PARA = 9,
            CMD_UNKNOWN = 10
        };


        enum USBoardReturns
        {
            NO_ERROR = 0,
            NOT_INITIALIZED = 1,
            GENERAL_SENDING_ERROR = 2,
            TOO_LESS_BYTES_IN_QUEUE = 3,
            NO_MESSAGES = 4, //for a long time, no message have been received, check com port!
            CHECKSUM_ERROR = 5
        };

    private:

        Mutex m_Mutex;

        int m_iNumBytesSend;

        //logging
        bool m_bLogging;

        // USBoard
        SerialIO m_SerIO;
        bool m_bComInit;
        int m_iConfigUSBoard;
        int m_iCmdUSBoard;

        //USBoard Data Readings
        int m_iCmdConnectAns[7];
        int m_iReadAnsFormat;
        int m_iSensorData1To4[4];
        int m_iSensorData5To8[4];
        int m_iSensorData9To12[4];
        int m_iSensorData13To16[4];
        int m_iAnalogInDataCh1To4LowByte[4];
        int m_iAnalogInDataCh1To4HighBits[2];

        unsigned int getCheckSum(unsigned char *icMsg, int iNumBytes);

        void convDataToSendMsg(unsigned char cMsg[]);
        bool convRecMsgToData(unsigned char cMsg[]);

};

#endif
