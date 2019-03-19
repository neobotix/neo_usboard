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

#include <ros/ros.h>
#include <../include/neo_usboard_node.h>




//--------------------------------------------------------------------------------
int neo_usboard_node::init()
{

    /************************************************************************
     * Read Parameter from ROS Parameter-Server
    ************************************************************************/
    n.param("use_can_interface", m_bUseCANInterface, true);

    if(!m_bUseCANInterface)
    {
        //Use Serial Interface
        ROS_INFO("USBoard: Loading Parameter from ROS-Parameter-Server");
        if(n.hasParam("ComPort"))
        {
            n.getParam("ComPort", m_sComPort);
            ROS_INFO("USBoard: Loaded ComPort parameter from parameter server: %s",m_sComPort.c_str());
        }

        n.param("message_timeout", usboard_timeout_, 2.0);
        n.param("requestRate", requestRate, 5.0);
    }
    else
    {
        ROS_INFO("USBoard: Using CAN-Interface");
        m_CANUSBoard = new CANUSBoard;
        m_CANUSBoard->init(n,(unsigned int)0x400);
    }

    //Mode
    n.param("mode", m_iMode, 1);

    //Timeout
    n.param("timeout", m_dTimeOut, 1.0);

    //Sensor Active Parameter
    n.param("sensor1_active", m_bSensorActive[0], false);
    n.param("sensor2_active", m_bSensorActive[1], false);
    n.param("sensor3_active", m_bSensorActive[2], false);
    n.param("sensor4_active", m_bSensorActive[3], false);
    n.param("sensor5_active", m_bSensorActive[4], false);
    n.param("sensor6_active", m_bSensorActive[5], false);
    n.param("sensor7_active", m_bSensorActive[6], false);
    n.param("sensor8_active", m_bSensorActive[7], false);
    n.param("sensor9_active", m_bSensorActive[8], false);
    n.param("sensor10_active", m_bSensorActive[9], false);
    n.param("sensor11_active", m_bSensorActive[10], false);
    n.param("sensor12_active", m_bSensorActive[11], false);
    n.param("sensor13_active", m_bSensorActive[12], false);
    n.param("sensor14_active", m_bSensorActive[13], false);
    n.param("sensor15_active", m_bSensorActive[14], false);
    n.param("sensor16_active", m_bSensorActive[15], false);

    //Sensor Frame Parameter
    n.param<std::string>("sensor1_frame", m_sSensorFrame[0], "range_1_link");
    n.param<std::string>("sensor2_frame", m_sSensorFrame[1], "range_2_link");
    n.param<std::string>("sensor3_frame", m_sSensorFrame[2], "range_3_link");
    n.param<std::string>("sensor4_frame", m_sSensorFrame[3], "range_4_link");
    n.param<std::string>("sensor5_frame", m_sSensorFrame[4], "range_5_link");
    n.param<std::string>("sensor6_frame", m_sSensorFrame[5], "range_6_link");
    n.param<std::string>("sensor7_frame", m_sSensorFrame[6], "range_7_link");
    n.param<std::string>("sensor8_frame", m_sSensorFrame[7], "range_8_link");
    n.param<std::string>("sensor9_frame", m_sSensorFrame[8], "range_9_link");
    n.param<std::string>("sensor10_frame", m_sSensorFrame[9], "range_10_link");
    n.param<std::string>("sensor11_frame", m_sSensorFrame[10], "range_11_link");
    n.param<std::string>("sensor12_frame", m_sSensorFrame[11], "range_12_link");
    n.param<std::string>("sensor13_frame", m_sSensorFrame[12], "range_13_link");
    n.param<std::string>("sensor14_frame", m_sSensorFrame[13], "range_14_link");
    n.param<std::string>("sensor15_frame", m_sSensorFrame[14], "range_15_link");
    n.param<std::string>("sensor16_frame", m_sSensorFrame[15], "range_16_link");

    n.param("min_range", m_dSensorMinRange, 0.2);
    n.param("max_range", m_dSensorMaxRange, 1.2);


    //log
    n.param("log", log, false);

    /************************************************************************
     * Initialize USBoard
     *
     *
     *
     *
    ************************************************************************/
    //m_SerUSBoard = new SerUSBoard();

    //bool bInitSerUSBoardRet = false;
    //bInitSerUSBoardRet = m_SerUSBoard->init(m_sComPort.c_str());

    /*if(bInitSerUSBoardRet)
    {
        ROS_INFO("Opened USboard at ComPort = %s", m_sComPort.c_str());
    }
    else
    {
        ROS_ERROR("FAILED: Could not opened USboard at ComPort = %s", m_sComPort.c_str());
        return 1;
    }*/

    //enable logging if needed
    if(log == true)
    {
        ROS_INFO("USBoard: Log enabled");
        //m_SerUSBoard->enable_logging();
    }
    else
    {
        ROS_INFO("USBoard: Log disabled");
        //m_SerUSBoard->disable_logging();
    }

    /************************************************************************
     * Advertise and Subscribe ROS Topics
     *
     *
     *
     *
    ************************************************************************/

    topicPub_usBoard = n.advertise<neo_msgs::USBoard>("/usboard/measurements",1);

    if(m_bSensorActive[0])topicPub_USRangeSensor1 = n.advertise<sensor_msgs::Range>("/usboard/sensor1",1);
    if(m_bSensorActive[1])topicPub_USRangeSensor2 = n.advertise<sensor_msgs::Range>("/usboard/sensor2",1);
    if(m_bSensorActive[2])topicPub_USRangeSensor3 = n.advertise<sensor_msgs::Range>("/usboard/sensor3",1);
    if(m_bSensorActive[3])topicPub_USRangeSensor4 = n.advertise<sensor_msgs::Range>("/usboard/sensor4",1);
    if(m_bSensorActive[4])topicPub_USRangeSensor5 = n.advertise<sensor_msgs::Range>("/usboard/sensor5",1);
    if(m_bSensorActive[5])topicPub_USRangeSensor6 = n.advertise<sensor_msgs::Range>("/usboard/sensor6",1);
    if(m_bSensorActive[6])topicPub_USRangeSensor7 = n.advertise<sensor_msgs::Range>("/usboard/sensor7",1);
    if(m_bSensorActive[7])topicPub_USRangeSensor8 = n.advertise<sensor_msgs::Range>("/usboard/sensor8",1);
    if(m_bSensorActive[8])topicPub_USRangeSensor9 = n.advertise<sensor_msgs::Range>("/usboard/sensor9",1);
    if(m_bSensorActive[9])topicPub_USRangeSensor10 = n.advertise<sensor_msgs::Range>("/usboard/sensor10",1);
    if(m_bSensorActive[10])topicPub_USRangeSensor11 = n.advertise<sensor_msgs::Range>("/usboard/sensor11",1);
    if(m_bSensorActive[11])topicPub_USRangeSensor12 = n.advertise<sensor_msgs::Range>("/usboard/sensor12",1);
    if(m_bSensorActive[12])topicPub_USRangeSensor13 = n.advertise<sensor_msgs::Range>("/usboard/sensor13",1);
    if(m_bSensorActive[13])topicPub_USRangeSensor14 = n.advertise<sensor_msgs::Range>("/usboard/sensor14",1);
    if(m_bSensorActive[14])topicPub_USRangeSensor15 = n.advertise<sensor_msgs::Range>("/usboard/sensor15",1);
    if(m_bSensorActive[15])topicPub_USRangeSensor16 = n.advertise<sensor_msgs::Range>("/usboard/sensor16",1);

    //ROS-Service
    if(m_iMode == 0) //request mode
    {
        srv_request_data = n.advertiseService("/usboard/request_data", &neo_usboard_node::ros_callback_req_data, this);
    }

    ROS_INFO("USBoard: Init done");

   return 0;
}

//--------------------------------------------------------------------------------
bool neo_usboard_node::ros_callback_req_data(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    m_bDataRequestedviaService = true;
    return true;
}

//--------------------------------------------------------------------------------
int neo_usboard_node::getMode()
{
    //ROS_INFO("Mode: %i", m_iMode);
    return m_iMode;
}

//--------------------------------------------------------------------------------
double neo_usboard_node::getRequestRate()
{
    //ROS_INFO("Requestrate: %d", requestRate);
    return requestRate;
}

//--------------------------------------------------------------------------------
double neo_usboard_node::getTimeOut()
{
    //ROS_INFO("Timeout: %d", m_dTimeOut);
    return m_dTimeOut;
}

//--------------------------------------------------------------------------------
bool neo_usboard_node::getDataRequestService()
{
    if(m_bDataRequestedviaService)
    {
        //ROS_INFO("Data was requested via service");
        m_bDataRequestedviaService = false;
        return true;
    }
    else
    {
        return false;
    }
}

//--------------------------------------------------------------------------------
int neo_usboard_node::requestSensorData()
{
    /* This functions wraps requesting sensor data for Serial and CAN Interfaces
     *
     */
    if(m_bUseCANInterface)
    {
        m_CANUSBoard->requestData1To8();
        m_CANUSBoard->requestData9To16();
    }
    else
    {
        //TODO
        ROS_WARN("USBoard: requesting ParameterSets for serial connection not implemented yet!");
    }
    return 1;
}

//--------------------------------------------------------------------------------
bool neo_usboard_node::receivedSensorData()
{
    if(m_bUseCANInterface)
    {
        if(m_CANUSBoard->receivedData1To8() && m_CANUSBoard->receivedData9To16())
        {
            //read sensor data
            //ROS_WARN("############SENSORDATA RECEIVED!###########");
            return true;
        }
        else
        {
            //wait
            return false;
        }
    }
    else
    {
        //TODO
        ROS_WARN("USBoard: requesting ParameterSets for serial connection not implemented yet!");
    }
    return false;
}

//--------------------------------------------------------------------------------
void neo_usboard_node::publishUSBoardData()
{
    /* This functions wraps publishing most recent sensor data for Serial and CAN Interfaces
     *
     */
    int iUSSensors1To8[8];
    int iUSSensors9To16[8];

    for (int i = 0; i < 8 ; i++)
    {
        iUSSensors1To8[i] = 0;
        iUSSensors9To16[i] = 0;
    }

    if(m_bUseCANInterface)
    {
        m_CANUSBoard->getData1To8(iUSSensors1To8);
        m_CANUSBoard->getData9To16(iUSSensors9To16);
    }
    else
    {
        //TODO
        ROS_WARN("USBoard: publish data from serial connection not implemented yet!");
    }

    //copy data to msg
    neo_msgs::USBoard msg_USBoard;
    for (int i = 0; i < 8 ; i++)
    {
        msg_USBoard.sensor[i] = iUSSensors1To8[i];
        msg_USBoard.sensor[i+8] = iUSSensors9To16[i];
    }

    //Publish raw data in neo_msgs::USBoard format
    //ROS_WARN("publishing data");
    topicPub_usBoard.publish(msg_USBoard);

    //Additionally publish data in ROS sensor_msgs::Range format
    //-------------------------------------------SENSOR1--------------------------------------------------------
    if(m_bSensorActive[0])
    {
        std_msgs::Header USRange1Header;
        sensor_msgs::Range USRange1Msg;
        //create USRanger1Msg
        //fill in header
        USRange1Header.seq = 1; 				//uint32
        USRange1Header.stamp = ros::Time::now(); 		//time
        USRange1Header.frame_id = "range_1_link";		//string

        USRange1Msg.header = USRange1Header;
        USRange1Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange1Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange1Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange1Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange1Msg.range = ((float)iUSSensors1To8[0]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor1.publish(USRange1Msg);
    }
    //----------------------------------------------------------------------------------------------------------
    //-------------------------------------------SENSOR2--------------------------------------------------------
    if(m_bSensorActive[1])
    {
        std_msgs::Header USRange2Header;
        sensor_msgs::Range USRange2Msg;
        //create USRanger2Msg
        //fill in header
        USRange2Header.seq = 1; 				//uint32
        USRange2Header.stamp = ros::Time::now(); 		//time
        USRange2Header.frame_id = "range_2_link";		//string

        USRange2Msg.header = USRange2Header;
        USRange2Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange2Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange2Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange2Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange2Msg.range = ((float)iUSSensors1To8[1]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor2.publish(USRange2Msg);

    }
    //----------------------------------------------------------------------------------------------------------
    //-------------------------------------------SENSOR3--------------------------------------------------------
    if(m_bSensorActive[2])
    {
        std_msgs::Header USRange3Header;
        sensor_msgs::Range USRange3Msg;
        //create USRanger3Msg
        //fill in header
        USRange3Header.seq = 1; 				//uint32
        USRange3Header.stamp = ros::Time::now(); 		//time
        USRange3Header.frame_id = "range_3_link";		//string

        USRange3Msg.header = USRange3Header;
        USRange3Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange3Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange3Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange3Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange3Msg.range = ((float)iUSSensors1To8[2]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor3.publish(USRange3Msg);
    }
    //----------------------------------------------------------------------------------------------------------
    //-------------------------------------------SENSOR4--------------------------------------------------------
    if(m_bSensorActive[3])
    {
        std_msgs::Header USRange4Header;
        sensor_msgs::Range USRange4Msg;
        //create USRanger4Msg
        //fill in header
        USRange4Header.seq = 1; 				//uint32
        USRange4Header.stamp = ros::Time::now(); 		//time
        USRange4Header.frame_id = "range_4_link";		//string

        USRange4Msg.header = USRange4Header;
        USRange4Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange4Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange4Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange4Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange4Msg.range = ((float)iUSSensors1To8[3]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor4.publish(USRange4Msg);
    }
    //----------------------------------------------------------------------------------------------------------
    //-------------------------------------------SENSOR5--------------------------------------------------------
    if(m_bSensorActive[4])
    {
        std_msgs::Header USRange5Header;
        sensor_msgs::Range USRange5Msg;
        //create USRanger5Msg
        //fill in header
        USRange5Header.seq = 1; 				//uint32
        USRange5Header.stamp = ros::Time::now(); 		//time
        USRange5Header.frame_id = "range_5_link";		//string

        USRange5Msg.header = USRange5Header;
        USRange5Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange5Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange5Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange5Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange5Msg.range = ((float)iUSSensors1To8[4]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor5.publish(USRange5Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR6--------------------------------------------------------
    if(m_bSensorActive[5])
    {
        std_msgs::Header USRange6Header;
        sensor_msgs::Range USRange6Msg;
        //create USRanger6Msg
        //fill in header
        USRange6Header.seq = 1; 				//uint32
        USRange6Header.stamp = ros::Time::now(); 		//time
        USRange6Header.frame_id = "range_6_link";		//string

        USRange6Msg.header = USRange6Header;
        USRange6Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange6Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange6Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange6Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange6Msg.range = ((float)iUSSensors1To8[5]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor6.publish(USRange6Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR7--------------------------------------------------------
    if(m_bSensorActive[6])
    {
        std_msgs::Header USRange7Header;
        sensor_msgs::Range USRange7Msg;
        //create USRanger7Msg
        //fill in header
        USRange7Header.seq = 1; 				//uint32
        USRange7Header.stamp = ros::Time::now(); 		//time
        USRange7Header.frame_id = "range_7_link";		//string

        USRange7Msg.header = USRange7Header;
        USRange7Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange7Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange7Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange7Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange7Msg.range = ((float)iUSSensors1To8[6]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor7.publish(USRange7Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR8--------------------------------------------------------
    if(m_bSensorActive[7])
    {
        std_msgs::Header USRange8Header;
        sensor_msgs::Range USRange8Msg;
        //create USRanger8Msg
        //fill in header
        USRange8Header.seq = 1; 				//uint32
        USRange8Header.stamp = ros::Time::now(); 		//time
        USRange8Header.frame_id = "range_8_link";		//string

        USRange8Msg.header = USRange8Header;
        USRange8Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange8Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange8Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange8Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange8Msg.range = ((float)iUSSensors1To8[7]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor8.publish(USRange8Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR9--------------------------------------------------------
    if(m_bSensorActive[8])
    {
        std_msgs::Header USRange9Header;
        sensor_msgs::Range USRange9Msg;
        //create USRanger4Msg
        //fill in header
        USRange9Header.seq = 1; 				//uint32
        USRange9Header.stamp = ros::Time::now(); 		//time
        USRange9Header.frame_id = "range_9_link";		//string

        USRange9Msg.header = USRange9Header;
        USRange9Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange9Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange9Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange9Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange9Msg.range = ((float)iUSSensors9To16[0]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor9.publish(USRange9Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR10-------------------------------------------------------
    if(m_bSensorActive[9])
    {
        std_msgs::Header USRange10Header;
        sensor_msgs::Range USRange10Msg;
        //create USRanger10Msg
        //fill in header
        USRange10Header.seq = 1; 				//uint32
        USRange10Header.stamp = ros::Time::now(); 		//time
        USRange10Header.frame_id = "range_10_link";		//string

        USRange10Msg.header = USRange10Header;
        USRange10Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange10Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange10Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange10Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange10Msg.range = ((float)iUSSensors9To16[1]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor10.publish(USRange10Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR11-------------------------------------------------------
    if(m_bSensorActive[10])
    {
        std_msgs::Header USRange11Header;
        sensor_msgs::Range USRange11Msg;
        //create USRanger11Msg
        //fill in header
        USRange11Header.seq = 1; 				//uint32
        USRange11Header.stamp = ros::Time::now(); 		//time
        USRange11Header.frame_id = "range_11_link";		//string

        USRange11Msg.header = USRange11Header;
        USRange11Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange11Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange11Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange11Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange11Msg.range = ((float)iUSSensors9To16[2]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor11.publish(USRange11Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR12-------------------------------------------------------
    if(m_bSensorActive[11])
    {
        std_msgs::Header USRange12Header;
        sensor_msgs::Range USRange12Msg;
        //create USRanger12Msg
        //fill in header
        USRange12Header.seq = 1; 				//uint32
        USRange12Header.stamp = ros::Time::now(); 		//time
        USRange12Header.frame_id = "range_12_link";		//string

        USRange12Msg.header = USRange12Header;
        USRange12Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange12Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange12Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange12Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange12Msg.range = ((float)iUSSensors9To16[3]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor12.publish(USRange12Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR13-------------------------------------------------------
    if(m_bSensorActive[12])
    {
        std_msgs::Header USRange13Header;
        sensor_msgs::Range USRange13Msg;
        //create USRanger11Msg
        //fill in header
        USRange13Header.seq = 1; 				//uint32
        USRange13Header.stamp = ros::Time::now(); 		//time
        USRange13Header.frame_id = "range_13_link";		//string

        USRange13Msg.header = USRange13Header;
        USRange13Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange13Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange13Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange13Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange13Msg.range = ((float)iUSSensors9To16[4]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor13.publish(USRange13Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR14-------------------------------------------------------
    if(m_bSensorActive[13])
    {
        std_msgs::Header USRange14Header;
        sensor_msgs::Range USRange14Msg;
        //create USRanger14Msg
        //fill in header
        USRange14Header.seq = 1; 				//uint32
        USRange14Header.stamp = ros::Time::now(); 		//time
        USRange14Header.frame_id = "range_14_link";		//string

        USRange14Msg.header = USRange14Header;
        USRange14Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange14Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange14Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange14Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange14Msg.range = ((float)iUSSensors9To16[5]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor14.publish(USRange14Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR15-------------------------------------------------------
    if(m_bSensorActive[14])
    {
        std_msgs::Header USRange15Header;
        sensor_msgs::Range USRange15Msg;
        //create USRanger15Msg
        //fill in header
        USRange15Header.seq = 1; 				//uint32
        USRange15Header.stamp = ros::Time::now(); 		//time
        USRange15Header.frame_id = "range_15_link";		//string

        USRange15Msg.header = USRange15Header;
        USRange15Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange15Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange15Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange15Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange15Msg.range = ((float)iUSSensors9To16[6]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor15.publish(USRange15Msg);
    }
    //------------------------------------------------------------------------------------------------------------

    //-------------------------------------------SENSOR16-------------------------------------------------------
    if(m_bSensorActive[15])
    {
        std_msgs::Header USRange16Header;
        sensor_msgs::Range USRange16Msg;
        //create USRanger16Msg
        //fill in header
        USRange16Header.seq = 1; 				//uint32
        USRange16Header.stamp = ros::Time::now(); 		//time
        USRange16Header.frame_id = "range_16_link";		//string

        USRange16Msg.header = USRange16Header;
        USRange16Msg.radiation_type = 0; 			//uint8   => Enum ULTRASOUND=0; INFRARED=1
        USRange16Msg.field_of_view = 1.05; 			//float32 [rad]
        USRange16Msg.min_range = m_dSensorMinRange; 		//float32 [m]
        USRange16Msg.max_range = m_dSensorMaxRange; 		//float32 [m]
        USRange16Msg.range = ((float)iUSSensors9To16[7]/100); 	//float32 [cm] => [m]

        //publish data for first USrange sensor
        topicPub_USRangeSensor16.publish(USRange16Msg);
    }
    //------------------------------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------
bool neo_usboard_node::requestParameterSet()
{
    /* This functions wraps requesting ParameterSets for Serial and CAN Interfaces
     *
     */
    if(m_bUseCANInterface)
    {
        m_CANUSBoard->reqestParameterSet();
    }
    else
    {
        //TODO
        ROS_WARN("USBoard: requesting ParameterSets for serial connection not implemented yet!");
    }
    return true;
}

/************************************************************************************************
 *
 *
 *
 *
 *
 *
 *
 *
 *                                        OLD!!!!!
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * ************************************************************************************************/

//--------------------------------------------------------------------------------

int neo_usboard_node::requestBoardStatus()
{

    int ret;
    // Request Status of USBoard
    ret = m_SerUSBoard->sendCmdConnect();
    ros::Duration(0.01).sleep();  // transmission command interval time

    if(ret != SerUSBoard::NO_ERROR) {
            ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
    }

    ret = m_SerUSBoard->eval_RXBuffer();
    if(ret==SerUSBoard::NOT_INITIALIZED) {
            ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
            usboard_online = false;
    } else if(ret==SerUSBoard::NO_MESSAGES) {
            ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
            if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
    } else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
            ROS_ERROR("USBoard: Too less bytes in queue");
    } else if(ret==SerUSBoard::CHECKSUM_ERROR) {
            ROS_ERROR("A checksum error occurred while reading from usboard data");
    } else if(ret==SerUSBoard::NO_ERROR) {

            usboard_online = true;
            usboard_available = true;
            time_last_message_received_ = ros::Time::now();

    }
    return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestActivateChannels()
{
    int ret;
    //Activate the USBoard Sensors
    ret = m_SerUSBoard->sendCmdSetChannelActive();
    ros::Duration(0.01).sleep(); // transmission command interval time

    if(ret != SerUSBoard::NO_ERROR) {
                    ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
    }

    ROS_INFO("Reqesting active channels is not implemented yet");

    return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestSensorReadings1TO8()
{
    int ret;
    //Request Sensor 1 to 8 readings
    ret = m_SerUSBoard->sendCmdGetData1To8();
    ros::Duration(0.02).sleep(); // transmission command interval time

    if(ret != SerUSBoard::NO_ERROR) {
            ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
    }

    ret = m_SerUSBoard->eval_RXBuffer();
    if(ret==SerUSBoard::NOT_INITIALIZED) {
            ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
            usboard_online = false;
    } else if(ret==SerUSBoard::NO_MESSAGES) {
            ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
            if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
    } else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
    //ROS_ERROR("USBoard 1-8: Too less bytes in queue");
    } else if(ret==SerUSBoard::CHECKSUM_ERROR) {
            ROS_ERROR("A checksum error occurred while reading from usboard data");
    } else if(ret==SerUSBoard::NO_ERROR) {
            usboard_online = true;
            usboard_available = true;
            time_last_message_received_ = ros::Time::now();

    }

    return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestSensorReadings9TO16()
{
    int ret;
    //Request Sensor 9 to 16 readings
    ret = m_SerUSBoard->sendCmdGetData9To16();
    ros::Duration(0.02).sleep(); // transmission command interval time

    if(ret != SerUSBoard::NO_ERROR) {
            ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
    }

    ret = m_SerUSBoard->eval_RXBuffer();
    if(ret==SerUSBoard::NOT_INITIALIZED) {
            ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
            usboard_online = false;
    } else if(ret==SerUSBoard::NO_MESSAGES) {
            ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
            if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
    } else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
    //ROS_ERROR("USBoard 9-16: Too less bytes in queue");
    } else if(ret==SerUSBoard::CHECKSUM_ERROR) {
            ROS_ERROR("A checksum error occurred while reading from usboard data");
    } else if(ret==SerUSBoard::NO_ERROR) {
            usboard_online = true;
            usboard_available = true;
            time_last_message_received_ = ros::Time::now();

    }
    return 0;
}

//--------------------------------------------------------------------------------

int neo_usboard_node::requestAnalogreadings()
{
    int ret;
    //Request Analog readings
    ret = m_SerUSBoard->sendCmdGetAnalogIn();
    ros::Duration(0.01).sleep(); // transmission command interval time

    if(ret != SerUSBoard::NO_ERROR) {
            ROS_ERROR("Error in sending message to USboard over SerialIO, lost bytes during writing");
    }

    ret = m_SerUSBoard->eval_RXBuffer();
    if(ret==SerUSBoard::NOT_INITIALIZED) {
            ROS_ERROR("Failed to read USBoard data over Serial, the device is not initialized");
            usboard_online = false;
    } else if(ret==SerUSBoard::NO_MESSAGES) {
            ROS_ERROR("For a long time, no messages from USBoard have been received, check com port!");
            if(time_last_message_received_.toSec() - ros::Time::now().toSec() > usboard_timeout_) {usboard_online = false;}
    } else if(ret==SerUSBoard::TOO_LESS_BYTES_IN_QUEUE) {
            //ROS_ERROR("USBoard: Too less bytes in queue");
    } else if(ret==SerUSBoard::CHECKSUM_ERROR) {
            ROS_ERROR("A checksum error occurred while reading from usboard data");
    } else if(ret==SerUSBoard::NO_ERROR) {
            usboard_online = true;
            usboard_available = true;
            time_last_message_received_ = ros::Time::now();

    }
    return 0;
}
