# ROS-Node for Neobotix USBoard

This node handles parameters and communication of the Neobotix USBoard.
The USBoard is available as standalone unit and in all Neobotix mobile robots (http://www.neobotix-robots.com/components.html).

## Neobotix USBoard features

![Neobotix USBoard](http://www.neobotix-roboter.de/fileadmin/files/produkte/Komponenten/USBoard.jpg)

* Users can connect up to 16 Bosch ParkPilot URF6 / USS4 ultrasonic sensors to the USBoard and read their measurements via CAN and RS232. This allows to easily fill gaps that cannot be covered by other sensors.
* In standalone mode two thresholds (warning and alarm) can be defined for each sensor individually. Two relays signal the detection of an obstacle within the warning or alarm range of any active sensor.
* Configuration of the Neobotix USBoard is done via an easy to use graphical user interface that is included in delivery. The user interface runs on any Java capable computer with an RS232 port and makes configuration and monitoring fast and convenient.
* For more information please take a look at the datasheet: http://www.neobotix-robots.com/fileadmin/files/downloads/Datenblaetter/USBoard-Datenblatt.pdf

## Distributors
Neobotix USBoard and accessories are available at:

* Neobotix GmbH (http://www.neobotix-robots.com/company-contact.html)
* Customers in the U.S. please contact our local distributor AutonomouStuff (http://autonomoustuff.com/product/ultrasonic/).

## Parameters and Usage

### Usage

Tested with: ROS Indigo on Ubuntu 14.04

1. Clone the repository into your catkin workspace src folder ($ git clone https://github.com/neobotix/neo_usboard.git)
2. Check the ttyUSB port on which the USBoard is connected to the PC via RS-232 (generally /dev/ttyUSB0)
3. Define parameters in launch/usboard_param.yaml 
3. Make sure that neo_msgs package is added to your catkin workspace
4. Build your catkin workspace
5. Run the neo_usboard roslaunch file ( $ roslaunch neo_usboard neo_usboard.launch)
6. To print and montior the published sensors data use rostopic or rqt ($ rostopic echo /USBoard/Measurements)

### Topics

| Name | Type |
| --- | --- |
| /USBoard/Measurements | neo_msgs/USBoard |
| /USBoard/Sensor1 | sensor_msgs/Range |
| /USBoard/Sensor2 | sensor_msgs/Range |
| /USBoard/Sensor3 | sensor_msgs/Range |
| /USBoard/Sensor4 | sensor_msgs/Range |
| /USBoard/Sensor5 | sensor_msgs/Range |
| /USBoard/Sensor6 | sensor_msgs/Range |
| /USBoard/Sensor7 | sensor_msgs/Range |
| /USBoard/Sensor8 | sensor_msgs/Range |
| /USBoard/Sensor9 | sensor_msgs/Range |
| /USBoard/Sensor10 | sensor_msgs/Range |
| /USBoard/Sensor11 | sensor_msgs/Range |
| /USBoard/Sensor12 | sensor_msgs/Range |
| /USBoard/Sensor13 | sensor_msgs/Range |
| /USBoard/Sensor14 | sensor_msgs/Range |
| /USBoard/Sensor15 | sensor_msgs/Range |
| /USBoard/Sensor16 | sensor_msgs/Range |


### Parameters

| Parameter | Default Value | Description |
| --- | --- | --- |
| ComPort | /dev/ttyUSB0 | Port the Neobotix USBoard is connected to |
| usboard_timeout | 0.5 | Timeout in [s] |
| requestRate | 5 | Request and Publish rate of sensor values in [Hz] (5Hz maximum) |
| log | false | Write raw data for debugging to file |
| sensor1_active | true | defines if sensor_msgs/Range Topic for ultrasonic sensor 1 should be published |
| sensor2_active | true | defines if sensor_msgs/Range Topic for ultrasonic sensor 2 should be published |
| sensor3_active | true | defines if sensor_msgs/Range Topic for ultrasonic sensor 3 should be published |
| sensor4_active | true | defines if sensor_msgs/Range Topic for ultrasonic sensor 4 should be published |
| sensor5_active | true | defines if sensor_msgs/Range Topic for ultrasonic sensor 5 should be published |
| sensor6_active | true | defines if sensor_msgs/Range Topic for ultrasonic sensor 6 should be published |
| sensor7_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 7 should be published |
| sensor8_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 8 should be published |
| sensor9_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 9 should be published |
| sensor10_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 10 should be published |
| sensor11_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 11 should be published |
| sensor12_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 12 should be published |
| sensor13_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 13 should be published |
| sensor14_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 14 should be published |
| sensor15_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 15 should be published |
| sensor16_active | false | defines if sensor_msgs/Range Topic for ultrasonic sensor 16 should be published |
