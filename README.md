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

* Neobotix GmbH (http://www.neobotix-robots.com)
* Customers in the U.S. please contact our local distributor AutonomouStuff (http://autonomoustuff.com/product/ultrasonic/).

## Versions

Tested with:

* ROS Indigo on Ubuntu 14.04
* ROS Kinetic on Ubuntu 16.04

## Installation

### Third Party Packages
```sudo apt-get install ros-kinetic-ros-canopen```

### Neobotix Packages

``` git clone https://github.com/neobotix/neo_msgs.git ```

``` git clone https://github.com/neobotix/neo_usboard.git ```

## Parameters

| Parameter | Default Value | Description |
| --- | --- | --- |
| useCAN | true | CAN is the prefered interface to communicate with USBoard. Other option is RS232. |
| comPort | /dev/ttyUSB0 | Port for RS232 connection. |
| mode | 1 | Available modes are Request (mode = 0) and Automatic (mode = 1). In request mode the node offers a ROS-Service to read data from USBoard and publish it. In automatic mode the node is waiting for data from USBoard and will publish it immediately. **Please make sure to configure the USBoard Firmware to the correct settings!**  |
| timeout | 1.0 | Print warning, if no message was received for longer than the timeout. |

## Usage

### CAN-Interface

```sudo ip link set can0 up type can bitrate 1000000```

### Launch

```roslaunch neo_usboard neo_usboard.launch```

## Warranty
This software is provided by the copyright holders and contributors "as is" and any express or implied warranties, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose are disclaimed. In no event shall the copyright owner or contributors be liable for any direct, indirect, incidental, special, exemplary, or consequential damages (including, but not limited to, procurement of substitute goods or services; loss of use, data, or profits; or business interruption) however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence or otherwise) arising in any way out of the use of this software, even if advised of the possibility of such damage.
