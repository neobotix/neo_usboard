cmake_minimum_required(VERSION 2.8.3)
project(neo_usboard)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  neo_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neo_usboard
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(SerialIO_USBoard common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerUSBoard.cpp common/src/neo_usboard_lib.cpp common/src/CANUSBoard.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(SerialIO_USBoard ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} SerialIO_USBoard)

add_executable(neo_usboard_node src/neo_usboard_node.cpp)
target_link_libraries(neo_usboard_node ${PROJECT_NAME})
target_link_libraries(neo_usboard_node ${catkin_LIBRARIES})
add_dependencies(neo_usboard_node ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############



#############
## Testing ##
#############




