cmake_minimum_required(VERSION 3.0.2)
project(robot_driver)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        message_generation
        roscpp
        serial
        std_msgs
        sensor_msgs
        rm_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  vision_tx_data.msg
  vision_rx_data.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs 
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rm_bringup
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
 DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/serialPort.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
