cmake_minimum_required(VERSION 3.25.1)
project(xesc_msgs)
set(CMAKE_CXX_STANDARD 17)

find_package(rosidl_default_generators)

find_package(std_msgs REQUIRED)

#add_message_files(
#  DIRECTORY msg
#  FILES XescState.msg XescStateStamped.msg
#)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/XescState.msg"
        "msg/XescStateStamped.msg"
        DEPENDENCIES std_msgs
)
#
#catkin_package(
#  CATKIN_DEPENDS std_msgs message_runtime
#)

ament_package()