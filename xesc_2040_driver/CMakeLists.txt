cmake_minimum_required(VERSION 3.25.1)
project(xesc_2040_driver)

set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(xesc_interface REQUIRED)


###########
## Build ##
###########


# driver libraries
add_library(xesc_2040_driver
        src/xesc_2040_interface.cpp
        src/xesc_2040_driver.cpp
        include/xesc_2040_driver/COBS.h
        include/xesc_2040_driver/xesc_2040_datatypes.h include/xesc_2040_driver/xesc_2040_driver.h)


target_include_directories(
        xesc_2040_driver
        PUBLIC
        include
)
ament_target_dependencies(xesc_2040_driver
        rclcpp
        std_msgs
        xesc_msgs
        serial
        xesc_interface
)

install(
        DIRECTORY include/
        DESTINATION include
)

## Install executable
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)


ament_export_dependencies(pluginlib std_msgs xesc_msgs serial)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})


ament_package()