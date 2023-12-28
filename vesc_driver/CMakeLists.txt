cmake_minimum_required(VERSION 3.25.1)
project(vesc_driver)

set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(xesc_interface REQUIRED)


# driver libraries
add_library(vesc_driver
        src/vesc_driver.cpp
        src/vesc_interface.cpp
        src/vesc_packet.cpp
        src/vesc_packet_factory.cpp
)

target_include_directories(
        vesc_driver
        PUBLIC
        include
)
ament_target_dependencies(vesc_driver
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